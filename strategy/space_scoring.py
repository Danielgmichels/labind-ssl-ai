from __future__ import annotations
import math
import time
from typing import List, Optional, Tuple, Dict
from strategy.tactical_utils import PositionCandidate, PositionDecision, TACTICAL_CONFIG

_last_space_log_time = 0.0
_last_logged_point: Optional[Tuple[float, float]] = None


def generate_free_space_candidates(world_model, blackboard, robot_id: int, side_y: Optional[float] = None) -> List[PositionCandidate]:
    """
    Gera pontos candidatos sobre uma grade regular (grid) na região ofensiva/intermediária do campo.
    Filtra pontos fora dos limites de segurança e dentro das áreas de penalidade (geofencing).
    Se side_y for informado, restringe a busca à respectiva ala (esquerda se side_y > 0, direita se side_y < 0).
    """
    config = TACTICAL_CONFIG["FREE_SPACE"]
    res = config.get("grid_resolution", 0.50)

    goal_x = world_model.field.enemy_goal_x
    goal_dir = 1.0 if goal_x > 0 else -1.0

    # Limites longitudinais: do meio-campo ligeiramente recuado até a entrada da grande área adversária
    x_min = min(0.0, goal_dir * 1.0) if goal_dir > 0 else -4.8
    x_max = 4.8 if goal_dir > 0 else max(0.0, goal_dir * 1.0)
    
    # Se goal_dir for negativo (lado esquerdo):
    if goal_dir < 0:
        x_start, x_end = -4.8, 1.0
    else:
        x_start, x_end = -1.0, 4.8

    # Limites laterais:
    if side_y is not None and abs(side_y) > 0.1:
        if side_y > 0:
            y_start, y_end = 0.5, 3.8
        else:
            y_start, y_end = -3.8, -0.5
    else:
        y_start, y_end = -3.8, 3.8

    candidates = []
    curr_x = x_start
    while curr_x <= x_end + 1e-4:
        curr_y = y_start
        while curr_y <= y_end + 1e-4:
            # Exclusão de grande área
            na_area_esq = (-6.0 <= curr_x <= -4.8) and (-1.4 <= curr_y <= 1.4)
            na_area_dir = (4.8 <= curr_x <= 6.0) and (-1.4 <= curr_y <= 1.4)

            if not na_area_esq and not na_area_dir:
                candidates.append(PositionCandidate(target_point=(round(curr_x, 2), round(curr_y, 2))))
            
            curr_y += res
        curr_x += res

    return candidates


def evaluate_position(candidate: PositionCandidate, shared_data: dict) -> float:
    """Calcula a pontuação ponderada multi-critério para uma posição do grid."""
    weights = TACTICAL_CONFIG["FREE_SPACE"]["weights"]

    candidate.space_score = _calc_space(candidate.target_point, shared_data)
    candidate.pass_line_score = _calc_pass_line(candidate.target_point, shared_data)
    candidate.support_score = _calc_support(candidate.target_point, shared_data)
    candidate.progression_score = _calc_progression(candidate.target_point, shared_data)
    candidate.defensive_cover_score = _calc_defensive_cover(candidate.target_point, shared_data)
    candidate.safety_score = _calc_safety(candidate.target_point, shared_data)

    # REGRA CRÍTICA: Se a linha de passe estiver totalmente bloqueada, penaliza drasticamente
    if candidate.pass_line_score <= 0.0:
        candidate.score = 0.10 * candidate.space_score # Score residual baixo
        candidate.reason = "Linha de passe bloqueada por obstáculo."
        return candidate.score

    candidate.score = (
        (candidate.space_score * weights["space"]) +
        (candidate.pass_line_score * weights["pass_line"]) +
        (candidate.support_score * weights["support"]) +
        (candidate.progression_score * weights["progression"]) +
        (candidate.defensive_cover_score * weights["defensive_cover"]) +
        (candidate.safety_score * weights["safety"])
    )

    min_score = TACTICAL_CONFIG["FREE_SPACE"]["min_score"]
    if candidate.score < min_score:
        candidate.reason = f"Score ({candidate.score:.2f}) abaixo do limiar ({min_score:.2f})."

    return candidate.score


def get_best_free_space_decision(world_model, blackboard, robot_id: int,
                                 current_target: Optional[Tuple[float, float]] = None,
                                 side_y: Optional[float] = None) -> PositionDecision:
    """
    Avalia a grade espacial e seleciona a melhor coordenada livre para recepção e apoio ofensivo.
    Aplica histerese (stability_bonus e switch_margin) para evitar oscilações a 60 Hz.
    """
    global _last_space_log_time, _last_logged_point

    ball = world_model.get_ball()
    bx, by = (ball.x, ball.y) if ball.visible else (0.0, 0.0)
    goal_x = world_model.field.enemy_goal_x

    opponents = [(opp.x, opp.y) for opp in world_model.get_opponent_team().values() if opp.visible]
    teammates = [
        (mate.x, mate.y)
        for mate in world_model.get_my_team().values()
        if mate.visible and mate.id != robot_id
    ]

    shared_data = {
        'ball_pos': (bx, by),
        'goal_x': goal_x,
        'opponents': opponents,
        'teammates': teammates,
        'robot_id': robot_id
    }

    candidates = generate_free_space_candidates(world_model, blackboard, robot_id, side_y=side_y)
    if not candidates:
        return PositionDecision()

    for candidate in candidates:
        evaluate_position(candidate, shared_data)

    config = TACTICAL_CONFIG["FREE_SPACE"]
    stability_bonus = config.get("stability_bonus", 0.15)
    switch_margin = config.get("switch_margin", 0.05)
    min_score = config.get("min_score", 0.35)

    # Identifica o candidato correspondente à posição atual se houver
    current_candidate = None
    if current_target is not None:
        cx, cy = current_target
        current_candidate = min(
            candidates,
            key=lambda c: math.hypot(c.target_point[0] - cx, c.target_point[1] - cy)
        )
        # Se a posição atual estiver razoavelmente próxima de um nó da grade
        if math.hypot(current_candidate.target_point[0] - cx, current_candidate.target_point[1] - cy) < 0.60:
            current_candidate.score += stability_bonus

    # Escolhe o melhor candidato
    best_candidate = max(candidates, key=lambda c: c.score)

    # Histerese de troca: só substitui a posição atual se superar o limiar com folga
    chosen_candidate = best_candidate
    if current_candidate is not None:
        if best_candidate.target_point != current_candidate.target_point:
            if best_candidate.score < (current_candidate.score + switch_margin):
                chosen_candidate = current_candidate

    decision = PositionDecision(all_candidates=candidates)
    if chosen_candidate.score >= min_score:
        decision.best_candidate = chosen_candidate
        decision.best_target_point = chosen_candidate.target_point

    # Throttled logging (1 Hz ou mudança de ponto)
    now = time.time()
    if (now - _last_space_log_time > 1.0) or (decision.best_target_point != _last_logged_point):
        _last_space_log_time = now
        _last_logged_point = decision.best_target_point
        if decision.best_candidate:
            c = decision.best_candidate
            print(f"[FreeSpace] Posição Escolhida: target={c.target_point} | score={c.score:.2f}")
            print(f"  Componentes: space={c.space_score:.2f} pass_line={c.pass_line_score:.2f} sup={c.support_score:.2f} prog={c.progression_score:.2f} def={c.defensive_cover_score:.2f} safe={c.safety_score:.2f}")
        else:
            print(f"[FreeSpace] Nenhuma posição aprovada acima de {min_score:.2f}.")

    return decision


# --- Funções dos Componentes de Avaliação (0.0 a 1.0) ---

def _calc_space(point: Tuple[float, float], data: dict) -> float:
    """Avalia o isolamento do ponto em relação aos adversários mais próximos."""
    px, py = point
    opponents = data['opponents']
    if not opponents:
        return 1.0

    min_opp_dist = min(math.hypot(ox - px, oy - py) for ox, oy in opponents)
    # A menos de 40cm é perigo extremo (score=0), a partir de 1.8m é espaço aberto pleno (score=1)
    if min_opp_dist <= 0.40:
        return 0.0
    score = (min_opp_dist - 0.40) / (1.80 - 0.40)
    return max(0.0, min(1.0, score))


def _calc_pass_line(point: Tuple[float, float], data: dict) -> float:
    """Calcula se há linha de passe desimpedida da bola até o ponto do grid."""
    bx, by = data['ball_pos']
    px, py = point
    opponents = data['opponents']

    dx = px - bx
    dy = py - by
    line_len_sq = dx*dx + dy*dy
    if line_len_sq < 0.01:
        return 1.0

    min_dist_to_line = float('inf')
    for ox, oy in opponents:
        # Projeção no segmento bola -> ponto
        t = ((ox - bx) * dx + (oy - by) * dy) / line_len_sq
        t = max(0.0, min(1.0, t))
        proj_x = bx + t * dx
        proj_y = by + t * dy

        dist = math.hypot(ox - proj_x, oy - proj_y)
        if dist < min_dist_to_line:
            min_dist_to_line = dist

    collision_radius = TACTICAL_CONFIG["FREE_SPACE"].get("collision_radius", 0.18)
    safe_margin = TACTICAL_CONFIG["FREE_SPACE"].get("safe_margin", 0.45)

    if min_dist_to_line <= collision_radius:
        return 0.0

    score = (min_dist_to_line - collision_radius) / (safe_margin - collision_radius)
    return max(0.0, min(1.0, score))


def _calc_support(point: Tuple[float, float], data: dict) -> float:
    """Avalia a distância ideal de apoio em relação à bola (nem colado, nem isolado)."""
    bx, by = data['ball_pos']
    px, py = point
    dist = math.hypot(px - bx, py - by)

    # Distância ideal: 1.8m a 3.5m
    if dist < 0.8:
        return 0.20
    elif 1.8 <= dist <= 3.5:
        return 1.00
    elif dist < 1.8:
        return 0.20 + (dist - 0.8) / (1.8 - 0.8) * 0.80
    else:
        # Decai suavemente acima de 3.5m até 6.0m
        score = 1.00 - ((dist - 3.5) / 3.0)
        return max(0.10, score)


def _calc_progression(point: Tuple[float, float], data: dict) -> float:
    """Avalia o avanço longitudinal em direção à meta adversária."""
    px, _ = point
    goal_x = data['goal_x']
    if abs(goal_x) < 0.001:
        return 0.0

    # Normalizado: 0.0 no meio do campo, 1.0 na linha de fundo adversária
    goal_dir = 1.0 if goal_x > 0 else -1.0
    prog = (goal_dir * px) / abs(goal_x)
    return max(0.0, min(1.0, prog))


def _calc_defensive_cover(point: Tuple[float, float], data: dict) -> float:
    """Penaliza posições excessivamente avançadas sem retorno defensivo."""
    px, _ = point
    bx, _ = data['ball_pos']
    goal_x = data['goal_x']

    goal_dir = 1.0 if goal_x > 0 else -1.0
    # Se o ponto estiver ligeiramente atrás ou alinhado com a bola, dá cobertura defensiva
    delta = goal_dir * (px - bx)
    if delta <= 0.5:
        return 1.00
    elif delta <= 2.0:
        return 0.70
    else:
        return 0.40


def _calc_safety(point: Tuple[float, float], data: dict) -> float:
    """Penaliza proximidade extrema das bordas laterais do campo."""
    _, py = point
    # Campo tem largura total de 9m (Y de -4.5 a 4.5)
    dist_to_sideline = 4.5 - abs(py)
    if dist_to_sideline >= 0.80:
        return 1.00
    elif dist_to_sideline <= 0.20:
        return 0.10
    else:
        return (dist_to_sideline - 0.20) / (0.80 - 0.20)


from __future__ import annotations
import math
import time
from typing import List, Optional, Dict, Tuple
from strategy.tactical_utils import PassCandidate, PassDecision, TACTICAL_CONFIG

# Variáveis para controle de taxa de logs (evitar flooding a 60 Hz)
_last_pass_log_time = 0.0
_last_logged_receiver_id: Optional[int] = None

def get_eligible_receivers(world_model, blackboard, passer_id: int) -> List[Tuple[object, str]]:
    """
    Identifica companheiros elegíveis para receber passe:
    - Visíveis no campo
    - Diferentes do próprio passador
    - Diferentes do goleiro (ID 0 ou papel GOLEIRO)
    - Distância dentro da faixa operacional configurada
    """
    config = TACTICAL_CONFIG["PASS"]
    min_dist = config.get("min_distance", 0.70)
    max_dist = config.get("max_distance", 6.00)

    passer = world_model.get_robot(passer_id)
    if not passer or not passer.visible:
        return []

    papeis = getattr(blackboard, 'papeis', {})
    eligible = []

    for robot in world_model.get_my_team().values():
        if not robot.visible or robot.id == passer_id:
            continue

        role = papeis.get(robot.id, "ESPERA")
        if robot.id == 0 or role in ["GOLEIRO", "ESPERA"]:
            continue

        dist = math.hypot(robot.x - passer.x, robot.y - passer.y)
        if min_dist <= dist <= max_dist:
            eligible.append((robot, role))

    return eligible

def generate_pass_candidates(world_model, blackboard, passer_id: int) -> List[PassCandidate]:
    """Gera um PassCandidate para cada receptor elegível."""
    eligible_receivers = get_eligible_receivers(world_model, blackboard, passer_id)
    candidates = []
    
    for robot, role in eligible_receivers:
        target_point = (robot.x, robot.y)
        candidates.append(PassCandidate(
            receiver_id=robot.id,
            receiver_role=role,
            target_point=target_point
        ))
        
    return candidates

def evaluate_pass(candidate: PassCandidate, shared_data: dict) -> float:
    """Calcula os componentes de score e a pontuação ponderada do passe."""
    weights = TACTICAL_CONFIG["PASS"]["weights"]
    
    candidate.progression_score = _calc_progression(candidate.target_point, shared_data)
    candidate.space_score = _calc_space(candidate.target_point, shared_data)
    candidate.reception_angle_score = _calc_reception_angle(candidate, shared_data)
    candidate.distance_score = _calc_distance(candidate.target_point, shared_data)
    candidate.support_score = _calc_support(candidate.receiver_role)
    candidate.interception_score = _calc_interception(candidate.target_point, candidate.receiver_id, shared_data)
    candidate.density_score = _calc_density(candidate.target_point, shared_data)

    # REGRA CRÍTICA DE INTERCEPTAÇÃO:
    # Se a trajetória entre passador e receptor estiver fisicamente bloqueada por um adversário ou aliado,
    # o passe é invalidado (score 0.0) para não perder a bola de graça.
    if candidate.interception_score <= 0.0:
        candidate.score = 0.0
        candidate.reason = "Trajetória de passe fisicamente bloqueada/interceptada."
        return 0.0

    candidate.score = (
        (candidate.progression_score * weights["progression"]) +
        (candidate.space_score * weights["space"]) +
        (candidate.reception_angle_score * weights["reception_angle"]) +
        (candidate.distance_score * weights["distance"]) +
        (candidate.support_score * weights["support"]) +
        (candidate.interception_score * weights["interception"]) +
        (candidate.density_score * weights["density"])
    )

    min_score = TACTICAL_CONFIG["PASS"]["min_score"]
    if candidate.score < min_score:
        candidate.reason = f"Score ({candidate.score:.2f}) abaixo do limiar aceitável ({min_score:.2f})."

    return candidate.score

def get_best_pass_decision(world_model, blackboard, robot_id: int, only_forward: bool = False) -> PassDecision:
    """
    Avalia todos os receptores elegíveis e retorna a melhor decisão de passe estruturada.
    Se only_forward for True, considera apenas passes com progressão positiva (para frente).
    """
    global _last_pass_log_time, _last_logged_receiver_id

    passer = world_model.get_robot(robot_id)
    if not passer or not passer.visible:
        return PassDecision()

    ball = world_model.get_ball()
    passer_pos = (passer.x, passer.y)
    ball_pos = (ball.x, ball.y) if ball.visible else passer_pos
    goal_x = world_model.field.enemy_goal_x

    opponents = [(opp.x, opp.y) for opp in world_model.get_opponent_team().values() if opp.visible]
    teammates = [(mate.x, mate.y) for mate in world_model.get_my_team().values() if mate.visible]

    shared_data = {
        'passer_id': robot_id,
        'passer_pos': passer_pos,
        'ball_pos': ball_pos,
        'goal_x': goal_x,
        'opponents': opponents,
        'teammates': teammates,
        'world_model': world_model
    }

    candidates = generate_pass_candidates(world_model, blackboard, robot_id)
    if not candidates:
        return PassDecision()

    for candidate in candidates:
        evaluate_pass(candidate, shared_data)

    considered_candidates = candidates
    if only_forward:
        considered_candidates = [c for c in candidates if c.progression_score >= 0.50]

    decision = PassDecision(all_candidates=candidates)
    if considered_candidates:
        best_candidate = max(considered_candidates, key=lambda c: c.score)
        min_score = TACTICAL_CONFIG["PASS"]["min_score"]
        if best_candidate.score >= min_score:
            decision.best_candidate = best_candidate
            decision.best_target_robot = best_candidate.receiver_id
            decision.best_target_point = best_candidate.target_point

    # Logging com taxa controlada (1x por segundo ou quando a decisão muda)
    now = time.time()
    if (now - _last_pass_log_time > 1.0) or (decision.best_target_robot != _last_logged_receiver_id):
        _last_pass_log_time = now
        _last_logged_receiver_id = decision.best_target_robot
        if decision.best_candidate:
            c = decision.best_candidate
            tag = "Passe Avançado" if only_forward else "Passe Aprovado"
            print(f"[PassScoring] {tag}: Receptor={c.receiver_id} ({c.receiver_role}) | score={c.score:.2f}")
            print(f"  Componentes: prog={c.progression_score:.2f} space={c.space_score:.2f} ang={c.reception_angle_score:.2f} dist={c.distance_score:.2f} sup={c.support_score:.2f} interc={c.interception_score:.2f} dens={c.density_score:.2f}")
        else:
            min_score = TACTICAL_CONFIG["PASS"]["min_score"]
            print(f"[PassScoring] Nenhum passe viável (only_forward={only_forward}) com score >= {min_score:.2f}. Mantendo condução/posse.")

    return decision

# --- Funções de Cálculo dos Componentes (0.0 a 1.0) ---

def _calc_progression(target_point: Tuple[float, float], data: dict) -> float:
    """
    Avalia o ganho de avanço longitudinal em direção à meta adversária.
    Passe para frente ganha pontuação alta (0.50 a 1.0).
    Passe lateral recebe ~0.50.
    Passe recuado recebe pontuação menor (0.0 a 0.35), especialmente no campo de ataque,
    mantendo-se viável para girar o jogo mas sem superar opções ofensivas.
    """
    px, _ = data['passer_pos']
    rx, _ = target_point
    goal_x = data['goal_x']

    goal_dir = 1.0 if goal_x > 0 else -1.0
    delta_progression = goal_dir * (rx - px)

    if delta_progression >= 0.0:
        # Avanço positivo: até 1.0 para 2.5m de ganho à frente
        score = 0.50 + min(0.50, delta_progression / 3.0)
    else:
        # Passe recuado: penaliza mais se já estivermos na metade de ataque
        in_offensive_half = (goal_dir * px) > 0.0
        max_backward = 0.30 if in_offensive_half else 0.40
        score = max(0.0, max_backward - (abs(delta_progression) / 3.5))

    return max(0.0, min(1.0, score))

def _calc_space(target_point: Tuple[float, float], data: dict) -> float:
    """Calcula o espaço livre do receptor em relação ao adversário mais próximo."""
    rx, ry = target_point
    opponents = data['opponents']

    if not opponents:
        return 1.0

    closest_dist = min(math.hypot(ox - rx, oy - ry) for ox, oy in opponents)
    # Menos de 30cm é marcação colada (score 0), 1.5m ou mais é livre (score 1)
    score = (closest_dist - 0.30) / 1.20
    return max(0.0, min(1.0, score))

def _calc_reception_angle(candidate: PassCandidate, data: dict) -> float:
    """Calcula a facilidade de recepção baseada na orientação do robô receptor."""
    world_model = data['world_model']
    robot = world_model.get_robot(candidate.receiver_id)
    if not robot or not robot.visible:
        return 0.50

    px, py = data['passer_pos']
    rx, ry = candidate.target_point

    # Vetor do receptor para a bola/passador
    angle_to_passer = math.atan2(py - ry, px - rx)
    angle_diff = abs(math.atan2(math.sin(angle_to_passer - robot.yaw), math.cos(angle_to_passer - robot.yaw)))

    # 0 rad (de frente para o passador) -> 1.0; pi rad (completamente de costas) -> 0.0
    score = 1.0 - (angle_diff / math.pi)
    return max(0.0, min(1.0, score))

def _calc_distance(target_point: Tuple[float, float], data: dict) -> float:
    """
    Avalia a distância do passe. Faixa ideal ~1.5m a 3.5m.
    Penaliza passes muito curtos (<1m) ou excessivamente longos (>4.5m).
    """
    px, py = data['passer_pos']
    rx, ry = target_point
    dist = math.hypot(rx - px, ry - py)
    config = TACTICAL_CONFIG["PASS"]
    max_dist = config.get("max_distance", 6.00)

    if dist < 1.0:
        score = max(0.0, (dist - 0.50) / 0.50)
    elif dist <= 3.5:
        score = 1.0
    else:
        score = 1.0 - ((dist - 3.5) / (max_dist - 3.5))

    return max(0.0, min(1.0, score))

def _calc_support(role: str) -> float:
    """Prioridade tática ofensiva associada ao papel do receptor."""
    scores = TACTICAL_CONFIG["PASS"].get("role_support_scores", {})
    return scores.get(role, 0.50)

def _calc_interception(target_point: Tuple[float, float], receiver_id: int, data: dict) -> float:
    """
    Raycast contra adversários e companheiros na linha do passe.
    Retorna 0.0 se houver obstáculo dentro do raio de colisão física.
    """
    px, py = data['passer_pos']
    rx, ry = target_point

    dx = rx - px
    dy = ry - py
    line_length_sq = dx*dx + dy*dy
    if line_length_sq == 0:
        return 0.0

    passer_id = data['passer_id']
    opponents = data['opponents']
    # Aliados que não sejam nem o passador nem o receptor
    teammates = [
        (m.x, m.y) for m in data['world_model'].get_my_team().values()
        if m.visible and m.id != passer_id and m.id != receiver_id
    ]
    obstacles = opponents + teammates

    min_dist_to_line = float('inf')

    for ox, oy in obstacles:
        # Projeção no segmento entre passador e receptor
        t = ((ox - px) * dx + (oy - py) * dy) / line_length_sq
        if t < 0.05 or t > 0.95:
            # Perto demais dos extremos (tolerância do próprio robô)
            continue

        proj_x = px + t * dx
        proj_y = py + t * dy

        dist = math.hypot(ox - proj_x, oy - proj_y)
        if dist < min_dist_to_line:
            min_dist_to_line = dist

    config = TACTICAL_CONFIG["PASS"]
    collision_radius = config.get("collision_radius", 0.18)
    safe_margin = config.get("safe_margin", 0.50)

    if min_dist_to_line <= collision_radius:
        return 0.0

    score = (min_dist_to_line - collision_radius) / (safe_margin - collision_radius)
    return max(0.0, min(1.0, score))

def _calc_density(target_point: Tuple[float, float], data: dict) -> float:
    """Calcula a quantidade de adversários na vizinhança imediata do receptor."""
    rx, ry = target_point
    opponents = data['opponents']
    density_radius = TACTICAL_CONFIG["PASS"].get("density_radius", 1.20)

    count = sum(1 for ox, oy in opponents if math.hypot(ox - rx, oy - ry) <= density_radius)

    # 0 adversários -> 1.0; 1 -> 0.67; 2 -> 0.33; 3+ -> 0.0
    score = 1.0 - (count / 3.0)
    return max(0.0, min(1.0, score))

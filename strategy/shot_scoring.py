from __future__ import annotations
import math
import time
from typing import List, Optional, Dict, Tuple
from strategy.tactical_utils import ShotCandidate, ShotDecision, TACTICAL_CONFIG

# Variáveis globais para controle de taxa de logs (evitar flooding a 60 Hz)
_last_shot_log_time = 0.0
_last_logged_target_y: Optional[float] = None

def generate_shot_candidates() -> List[ShotCandidate]:
    targets = TACTICAL_CONFIG["SHOT"]["candidate_targets_y"]
    return [ShotCandidate(target_y=y) for y in targets]

def evaluate_shot(candidate: ShotCandidate, shared_data: dict) -> float:
    weights = TACTICAL_CONFIG["SHOT"]["weights"]
    
    candidate.opening_score = _calc_opening(candidate.target_y, shared_data)
    candidate.angle_score = _calc_angle(candidate.target_y, shared_data)
    candidate.distance_score = _calc_distance(candidate.target_y, shared_data)
    candidate.progression_score = _calc_progression(candidate.target_y, shared_data)
    candidate.blocking_score = _calc_blocking(candidate.target_y, shared_data)
    candidate.risk_score = _calc_risk(candidate.target_y, shared_data)

    # REGRA CRÍTICA DE BLOQUEIO:
    # Se a trajetória até o alvo estiver fisicamente bloqueada por um robô (adversário ou aliado),
    # o score é zerado para impedir chutes contra defensores ou barreiras.
    if candidate.blocking_score <= 0.0:
        candidate.score = 0.0
        candidate.reason = "Trajetória fisicamente bloqueada por obstáculo."
        return 0.0

    candidate.score = (
        (candidate.opening_score * weights["opening"]) +
        (candidate.angle_score * weights["angle"]) +
        (candidate.distance_score * weights["distance"]) +
        (candidate.progression_score * weights["progression"]) +
        (candidate.blocking_score * weights["blocking"]) +
        (candidate.risk_score * weights["risk"])
    )
    
    min_score = TACTICAL_CONFIG["SHOT"]["min_score"]
    if candidate.score < min_score:
        candidate.reason = f"Score ({candidate.score:.2f}) abaixo do limiar aceitável ({min_score:.2f})."
        
    return candidate.score

def get_best_shot_decision(world_model, robot_id: int) -> ShotDecision:
    global _last_shot_log_time, _last_logged_target_y

    goal_x = world_model.field.enemy_goal_x
    ball = world_model.get_ball()
    bx, by = ball.x, ball.y
    
    robot = world_model.get_robot(robot_id)
    rx, ry, ryaw = (robot.x, robot.y, robot.yaw) if (robot and robot.visible) else (0.0, 0.0, 0.0)
    
    # Adversários visíveis
    opponents = [(opp.x, opp.y) for opp in world_model.get_opponent_team().values() if opp.visible]
    
    # Aliados visíveis (exceto o próprio robô chutador)
    teammates = [
        (mate.x, mate.y)
        for mate in world_model.get_my_team().values()
        if mate.visible and mate.id != robot_id
    ]
    
    # Todos os obstáculos capazes de interceptar a bola
    all_obstacles = opponents + teammates
    
    shared_data = {
        'goal_x': goal_x, 'bx': bx, 'by': by,
        'rx': rx, 'ry': ry, 'ryaw': ryaw,
        'opponents': opponents,
        'obstacles': all_obstacles
    }

    candidates = generate_shot_candidates()
    
    for candidate in candidates:
        evaluate_shot(candidate, shared_data)
        
    best_candidate = max(candidates, key=lambda c: c.score)
    decision = ShotDecision(all_candidates=candidates)
    
    min_score = TACTICAL_CONFIG["SHOT"]["min_score"]
    if best_candidate.score >= min_score:
        decision.best_candidate = best_candidate
        decision.best_target_y = best_candidate.target_y

    # Logging com taxa controlada (1x por segundo ou quando a decisão muda)
    now = time.time()
    if (now - _last_shot_log_time > 1.0) or (decision.best_target_y != _last_logged_target_y):
        _last_shot_log_time = now
        _last_logged_target_y = decision.best_target_y
        if decision.best_candidate:
            c = decision.best_candidate
            print(f"[ShotScoring] Alvo Escolhido: target_y={c.target_y:.2f} | score={c.score:.2f}")
            print(f"  Componentes: open={c.opening_score:.2f} ang={c.angle_score:.2f} dist={c.distance_score:.2f} prog={c.progression_score:.2f} block={c.blocking_score:.2f} risk={c.risk_score:.2f}")
        else:
            print(f"[ShotScoring] Nenhum alvo viável com score >= {min_score:.2f}. Mantendo posse/procurando alternativa.")

    return decision

# --- Funções de Cálculo dos Componentes (0.0 a 1.0) ---

def _calc_opening(target_y: float, data: dict) -> float:
    """Calcula a abertura da trave em torno do alvo (proximidade de defensores/goleiro)."""
    opponents = data['opponents']
    goal_x = data['goal_x']
    min_dist_to_target = float('inf')
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in opponents:
        # Considera adversários na região defensiva final (últimos 1.5m de campo)
        if (goal_dir * ox) > (abs(goal_x) - 1.5): 
            dist = math.hypot(goal_x - ox, target_y - oy)
            if dist < min_dist_to_target:
                min_dist_to_target = dist
                
    if min_dist_to_target == float('inf'):
        return 1.0
    return min(1.0, min_dist_to_target / 0.60)

def _calc_angle(target_y: float, data: dict) -> float:
    """Calcula a facilidade de alinhamento entre a orientação do robô e a mira."""
    rx, ry, ryaw = data['rx'], data['ry'], data['ryaw']
    goal_x = data['goal_x']
    
    target_angle = math.atan2(target_y - ry, goal_x - rx)
    angle_diff = abs(math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw)))
    
    # 0 rad (perfeitamente alinhado) -> 1.0; pi rad (completamente de costas) -> 0.0
    score = 1.0 - (angle_diff / math.pi)
    return max(0.0, min(1.0, score))

def _calc_distance(target_y: float, data: dict) -> float:
    """Calcula a proximidade do robô ao alvo de finalização."""
    rx, ry = data['rx'], data['ry']
    goal_x = data['goal_x']
    max_dist = TACTICAL_CONFIG["SHOT"].get("max_distance", 6.0)
    
    dist = math.hypot(goal_x - rx, target_y - ry)
    score = 1.0 - (dist / max_dist)
    return max(0.0, min(1.0, score))

def _calc_progression(target_y: float, data: dict) -> float:
    """
    Calcula o avanço longitudinal em direção ao gol inimigo.
    Quanto mais próximo de goal_x a bola estiver, maior o score (0.0 no meio-campo, 1.0 na linha do gol).
    """
    bx = data['bx']
    goal_x = data['goal_x']
    
    if abs(goal_x) < 0.001:
        return 0.0
        
    progression = bx / goal_x
    return max(0.0, min(1.0, progression))

def _calc_blocking(target_y: float, data: dict) -> float:
    """
    Calcula a distância mínima de obstáculos à linha de chute bola -> alvo.
    Retorna 0.0 se houver obstáculo dentro do raio de colisão física.
    """
    bx, by = data['bx'], data['by']
    goal_x = data['goal_x']
    obstacles = data.get('obstacles', [])
    
    min_dist_to_line = float('inf')
    dx = goal_x - bx
    dy = target_y - by
    line_length_sq = dx*dx + dy*dy
    if line_length_sq == 0:
        return 0.0
    
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in obstacles:
        # Ignora obstáculos posicionados atrás da bola
        if (goal_dir * ox) < (goal_dir * bx) - 0.15: 
            continue 
        
        # Projeção escalar sobre o segmento [bola, alvo]
        t = ((ox - bx) * dx + (oy - by) * dy) / line_length_sq
        t = max(0.0, min(1.0, t))
        
        px = bx + t * dx
        py = by + t * dy
        
        dist_to_line = math.hypot(ox - px, oy - py)
        if dist_to_line < min_dist_to_line:
            min_dist_to_line = dist_to_line

    collision_radius = TACTICAL_CONFIG["SHOT"].get("collision_radius", 0.18)
    safe_margin = TACTICAL_CONFIG["SHOT"].get("safe_margin", 0.45)

    if min_dist_to_line <= collision_radius:
        return 0.0
    
    score = (min_dist_to_line - collision_radius) / (safe_margin - collision_radius)
    return max(0.0, min(1.0, score))

def _calc_risk(target_y: float, data: dict) -> float:
    """Calcula o risco baseado na pressão de adversários sobre o chutador/bola."""
    bx, by = data['bx'], data['by']
    opponents = data['opponents']
    
    if not opponents:
        return 1.0
    
    closest_opp_dist = min(math.hypot(ox - bx, oy - by) for ox, oy in opponents)
    # A menos de 20cm é pressão extrema (score=0), a 1 metro é confortável (score=1)
    score = (closest_opp_dist - 0.20) / 0.80
    return max(0.0, min(1.0, score))
from strategy.tactical_utils import ShotCandidate, ShotDecision, TACTICAL_CONFIG
import math

# --- Constantes geométricas aproximadas (Ajuste conforme seu campo) ---
MAX_SHOT_DIST = 6.0    # Distância máxima de chute para normalização
ROBOT_RADIUS = 0.09    # Raio do robô em metros
SAFE_MARGIN = 0.30     # Margem segura para um chute não ser interceptado

def generate_shot_candidates() -> list[ShotCandidate]:
    """Gera a lista de alvos discretos com base na configuração."""
    targets = TACTICAL_CONFIG["SHOT"]["candidate_targets_y"]
    return [ShotCandidate(target_y=y) for y in targets]

def evaluate_shot(candidate: ShotCandidate, world_model, robot_id: int) -> float:
    """
    Calcula o score do candidato combinando os fatores geométricos.
    Cada função auxiliar deve retornar um valor normalizado entre 0.0 e 1.0.
    """
    weights = TACTICAL_CONFIG["SHOT"]["weights"]
    
    # 1. Calcular os fatores individuais (Plugar a matemática do projeto aqui)
    candidate.opening_score = _calc_opening(candidate.target_y, world_model)
    candidate.angle_score = _calc_angle(candidate.target_y, world_model, robot_id)
    candidate.distance_score = _calc_distance(candidate.target_y, world_model, robot_id)
    candidate.progression_score = _calc_progression(candidate.target_y, world_model)
    candidate.blocking_score = _calc_blocking(candidate.target_y, world_model, robot_id)
    candidate.risk_score = _calc_risk(candidate.target_y, world_model)

    # 2. Somar o score total
    candidate.score = (
        (candidate.opening_score * weights["opening"]) +
        (candidate.angle_score * weights["angle"]) +
        (candidate.distance_score * weights["distance"]) +
        (candidate.progression_score * weights["progression"]) +
        (candidate.blocking_score * weights["blocking"]) +
        (candidate.risk_score * weights["risk"])
    )
    
    # Adiciona um motivo legível para os logs se a pontuação for muito baixa
    if candidate.score < TACTICAL_CONFIG["SHOT"]["min_score"]:
        candidate.reason = "Score abaixo do limiar aceitável."
        
    return candidate.score

def get_best_shot_decision(world_model, robot_id: int) -> ShotDecision:
    """Gera candidatos, avalia todos e retorna a decisão estruturada."""
    candidates = generate_shot_candidates()
    
    for candidate in candidates:
        evaluate_shot(candidate, world_model, robot_id)
        
    # Escolhe o candidato com o maior score
    best_candidate = max(candidates, key=lambda c: c.score)
    
    decision = ShotDecision(all_candidates=candidates)
    
    # Se o melhor alvo superar o limiar, salva na decisão
    if best_candidate.score >= TACTICAL_CONFIG["SHOT"]["min_score"]:
        decision.best_candidate = best_candidate
        decision.best_target_y = best_candidate.target_y
        
    return decision

# --- Funções Auxiliares de Adaptação ao WorldModel ---

def _get_goal_x(world_model) -> float:
    return world_model.field.enemy_goal_x

def _get_robot_pos(world_model, robot_id: int):
    robot = world_model.get_robot(robot_id)
    if robot and robot.visible:
        return robot.x, robot.y, robot.yaw
    return 0.0, 0.0, 0.0 # Fallback de segurança

def _get_ball_pos(world_model):
    ball = world_model.get_ball()
    return ball.x, ball.y

def _get_opponents_pos(world_model):
    # Retorna apenas os adversários visíveis no ciclo atual
    opponents = world_model.get_opponent_team()
    return [(opp.x, opp.y) for opp in opponents.values() if opp.visible]

# --- Funções de Cálculo (Retornam de 0.0 a 1.0) ---

def _calc_opening(target_y: float, world_model) -> float:
    opponents = _get_opponents_pos(world_model)
    goal_x = _get_goal_x(world_model)
    min_dist_to_target = float('inf')
    
    # Define a direção do gol para saber onde é a linha de fundo
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in opponents:
        # Se o adversário estiver no último metro do campo (perto do gol)
        if (goal_dir * ox) > (abs(goal_x) - 1.0): 
            dist = abs(oy - target_y)
            if dist < min_dist_to_target:
                min_dist_to_target = dist
                
    return min(1.0, min_dist_to_target / 0.5)

def _calc_angle(target_y: float, world_model, robot_id: int) -> float:
    rx, ry, ryaw = _get_robot_pos(world_model, robot_id)
    goal_x = _get_goal_x(world_model)
    
    target_angle = math.atan2(target_y - ry, goal_x - rx)
    angle_diff = abs(math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw)))
    
    score = 1.0 - (angle_diff / (math.pi / 2))
    return max(0.0, min(1.0, score))

def _calc_distance(target_y: float, world_model, robot_id: int) -> float:
    rx, ry, _ = _get_robot_pos(world_model, robot_id)
    goal_x = _get_goal_x(world_model)
    
    dist = math.hypot(goal_x - rx, target_y - ry)
    score = 1.0 - (dist / MAX_SHOT_DIST)
    return max(0.0, min(1.0, score))

def _calc_progression(target_y: float, world_model) -> float:
    bx, by = _get_ball_pos(world_model)
    goal_x = _get_goal_x(world_model)
    
    # Progressão é o avanço longitudinal. Depende se atacamos pra +X ou -X
    progression_x = (goal_x - bx) if goal_x > 0 else (bx - goal_x)
    
    if progression_x <= 0: return 0.0
    return min(1.0, progression_x / (abs(goal_x) / 2.0))

def _calc_blocking(target_y: float, world_model, robot_id: int) -> float:
    bx, by = _get_ball_pos(world_model)
    goal_x = _get_goal_x(world_model)
    opponents = _get_opponents_pos(world_model)
    
    min_dist_to_line = float('inf')
    dx = goal_x - bx
    dy = target_y - by
    line_length_sq = dx*dx + dy*dy
    if line_length_sq == 0: return 0.0
    
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in opponents:
        # Ignora adversários que estão atrás da bola
        if (goal_dir * ox) < (goal_dir * bx) - 0.2: 
            continue 
        
        t = ((ox - bx) * dx + (oy - by) * dy) / line_length_sq
        t = max(0, min(1, t))
        
        px = bx + t * dx
        py = by + t * dy
        
        dist_to_line = math.hypot(ox - px, oy - py)
        if dist_to_line < min_dist_to_line:
            min_dist_to_line = dist_to_line

    if min_dist_to_line <= ROBOT_RADIUS:
        return 0.0
    
    score = (min_dist_to_line - ROBOT_RADIUS) / (SAFE_MARGIN - ROBOT_RADIUS)
    return max(0.0, min(1.0, score))

def _calc_risk(target_y: float, world_model) -> float:
    bx, by = _get_ball_pos(world_model)
    opponents = _get_opponents_pos(world_model)
    
    if not opponents: return 1.0
    
    closest_opp_dist = min(math.hypot(ox - bx, oy - by) for ox, oy in opponents)
    score = (closest_opp_dist - 0.2) / 0.8
    return max(0.0, min(1.0, score))
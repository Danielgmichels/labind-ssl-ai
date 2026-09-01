from strategy.tactical_utils import ShotCandidate, ShotDecision, TACTICAL_CONFIG
import math

# --- Constantes geométricas aproximadas (Ajuste conforme seu campo) ---
MAX_SHOT_DIST = 6.0    
ROBOT_RADIUS = 0.09    
SAFE_MARGIN = 0.50     

def generate_shot_candidates() -> list[ShotCandidate]:
    targets = TACTICAL_CONFIG["SHOT"]["candidate_targets_y"]
    return [ShotCandidate(target_y=y) for y in targets]

def evaluate_shot(candidate: ShotCandidate, shared_data: dict) -> float:
    weights = TACTICAL_CONFIG["SHOT"]["weights"]
    
    # Passamos o dicionário com os dados pré-calculados
    candidate.opening_score = _calc_opening(candidate.target_y, shared_data)
    candidate.angle_score = _calc_angle(candidate.target_y, shared_data)
    candidate.distance_score = _calc_distance(candidate.target_y, shared_data)
    candidate.progression_score = _calc_progression(candidate.target_y, shared_data)
    candidate.blocking_score = _calc_blocking(candidate.target_y, shared_data)
    candidate.risk_score = _calc_risk(candidate.target_y, shared_data)

    candidate.score = (
        (candidate.opening_score * weights["opening"]) +
        (candidate.angle_score * weights["angle"]) +
        (candidate.distance_score * weights["distance"]) +
        (candidate.progression_score * weights["progression"]) +
        (candidate.blocking_score * weights["blocking"]) +
        (candidate.risk_score * weights["risk"])
    )
    
    if candidate.score < TACTICAL_CONFIG["SHOT"]["min_score"]:
        candidate.reason = "Score abaixo do limiar aceitável."
        
    return candidate.score

def get_best_shot_decision(world_model, robot_id: int) -> ShotDecision:
    # EXTRAÇÃO ÚNICA POR CICLO (Evita gargalo de FPS)
    goal_x = world_model.field.enemy_goal_x
    ball = world_model.get_ball()
    bx, by = ball.x, ball.y
    
    robot = world_model.get_robot(robot_id)
    rx, ry, ryaw = (robot.x, robot.y, robot.yaw) if (robot and robot.visible) else (0.0, 0.0, 0.0)
    
    opponents = [(opp.x, opp.y) for opp in world_model.get_opponent_team().values() if opp.visible]
    
    shared_data = {
        'goal_x': goal_x, 'bx': bx, 'by': by,
        'rx': rx, 'ry': ry, 'ryaw': ryaw,
        'opponents': opponents
    }

    candidates = generate_shot_candidates()
    
    for candidate in candidates:
        evaluate_shot(candidate, shared_data)
        
    best_candidate = max(candidates, key=lambda c: c.score)
    decision = ShotDecision(all_candidates=candidates)
    
    if best_candidate.score >= TACTICAL_CONFIG["SHOT"]["min_score"]:
        decision.best_candidate = best_candidate
        decision.best_target_y = best_candidate.target_y

    if decision.best_candidate:
        c = decision.best_candidate
        print(f"[ShotScoring] Alvo Escolhido: target_y={c.target_y:.2f} | score={c.score:.2f}")
        print(f"  Componentes: open={c.opening_score:.2f} ang={c.angle_score:.2f} dist={c.distance_score:.2f} prog={c.progression_score:.2f} block={c.blocking_score:.2f} risk={c.risk_score:.2f}")
    else:
        print(f"[ShotScoring] Nenhum alvo viável. Fallback para ConditionIsPathClear...")

    return decision

# --- Funções de Cálculo (Retornam de 0.0 a 1.0) ---

def _calc_opening(target_y: float, data: dict) -> float:
    opponents = data['opponents']
    goal_x = data['goal_x']
    min_dist_to_target = float('inf')
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in opponents:
        if (goal_dir * ox) > (abs(goal_x) - 1.0): 
            dist = abs(oy - target_y)
            if dist < min_dist_to_target:
                min_dist_to_target = dist
                
    return min(1.0, min_dist_to_target / 0.5)

def _calc_angle(target_y: float, data: dict) -> float:
    rx, ry, ryaw = data['rx'], data['ry'], data['ryaw']
    goal_x = data['goal_x']
    
    target_angle = math.atan2(target_y - ry, goal_x - rx)
    angle_diff = abs(math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw)))
    
    score = 1.0 - (angle_diff / (math.pi / 2))
    return max(0.0, min(1.0, score))

def _calc_distance(target_y: float, data: dict) -> float:
    rx, ry = data['rx'], data['ry']
    goal_x = data['goal_x']
    
    dist = math.hypot(goal_x - rx, target_y - ry)
    score = 1.0 - (dist / MAX_SHOT_DIST)
    return max(0.0, min(1.0, score))

def _calc_progression(target_y: float, data: dict) -> float:
    bx = data['bx']
    goal_x = data['goal_x']
    
    progression_x = (goal_x - bx) if goal_x > 0 else (bx - goal_x)
    if progression_x <= 0: return 0.0
    return min(1.0, progression_x / (abs(goal_x) / 2.0))

def _calc_blocking(target_y: float, data: dict) -> float:
    bx, by = data['bx'], data['by']
    goal_x = data['goal_x']
    opponents = data['opponents']
    
    min_dist_to_line = float('inf')
    dx = goal_x - bx
    dy = target_y - by
    line_length_sq = dx*dx + dy*dy
    if line_length_sq == 0: return 0.0
    
    goal_dir = 1.0 if goal_x > 0 else -1.0
    
    for ox, oy in opponents:
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

def _calc_risk(target_y: float, data: dict) -> float:
    bx, by = data['bx'], data['by']
    opponents = data['opponents']
    
    if not opponents: return 1.0
    
    closest_opp_dist = min(math.hypot(ox - bx, oy - by) for ox, oy in opponents)
    score = (closest_opp_dist - 0.2) / 0.8
    return max(0.0, min(1.0, score))
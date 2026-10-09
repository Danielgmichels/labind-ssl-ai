import time
import math

class BallState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.visible = False
        self.last_update = time.time()
        
        # Parâmetro do filtro EMA para velocidade
        self.alpha = 0.3 

    def update(self, new_x, new_y, current_time):
        dt = current_time - self.last_update
        
        if dt > 0 and self.visible:
            # Velocidade instantânea
            raw_vx = (new_x - self.x) / dt
            raw_vy = (new_y - self.y) / dt
            
            # Filtro EMA (mantendo o comportamento atual do main.py)
            self.vx = (self.alpha * raw_vx) + ((1.0 - self.alpha) * self.vx)
            self.vy = (self.alpha * raw_vy) + ((1.0 - self.alpha) * self.vy)
        else:
            # Se a bola acabou de reaparecer, resetamos a velocidade
            self.vx = 0.0
            self.vy = 0.0

        self.x = new_x
        self.y = new_y
        self.visible = True
        self.last_update = current_time

    def mark_invisible(self):
        self.visible = False
        # Mantemos self.x e self.y como a última posição conhecida


class RobotState:
    def __init__(self, robot_id):
        self.id = robot_id
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.visible = False

    def update(self, x, y, yaw):
        # Lógica de detecção de "fantasmas" (robôs na origem)
        if abs(x) < 0.001 and abs(y) < 0.001:
            self.visible = False
        else:
            self.x = x
            self.y = y
            self.yaw = yaw
            self.visible = True

    def mark_invisible(self):
        self.visible = False


class RefereeState:
    def __init__(self):
        self.command = None
        self.stage = None
        self.designated_position = None # Pode ser uma tupla (x, y) ou None


class FieldState:
    def __init__(self, is_yellow):
        # Mantendo as dimensões atuais hardcoded conforme especificado
        if is_yellow:
            self.our_goal_x = 6.0
            self.enemy_goal_x = -6.0
        else:
            self.our_goal_x = -6.0
            self.enemy_goal_x = 6.0


class WorldModel:
    def __init__(self, is_yellow=True):
        self.is_yellow = is_yellow
        
        self.ball = BallState()
        self.referee = RefereeState()
        self.field = FieldState(is_yellow)
        
        # Dicionários para acesso rápido por ID
        self.yellow = {i: RobotState(i) for i in range(16)}
        self.blue = {i: RobotState(i) for i in range(16)}
        
        self.last_update = time.time()

    def set_team(self, is_yellow):
        self.is_yellow = is_yellow
        self.field = FieldState(is_yellow)

    def _extract_yaw(self, v_robot):
        """Busca o ângulo do robô independente da nomenclatura do Protobuf."""
        # 1. Busca primeiro pelo nome exato usado nas suas actions!
        if hasattr(v_robot, 'yaw'): return v_robot.yaw
        
        # 2. Fallbacks de segurança
        if hasattr(v_robot, 'orientation'): return v_robot.orientation
        if hasattr(v_robot, 'theta'): return v_robot.theta
        
        # 3. Busca dentro do sub-objeto pos, se existir
        if hasattr(v_robot, 'pos'):
            if hasattr(v_robot.pos, 'yaw'): return v_robot.pos.yaw
            if hasattr(v_robot.pos, 'orientation'): return v_robot.pos.orientation
            if hasattr(v_robot.pos, 'theta'): return v_robot.pos.theta
            
        return 0.0

    def update_vision(self, vision_state):
        current_time = time.time()
        self.last_update = current_time

        # 1. Atualizar Bola
        if vision_state.ball.visible:
            self.ball.update(vision_state.ball.pos.x, vision_state.ball.pos.y, current_time)
        else:
            self.ball.mark_invisible()

        # 2. Atualizar Robôs Amarelos
        for robot in self.yellow.values():
            robot.mark_invisible()
            
        for v_robot in vision_state.yellow:
            if v_robot.id in self.yellow:
                yaw = self._extract_yaw(v_robot)
                self.yellow[v_robot.id].update(v_robot.pos.x, v_robot.pos.y, yaw)

        # 3. Atualizar Robôs Azuis
        for robot in self.blue.values():
            robot.mark_invisible()
            
        for v_robot in vision_state.blue:
            if v_robot.id in self.blue:
                yaw = self._extract_yaw(v_robot)
                self.blue[v_robot.id].update(v_robot.pos.x, v_robot.pos.y, yaw)

    def update_referee(self, command, stage, designated_position=None):
        self.referee.command = command
        self.referee.stage = stage
        self.referee.designated_position = designated_position

    # --- Consultas (Getters) ---

    def get_ball(self):
        return self.ball

    def get_my_team(self):
        return self.yellow if self.is_yellow else self.blue

    def get_opponent_team(self):
        return self.blue if self.is_yellow else self.yellow

    def get_robot(self, robot_id, is_opponent=False):
        team = self.get_opponent_team() if is_opponent else self.get_my_team()
        return team.get(robot_id)

    def get_nearest_robot_to_ball(self, check_opponents=False):
        if not self.ball.visible:
            return None

        team = self.get_opponent_team() if check_opponents else self.get_my_team()
        
        closest_robot = None
        min_dist = float('inf')

        for robot in team.values():
            if robot.visible:
                dist = math.hypot(robot.x - self.ball.x, robot.y - self.ball.y)
                if dist < min_dist:
                    min_dist = dist
                    closest_robot = robot
                    
        return closest_robot
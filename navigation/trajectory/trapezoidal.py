import math
from typing import List, Tuple, Optional
from navigation.interfaces import ITrajectoryGenerator


class TrapezoidalTrajectoryGenerator(ITrajectoryGenerator):
    """
    Gerador de Trajetória Trapezoidal Temporal Sincronizado para a RoboCup SSL.
    
    Principais características físicas:
    1. Sincronização Dinâmica: Inicia a partir da velocidade estimada atual do robô (v0).
    2. Restrição Centrípeta em Curvas: Calcula o raio de curvatura R nos waypoints intermediários
       e limita a velocidade escalar para v <= sqrt(a_lat * R), evitando derrapagem.
    3. Perfil de 3 Fases: Aceleração (+a_max), Cruzeiro (v_max) e Frenagem (-a_max).
    4. Conversão para o referencial local do robô: (v_forward, v_left, vw).
    """

    def __init__(
        self,
        max_vel: float = 3.0,
        max_acc: float = 5.0,
        max_lat_acc: float = 3.5,
        kp_angular: float = 2.0,
        max_angular_vel: float = 5.0,
        lookahead_dist: float = 0.18,
        dt: float = 1.0 / 60.0
    ):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_lat_acc = max_lat_acc
        self.kp_angular = kp_angular
        self.max_angular_vel = max_angular_vel
        self.lookahead_dist = lookahead_dist
        self.dt = dt
        self.current_speed = 0.0

    def compute_step(
        self,
        path: List[Tuple[float, float]],
        robot_pose: Tuple[float, float, float],
        target_yaw: Optional[float] = None,
        v0: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[float, float, float]:
        """
        Executa um passo do controlador temporal trapezoidal a 60 Hz.

        :param path: Lista ordenada de waypoints [(x, y), ...] do início até a meta.
        :param robot_pose: Pose atual do robô (x, y, yaw).
        :param target_yaw: Orientação angular desejada (None para apontar para o movimento).
        :param v0: Velocidade linear estimada atual do robô (vx, vy) no plano global.
        :return: (v_forward, v_left, vw) no referencial local do robô.
        """
        rx, ry, ryaw = robot_pose

        if not path:
            self.current_speed = 0.0
            return 0.0, 0.0, 0.0

        # 1. Limpeza de waypoints já ultrapassados (Lookahead dinâmico)
        active_path = self._advance_waypoints(path, (rx, ry))
        if not active_path:
            self.current_speed = 0.0
            return 0.0, 0.0, 0.0

        target_wp = active_path[0]
        dx = target_wp[0] - rx
        dy = target_wp[1] - ry
        dist_to_next = math.hypot(dx, dy)

        # Distância total restante ao longo de todo o caminho restante
        total_remaining_dist = dist_to_next
        for i in range(len(active_path) - 1):
            total_remaining_dist += math.hypot(
                active_path[i + 1][0] - active_path[i][0],
                active_path[i + 1][1] - active_path[i][1]
            )

        # Se já estiver exatamente no alvo final (tolerância de 3 cm), encerra o movimento
        if total_remaining_dist < 0.03:
            self.current_speed = 0.0
            vw = self._compute_angular_velocity(ryaw, target_yaw, 0.0, 0.0)
            return 0.0, 0.0, vw

        # 2. Direção unitária para o waypoint ativo
        dir_x = dx / dist_to_next
        dir_y = dy / dist_to_next

        # 3. Sincronização Dinâmica com v0
        # Projeta o vetor de velocidade real atual na direção do caminho
        v0_x, v0_y = v0
        current_speed_proj = max(0.0, v0_x * dir_x + v0_y * dir_y)

        # 4. Cálculo de Restrição de Curvatura no Próximo Canto
        corner_speed_limit = self.max_vel
        if len(active_path) >= 2:
            # Analisa o canto entre a posição atual do robô e os 2 primeiros waypoints
            p_prev = (rx, ry)
            p_curr = active_path[0]
            p_next = active_path[1]
            radius = self._compute_corner_radius(p_prev, p_curr, p_next)
            if radius < 5.0: # Se for uma curva mensurável (não quase reta)
                v_curve = math.sqrt(self.max_lat_acc * radius)
                # Velocidade máxima que podemos ter agora para conseguir chegar na curva em v_curve
                dist_to_corner = dist_to_next
                v_allowed_for_corner = math.sqrt(v_curve**2 + 2.0 * self.max_acc * dist_to_corner)
                corner_speed_limit = min(self.max_vel, v_allowed_for_corner)

        # 5. Cálculo do Perfil Trapezoidal (Aceleração, Cruzeiro e Frenagem)
        # Distância de frenagem necessária para parar em total_remaining_dist com velocidade zero:
        # v_max_brake = sqrt(2 * a_max * d)
        v_brake_limit = math.sqrt(max(0.0, 2.0 * self.max_acc * total_remaining_dist))

        # Teto de velocidade admissível para este ciclo
        speed_ceiling = min(self.max_vel, corner_speed_limit, v_brake_limit)

        # Rampa de aceleração contínua mantendo o estado da velocidade do ciclo anterior
        max_delta_v = self.max_acc * self.dt
        # Sincronização dinâmica: utiliza a velocidade interna como base,
        # ou v0_proj se o robô já estiver em movimento mais rápido (ex: retomada externa)
        baseline_speed = max(self.current_speed, current_speed_proj)
        commanded_speed = min(speed_ceiling, baseline_speed + max_delta_v)

        # Se o teto de frenagem exigir desaceleração, força a redução imediata
        if commanded_speed > speed_ceiling:
            commanded_speed = speed_ceiling

        self.current_speed = commanded_speed

        # 6. Geração do Vetor de Velocidade Global
        vx_global = commanded_speed * dir_x
        vy_global = commanded_speed * dir_y

        # 7. Conversão para o Referencial Local do Robô
        v_forward = vx_global * math.cos(ryaw) + vy_global * math.sin(ryaw)
        v_left = -vx_global * math.sin(ryaw) + vy_global * math.cos(ryaw)

        # 8. Controle Angular Desacoplado
        vw = self._compute_angular_velocity(ryaw, target_yaw, vx_global, vy_global)

        return v_forward, v_left, vw

    def _advance_waypoints(
        self,
        path: List[Tuple[float, float]],
        robot_xy: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Avança ao longo da lista de waypoints descartando pontos intermediários
        já atingidos dentro da distância de lookahead.
        """
        rx, ry = robot_xy
        remaining = list(path)

        while len(remaining) > 1:
            wp = remaining[0]
            dist = math.hypot(wp[0] - rx, wp[1] - ry)
            if dist < self.lookahead_dist:
                remaining.pop(0)
            else:
                break

        return remaining

    def _compute_corner_radius(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float]
    ) -> float:
        """
        Calcula o raio do círculo circunscrito aos 3 pontos consecutivos (p1, p2, p3).
        R = (a * b * c) / (4 * Area)
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        a = math.hypot(x2 - x1, y2 - y1)
        b = math.hypot(x3 - x2, y3 - y2)
        c = math.hypot(x3 - x1, y3 - y1)

        # Área pelo produto vetorial 2D (determinante)
        double_area = abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))

        if double_area < 1e-4 or a < 1e-3 or b < 1e-3:
            # Pontos quase colineares (linha reta) -> Raio infinito
            return float('inf')

        radius = (a * b * c) / (2.0 * double_area)
        return radius

    def _compute_angular_velocity(
        self,
        current_yaw: float,
        target_yaw: Optional[float],
        vx: float,
        vy: float
    ) -> float:
        """Calcula a velocidade angular vw com convergência suave e saturação."""
        if target_yaw is not None:
            desired_angle = target_yaw
        else:
            speed = math.hypot(vx, vy)
            if speed > 0.10:
                desired_angle = math.atan2(vy, vx)
            else:
                desired_angle = current_yaw

        erro = desired_angle - current_yaw
        erro = (erro + math.pi) % (2.0 * math.pi) - math.pi

        if abs(erro) < 0.03:
            return 0.0

        vw = erro * self.kp_angular
        return max(-self.max_angular_vel, min(self.max_angular_vel, vw))

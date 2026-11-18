import math
import time
from typing import List, Tuple, Dict, Any, Optional

from navigation.APF import ProportionalController
from navigation.factory import get_path_planner


class _BaselineTrajectoryStepper:
    """
    Controlador temporal simples utilizado como fallback na Etapa 0 antes da
    implementação completa do gerador trapezoidal da Etapa 1.
    """
    def __init__(self, max_vel: float = 2.5, kp_linear: float = 2.0, kp_angular: float = 2.0, max_angular_vel: float = 5.0):
        self.max_vel = max_vel
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_angular_vel = max_angular_vel

    def compute_step(
        self,
        path: List[Tuple[float, float]],
        robot_pose: Tuple[float, float, float],
        target_yaw: Optional[float] = None,
        v0: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[float, float, float]:
        rx, ry, ryaw = robot_pose

        if not path:
            return 0.0, 0.0, 0.0

        # Se houver mais de um waypoint, aponta para o primeiro ou próximo
        next_wp = path[1] if len(path) > 1 and math.hypot(path[0][0] - rx, path[0][1] - ry) < 0.15 else path[0]
        dx = next_wp[0] - rx
        dy = next_wp[1] - ry
        dist = math.hypot(dx, dy)

        vx_global = dx * self.kp_linear
        vy_global = dy * self.kp_linear

        mag = math.hypot(vx_global, vy_global)
        if mag > self.max_vel:
            vx_global = (vx_global / mag) * self.max_vel
            vy_global = (vy_global / mag) * self.max_vel

        # Rotação para o alvo ou orientação desejada
        if target_yaw is not None:
            target_angle = target_yaw
        else:
            target_angle = math.atan2(dy, dx) if dist > 0.05 else ryaw

        erro_angular = target_angle - ryaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi

        if abs(erro_angular) < 0.05:
            vw = 0.0
        else:
            vw = max(-self.max_angular_vel, min(self.max_angular_vel, erro_angular * self.kp_angular))

        # Matriz de Rotação para referencial local do robô
        v_forward = vx_global * math.cos(ryaw) + vy_global * math.sin(ryaw)
        v_left = -vx_global * math.sin(ryaw) + vy_global * math.cos(ryaw)

        return v_forward, v_left, vw


class NavigationManager:
    """
    Gerenciador Universal de Navegação da RoboCup SSL.
    Mantém retrocompatibilidade estrita com a Behavior Tree (calculate_velocity),
    isola o estado cinemático entre robôs do time e aplica proteções de campo e desatolamento.
    """

    # Limites rígidos das quatro linhas do campo oficial SSL (Divisão A / Expandido)
    FIELD_X_MIN: float = -5.80
    FIELD_X_MAX: float = 5.80
    FIELD_Y_MIN: float = -4.30
    FIELD_Y_MAX: float = 4.30

    def __init__(
        self,
        planner_type: str = "LEGACY_APF",
        max_vel: float = 2.0,
        max_acc: float = 3.0,
        kp_linear: float = 2.0,
        kp_angular: float = 2.0,
        max_angular_vel: float = 5.0,
        predict_obstacles_dt: float = 0.0
    ):
        self.planner_type = planner_type.strip().upper()
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_angular_vel = max_angular_vel
        self.predict_obstacles_dt = predict_obstacles_dt

        self.legacy_apf = ProportionalController(
            kp_linear=kp_linear,
            kp_angular=kp_angular,
            max_vel=max_vel,
            max_angular_vel=max_angular_vel
        )

        if self.planner_type != "LEGACY_APF":
            self.planner = get_path_planner(self.planner_type)
        else:
            self.planner = None

        # Instâncias de contexto cinemático isoladas por robô (id -> context)
        self.robot_states: Dict[Any, Dict[str, Any]] = {}

    def calculate_velocity(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        target_x: float,
        target_y: float,
        obstacles: list,
        target_yaw: Optional[float] = None,
        robot_id: Optional[int] = None
    ) -> Tuple[float, float, float]:
        """
        Calcula as velocidades locais (v_forward, v_left, vw) para guiar o robô até o alvo.
        Totalmente compatível com a assinatura consumida por behavior_tree/actions.py.
        """
        # 1. Proteção de Limites de Campo: satura o alvo para dentro das linhas navegáveis
        clamped_target_x = max(self.FIELD_X_MIN, min(target_x, self.FIELD_X_MAX))
        clamped_target_y = max(self.FIELD_Y_MIN, min(target_y, self.FIELD_Y_MAX))

        # 2. Modo Legado (ProportionalController / APF original mantido intacto)
        if self.planner_type == "LEGACY_APF":
            return self.legacy_apf.calculate_velocity(
                robot_x, robot_y, robot_yaw,
                clamped_target_x, clamped_target_y,
                obstacles,
                target_yaw=target_yaw
            )

        # 3. Recupera o contexto multi-robô isolado e a velocidade estimada v0
        r_key, ctx, v0 = self._get_robot_context(robot_x, robot_y, robot_id)

        # 4. Extração e separação de obstáculos circulares e zonas proibidas
        parsed_obs, forbidden_zones = self._parse_obstacles_and_zones(obstacles)

        # 5. Tratamento de Unstuck para partida e proteção da grande área
        safe_start_x, safe_start_y = self._project_if_inside_obstacle(robot_x, robot_y, parsed_obs)
        safe_goal_x, safe_goal_y = self._project_if_inside_forbidden_zone(
            clamped_target_x, clamped_target_y, forbidden_zones
        )

        # 6. Planejamento Geométrico
        start_pos = (safe_start_x, safe_start_y)
        goal_pos = (safe_goal_x, safe_goal_y)
        path = self.planner.plan(start_pos, goal_pos, parsed_obs, forbidden_zones)

        # 7. Geração de Trajetória / Controle Temporal
        traj_gen = ctx['trajectory_gen']
        vf, vl, vw = traj_gen.compute_step(
            path, (robot_x, robot_y, robot_yaw), target_yaw=target_yaw, v0=v0
        )
        return vf, vl, vw

    def _get_robot_context(
        self,
        robot_x: float,
        robot_y: float,
        robot_id: Optional[int]
    ) -> Tuple[Any, Dict[str, Any], Tuple[float, float]]:
        """Recupera ou inicializa o contexto dinâmico isolado por robô."""
        now = time.time()
        r_key = robot_id if robot_id is not None else self._resolve_robot_key(robot_x, robot_y)

        if r_key not in self.robot_states:
            traj_gen = self._create_trajectory_generator()
            self.robot_states[r_key] = {
                'last_pos': (robot_x, robot_y),
                'last_time': now,
                'v0': (0.0, 0.0),
                'trajectory_gen': traj_gen
            }
            return r_key, self.robot_states[r_key], (0.0, 0.0)

        ctx = self.robot_states[r_key]
        dt = now - ctx['last_time']

        if 0.001 < dt < 0.2:
            raw_vx = (robot_x - ctx['last_pos'][0]) / dt
            raw_vy = (robot_y - ctx['last_pos'][1]) / dt
            # Filtro exponencial (EMA) para suavizar ruído de visão
            v0_x = 0.7 * ctx['v0'][0] + 0.3 * raw_vx
            v0_y = 0.7 * ctx['v0'][1] + 0.3 * raw_vy
        else:
            v0_x, v0_y = ctx['v0']

        ctx['last_pos'] = (robot_x, robot_y)
        ctx['last_time'] = now
        ctx['v0'] = (v0_x, v0_y)
        return r_key, ctx, (v0_x, v0_y)

    def _resolve_robot_key(self, rx: float, ry: float) -> str:
        """Associa a pose ao robô mais próximo registrado no ciclo anterior."""
        best_key = None
        best_dist = 0.40  # Tolerância máxima de continuidade espacial (m)

        for k, ctx in self.robot_states.items():
            lx, ly = ctx['last_pos']
            dist = math.hypot(rx - lx, ry - ly)
            if dist < best_dist:
                best_dist = dist
                best_key = k

        if best_key is not None:
            return best_key
        return f"bot_auto_{len(self.robot_states)}"

    def _create_trajectory_generator(self):
        """Cria o gerador de trajetória. Utiliza TrapezoidalTrajectoryGenerator se disponível, ou fallback."""
        try:
            from navigation.trajectory.trapezoidal import TrapezoidalTrajectoryGenerator
            return TrapezoidalTrajectoryGenerator(max_vel=self.max_vel, max_acc=self.max_acc)
        except (ImportError, ModuleNotFoundError):
            return _BaselineTrajectoryStepper(
                max_vel=self.max_vel,
                kp_linear=self.kp_linear,
                kp_angular=self.kp_angular,
                max_angular_vel=self.max_angular_vel
            )

    def _parse_obstacles_and_zones(self, obstacles: list) -> Tuple[List[Dict[str, Any]], List[Dict[str, float]]]:
        """
        Separa os robôs físicos das barreiras pontuais de área de defesa (VirtualObstacle).
        Converte as ~86 barreiras pontuais em 2 caixas retangulares (forbidden_zones).
        """
        parsed_obs: List[Dict[str, Any]] = []
        has_defense_walls = False

        for obs in obstacles:
            ox = obs.pos.x if hasattr(obs, 'pos') else getattr(obs, 'x', 0.0)
            oy = obs.pos.y if hasattr(obs, 'pos') else getattr(obs, 'y', 0.0)
            is_robot = hasattr(obs, 'id')

            if is_robot:
                vx = getattr(obs, 'vx', 0.0)
                vy = getattr(obs, 'vy', 0.0)
                # Predição opcional de obstáculos dinâmicos
                if self.predict_obstacles_dt > 0.0:
                    ox += vx * self.predict_obstacles_dt
                    oy += vy * self.predict_obstacles_dt

                parsed_obs.append({
                    'x': ox,
                    'y': oy,
                    'is_robot': True,
                    'radius': 0.09,
                    'vx': vx,
                    'vy': vy
                })
            else:
                has_defense_walls = True

        forbidden_zones: List[Dict[str, float]] = []
        if has_defense_walls:
            # Grande Área Esquerda (Nossa defesa): X de -6.0 a -4.2, Y de -1.8 a 1.8
            forbidden_zones.append({'xmin': -6.0, 'xmax': -4.2, 'ymin': -1.8, 'ymax': 1.8})
            # Grande Área Direita (Inimiga): X de 4.2 a 6.0, Y de -1.8 a 1.8
            forbidden_zones.append({'xmin': 4.2, 'xmax': 6.0, 'ymin': -1.8, 'ymax': 1.8})

        return parsed_obs, forbidden_zones

    def _project_if_inside_forbidden_zone(
        self,
        gx: float,
        gy: float,
        forbidden_zones: List[Dict[str, float]],
        margin: float = 0.20
    ) -> Tuple[float, float]:
        """Projeta a meta para fora da grande área caso um jogador de linha tenha sido ordenado para dentro dela."""
        for zone in forbidden_zones:
            xmin, xmax = zone['xmin'], zone['xmax']
            ymin, ymax = zone['ymin'], zone['ymax']
            if (xmin <= gx <= xmax) and (ymin <= gy <= ymax):
                candidates = []
                # Borda frontal em direção ao centro de campo
                if xmax >= 4.0:
                    candidates.append((xmin - margin, gy))
                if xmin <= -4.0:
                    candidates.append((xmax + margin, gy))
                # Bordas laterais superior e inferior
                candidates.append((gx, ymax + margin))
                candidates.append((gx, ymin - margin))

                best_pt = min(candidates, key=lambda c: (c[0] - gx)**2 + (c[1] - gy)**2)
                gx = max(self.FIELD_X_MIN, min(best_pt[0], self.FIELD_X_MAX))
                gy = max(self.FIELD_Y_MIN, min(best_pt[1], self.FIELD_Y_MAX))
        return gx, gy

    def _project_if_inside_obstacle(
        self,
        px: float,
        py: float,
        obstacles: List[Dict[str, Any]],
        reference_point: Optional[Tuple[float, float]] = None
    ) -> Tuple[float, float]:
        """
        Desatola o ponto caso ele esteja dentro do raio inflado de algum obstáculo circular.
        """
        safe_margin = 0.22  # Raio inflado padrão (2 * r_bot + folga)
        for obs in obstacles:
            ox, oy = obs['x'], obs['y']
            dist = math.hypot(px - ox, py - oy)
            if dist < safe_margin:
                if dist > 0.001:
                    # Empurra radialmente para fora
                    nx = (px - ox) / dist
                    ny = (py - oy) / dist
                elif reference_point is not None:
                    # Direção a partir do ponto de referência
                    rdx = px - reference_point[0]
                    rdy = py - reference_point[1]
                    rdist = math.hypot(rdx, rdy)
                    if rdist > 0.001:
                        nx, ny = rdx / rdist, rdy / rdist
                    else:
                        nx, ny = 1.0, 0.0
                else:
                    nx, ny = 1.0, 0.0

                px = ox + nx * (safe_margin + 0.02)
                py = oy + ny * (safe_margin + 0.02)
        return px, py

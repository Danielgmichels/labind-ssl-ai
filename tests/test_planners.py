import os
import sys
import unittest
import math
import tempfile
import csv

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)
if os.path.join(PROJECT_ROOT, "proto_msg") not in sys.path:
    sys.path.append(os.path.join(PROJECT_ROOT, "proto_msg"))

from navigation.interfaces import IPathPlanner, ITrajectoryGenerator
from navigation.factory import get_path_planner, register_path_planner
from navigation.manager import NavigationManager
from navigation.APF import ProportionalController
from navigation.Obstacles import VirtualObstacle, create_solid_defense_walls
from navigation.trajectory.trapezoidal import TrapezoidalTrajectoryGenerator
from benchmarks.logger import BenchmarkLogger


class MockRobotPos:
    """Mock representando robô real com id e coordenada pos."""
    def __init__(self, x, y, yaw=0.0, r_id=1, vx=0.0, vy=0.0):
        self.id = r_id
        self.yaw = yaw
        self.pos = type('Pos', (), {'x': x, 'y': y})()
        self.vx = vx
        self.vy = vy


class DummyTestPlanner(IPathPlanner):
    def plan(self, start_pos, goal_pos, obstacles, forbidden_zones):
        # Retorna reta direta simples
        return [start_pos, goal_pos]


class TestInterfacesAndFactory(unittest.TestCase):
    def test_abstract_classes_enforce_implementation(self):
        with self.assertRaises(TypeError):
            IPathPlanner()

        with self.assertRaises(TypeError):
            ITrajectoryGenerator()

    def test_factory_registry_and_lookup(self):
        @register_path_planner("TEST_DUMMY")
        class RegisteredDummy(IPathPlanner):
            def plan(self, start_pos, goal_pos, obstacles, forbidden_zones):
                return [start_pos, goal_pos]

        planner = get_path_planner("TEST_DUMMY")
        self.assertIsInstance(planner, RegisteredDummy)

        with self.assertRaises(NotImplementedError):
            get_path_planner("NON_EXISTENT_PLANNER_XYZ")


class TestNavigationManager(unittest.TestCase):
    def setUp(self):
        self.legacy_mgr = NavigationManager(planner_type="LEGACY_APF", max_vel=2.5)
        self.raw_apf = ProportionalController(max_vel=2.5)

    def test_legacy_apf_mode_backward_compatibility(self):
        # Compara a saída do NavigationManager(LEGACY_APF) com o ProportionalController direto
        robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
        target_x, target_y = 2.0, 1.0
        obstacles = [MockRobotPos(1.0, 0.5, r_id=2)]

        vf_mgr, vl_mgr, vw_mgr = self.legacy_mgr.calculate_velocity(
            robot_x, robot_y, robot_yaw, target_x, target_y, obstacles
        )
        vf_raw, vl_raw, vw_raw = self.raw_apf.calculate_velocity(
            robot_x, robot_y, robot_yaw, target_x, target_y, obstacles
        )

        self.assertAlmostEqual(vf_mgr, vf_raw, places=3)
        self.assertAlmostEqual(vl_mgr, vl_raw, places=3)
        self.assertAlmostEqual(vw_mgr, vw_raw, places=3)

    def test_field_bounding_box_clamping(self):
        # Alvo muito fora do campo (ex: x=10.0, y=8.0) deve ser saturado para x=5.8, y=4.3
        target_x, target_y = 10.0, 8.0
        mgr = NavigationManager(planner_type="LEGACY_APF")

        # Chama calculate_velocity
        vf, vl, vw = mgr.calculate_velocity(0.0, 0.0, 0.0, target_x, target_y, [])
        # O robô deve se mover em direção ao ponto saturado (5.8, 4.3), e não (10.0, 8.0)
        expected_angle = math.atan2(4.3 - 0.0, 5.8 - 0.0)
        actual_angle = math.atan2(vl, vf) # Com yaw=0, v_forward=vx e v_left=vy
        self.assertAlmostEqual(actual_angle, expected_angle, places=2)

    def test_obstacle_and_zone_parsing(self):
        mgr = NavigationManager(planner_type="LEGACY_APF")

        # Mistura robôs reais com paredes virtuais das grandes áreas
        real_robot = MockRobotPos(1.5, 0.0, r_id=3, vx=1.0, vy=-0.5)
        defense_walls = create_solid_defense_walls()
        mixed_obstacles = [real_robot] + defense_walls

        parsed_obs, forbidden_zones = mgr._parse_obstacles_and_zones(mixed_obstacles)

        # Deve conter apenas 1 obstáculo circular (o robô)
        self.assertEqual(len(parsed_obs), 1)
        self.assertEqual(parsed_obs[0]['x'], 1.5)
        self.assertEqual(parsed_obs[0]['is_robot'], True)
        self.assertEqual(parsed_obs[0]['radius'], 0.09)

        # As ~86 paredes devem ter sido agrupadas em 2 caixas retangulares
        self.assertEqual(len(forbidden_zones), 2)
        # Nossos limites de defesa: -6.0 a -4.8 e 4.8 a 6.0
        self.assertIn({'xmin': -6.0, 'xmax': -4.8, 'ymin': -1.2, 'ymax': 1.2}, forbidden_zones)
        self.assertIn({'xmin': 4.8, 'xmax': 6.0, 'ymin': -1.2, 'ymax': 1.2}, forbidden_zones)

    def test_multi_robot_context_isolation(self):
        @register_path_planner("TEST_DUMMY_MULTI")
        class MultiDummy(IPathPlanner):
            def plan(self, start_pos, goal_pos, obstacles, forbidden_zones):
                return [start_pos, goal_pos]

        mgr = NavigationManager(planner_type="TEST_DUMMY_MULTI")

        # Executa para Robô 1
        mgr.calculate_velocity(0.0, 0.0, 0.0, 1.0, 0.0, [], robot_id=1)
        # Executa para Robô 2
        mgr.calculate_velocity(-2.0, 2.0, 0.0, -1.0, 2.0, [], robot_id=2)

        # Os estados devem existir de forma independente no dicionário
        self.assertIn(1, mgr.robot_states)
        self.assertIn(2, mgr.robot_states)
        self.assertEqual(mgr.robot_states[1]['last_pos'], (0.0, 0.0))
        self.assertEqual(mgr.robot_states[2]['last_pos'], (-2.0, 2.0))

    def test_navigation_manager_with_trapezoidal_generator(self):
        @register_path_planner("TEST_TRAP_INTEGRATION")
        class TrapPlanner(IPathPlanner):
            def plan(self, start_pos, goal_pos, obstacles, forbidden_zones):
                return [start_pos, (2.0, 0.0), goal_pos]

        mgr = NavigationManager(planner_type="TEST_TRAP_INTEGRATION", max_vel=2.5, max_acc=4.0)

        # Passo 1: Partida do repouso
        vf1, vl1, vw1 = mgr.calculate_velocity(0.0, 0.0, 0.0, 4.0, 0.0, [], robot_id=1)
        self.assertGreater(vf1, 0.0)
        self.assertLessEqual(vf1, 4.0 * (1.0 / 60.0) + 0.01)

        # Passo 2: Próximo ao alvo (3.95m de 4.0m)
        vf2, vl2, vw2 = mgr.calculate_velocity(3.95, 0.0, 0.0, 4.0, 0.0, [], robot_id=1)
        # Deve estar freando muito forte próximo de zero
        self.assertLess(vf2, 1.0)

    def test_unstuck_projection(self):
        mgr = NavigationManager(planner_type="LEGACY_APF")
        obstacles = [{'x': 1.0, 'y': 1.0, 'radius': 0.09}]

        # Ponto exatamente sobre o obstáculo (colisão severa)
        px, py = mgr._project_if_inside_obstacle(1.0, 1.0, obstacles)
        dist = math.hypot(px - 1.0, py - 1.0)

        # Deve ter sido projetado para fora da margem de segurança (>= 0.22m)
        self.assertGreaterEqual(dist, 0.22)


class TestBenchmarkLogger(unittest.TestCase):
    def test_logger_creates_valid_csv(self):
        with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            logger = BenchmarkLogger(tmp_path, algorithm_name="TEST_ALGO", run_id=42, density_level=3)
            logger.log_step(
                calc_time_ms=0.45,
                pos_x=1.2,
                pos_y=-0.8,
                vel_real_x=2.1,
                vel_real_y=0.0,
                vel_cmd_x=2.5,
                vel_cmd_y=0.0,
                min_obstacle_dist=0.35,
                collision_flag=False,
                timestamp=1000.0
            )
            logger.close()

            # Lê o arquivo gravado e confere integridade
            with open(tmp_path, "r", encoding="utf-8") as f:
                reader = list(csv.reader(f))

            self.assertEqual(len(reader), 2)  # Header + 1 linha de dados
            self.assertEqual(reader[0], BenchmarkLogger.HEADERS)
            self.assertEqual(reader[1][1], "42")
            self.assertEqual(reader[1][2], "TEST_ALGO")
            self.assertEqual(reader[1][3], "3")
            self.assertEqual(reader[1][12], "0")  # collision_flag == False -> 0
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)


class TestTrapezoidalTrajectoryGenerator(unittest.TestCase):
    def setUp(self):
        self.traj_gen = TrapezoidalTrajectoryGenerator(
            max_vel=2.5,
            max_acc=4.0,
            max_lat_acc=3.5,
            kp_angular=2.0,
            max_angular_vel=5.0,
            lookahead_dist=0.18,
            dt=1.0 / 60.0
        )

    def test_acceleration_from_rest(self):
        # Robô em (0, 0), meta distante em (4.0, 0.0), v0 = 0
        path = [(0.0, 0.0), (4.0, 0.0)]
        vf, vl, vw = self.traj_gen.compute_step(path, (0.0, 0.0, 0.0), v0=(0.0, 0.0))

        # A velocidade linear no primeiro ciclo deve ser a_max * dt = 4.0 * (1/60) ~ 0.0667 m/s
        expected_v = 4.0 * (1.0 / 60.0)
        self.assertAlmostEqual(vf, expected_v, places=3)
        self.assertAlmostEqual(vl, 0.0, places=3)

    def test_preserves_initial_velocity_v0(self):
        # Robô já em movimento a 2.0 m/s na direção X
        path = [(0.0, 0.0), (4.0, 0.0)]
        vf, vl, vw = self.traj_gen.compute_step(path, (0.0, 0.0, 0.0), v0=(2.0, 0.0))

        # Deve continuar a partir de 2.0 m/s + incremento de aceleração, sem cair para 0
        self.assertGreater(vf, 2.0)
        self.assertLessEqual(vf, 2.5)
        self.assertAlmostEqual(vl, 0.0, places=3)

    def test_braking_near_goal(self):
        # Robô muito próximo do alvo (10 cm restantes), mesmo com v0 alta ele é forçado a frear
        path = [(0.90, 0.0), (1.0, 0.0)]
        vf, vl, vw = self.traj_gen.compute_step(path, (0.90, 0.0, 0.0), v0=(2.5, 0.0))

        # Limite de frenagem: v_brake = sqrt(2 * a * d) = sqrt(2 * 4.0 * 0.10) = sqrt(0.8) ~ 0.894 m/s
        v_brake = math.sqrt(2.0 * 4.0 * 0.10)
        self.assertLessEqual(vf, v_brake + 0.1)

    def test_stop_when_reached_goal(self):
        # Robô a 2 cm do alvo (dentro da tolerância de 3 cm)
        path = [(1.0, 0.0)]
        vf, vl, vw = self.traj_gen.compute_step(path, (0.985, 0.0, 0.0), v0=(0.1, 0.0))

        self.assertEqual(vf, 0.0)
        self.assertEqual(vl, 0.0)

    def test_corner_radius_and_curvature_limit(self):
        # Curva de 90 graus: (0, 0) -> (1, 0) -> (1, 1)
        p1 = (0.0, 0.0)
        p2 = (1.0, 0.0)
        p3 = (1.0, 1.0)
        radius = self.traj_gen._compute_corner_radius(p1, p2, p3)

        # Raio circunscrito aos 3 pontos deve ser finito e mensurável (~0.707 m)
        self.assertFalse(math.isinf(radius))
        self.assertAlmostEqual(radius, 0.7071, places=2)

        # Teste de colineares: (0, 0) -> (1, 0) -> (2, 0) deve dar raio infinito
        inf_radius = self.traj_gen._compute_corner_radius((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))
        self.assertTrue(math.isinf(inf_radius))

    def test_target_yaw_decoupling(self):
        # Robô se movendo em X para (2.0, 0.0), mas com target_yaw fixado em 90 graus (pi/2)
        path = [(0.0, 0.0), (2.0, 0.0)]
        target_yaw = math.pi / 2.0
        vf, vl, vw = self.traj_gen.compute_step(path, (0.0, 0.0, 0.0), target_yaw=target_yaw, v0=(1.0, 0.0))

        # A velocidade angular deve ser positiva para girar em direção a pi/2
        self.assertGreater(vw, 0.0)
        # O robô continua acelerando para frente em X (v_forward > 0)
        self.assertGreater(vf, 0.0)


if __name__ == "__main__":
    unittest.main()

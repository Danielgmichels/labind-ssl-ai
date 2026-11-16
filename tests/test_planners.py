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


if __name__ == "__main__":
    unittest.main()

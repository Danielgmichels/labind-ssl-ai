import os
import sys
import unittest
import math
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(PROJECT_ROOT, "proto_msg"))

from world.world_model import WorldModel
from world.Blackboard import Blackboard
from navigation.APF import ProportionalController
from behavior_tree.conditions import ConditionIsUnderPressure, ConditionIsNearBall
from behavior_tree.actions import ActionShieldBall, ActionFindShootingAngle
from behavior_tree.trees.attacker import build_attacker_tree
from behavior_tree.core import NodeState


class MockActionClient:
    def __init__(self):
        self.last_command = None

    def send_command(self, robot_id, v_forward, v_left, vw, kick_speed=0.0, dribbler_speed=0.0):
        self.last_command = {
            "robot_id": robot_id,
            "v_forward": v_forward,
            "v_left": v_left,
            "vw": vw,
            "kick_speed": kick_speed,
            "dribbler_speed": dribbler_speed
        }


class MockRobotPos:
    def __init__(self, x, y, yaw=0.0, r_id=1):
        self.id = r_id
        self.yaw = yaw
        self.pos = type('Pos', (), {'x': x, 'y': y})()


class TestAPFDecoupledRotation(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)

    def test_default_rotation_faces_target(self):
        # Robô em (0, 0, yaw=0), alvo em (1, 1). Sem target_yaw, aponta para pi/4 (~0.785 rad)
        vf, vl, vw = self.controller.calculate_velocity(
            0.0, 0.0, 0.0, 1.0, 1.0, []
        )
        expected_yaw = math.atan2(1.0, 1.0)
        expected_vw = expected_yaw * self.controller.kp_angular
        self.assertAlmostEqual(vw, expected_vw, places=2)

    def test_explicit_target_yaw_decouples_rotation(self):
        # Alvo de atração continua em (1, 1), mas especificamos target_yaw = -pi/2
        target_yaw = -math.pi / 2.0
        vf, vl, vw = self.controller.calculate_velocity(
            0.0, 0.0, 0.0, 1.0, 1.0, [], target_yaw=target_yaw
        )
        expected_vw = target_yaw * self.controller.kp_angular
        self.assertAlmostEqual(vw, expected_vw, places=2)


class TestConditionIsUnderPressure(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action_client = MockActionClient()
        self.bb = Blackboard(self.controller, self.action_client)
        self.bb.my_pos = MockRobotPos(0.0, 0.0, yaw=0.0, r_id=1)
        self.cond = ConditionIsUnderPressure(danger_radius=0.75)

    def test_enemy_within_danger_radius_triggers_success(self):
        enemy = MockRobotPos(0.50, 0.0, r_id=9)
        self.bb.enemies = [enemy]

        status = self.cond.tick(self.bb)
        self.assertEqual(status, NodeState.SUCCESS)
        self.assertIsNotNone(self.bb.threatening_enemy)
        self.assertEqual(self.bb.threatening_enemy.id, 9)
        self.assertAlmostEqual(self.bb.threat_distance, 0.50, places=2)

    def test_enemy_outside_danger_radius_triggers_failure(self):
        enemy = MockRobotPos(1.20, 0.0, r_id=9)
        self.bb.enemies = [enemy]

        status = self.cond.tick(self.bb)
        self.assertEqual(status, NodeState.FAILURE)
        self.assertIsNone(self.bb.threatening_enemy)

    def test_multiple_enemies_picks_closest(self):
        enemy_far = MockRobotPos(0.70, 0.0, r_id=7)
        enemy_close = MockRobotPos(0.35, 0.1, r_id=8)
        self.bb.enemies = [enemy_far, enemy_close]

        status = self.cond.tick(self.bb)
        self.assertEqual(status, NodeState.SUCCESS)
        self.assertEqual(self.bb.threatening_enemy.id, 8)


class TestActionShieldBall(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action_client = MockActionClient()
        self.bb = Blackboard(self.controller, self.action_client)
        self.bb.my_id = 1
        self.bb.enemy_goal_x = 6.0
        self.bb.obstacles = []
        self.bb.my_pos = MockRobotPos(0.0, 0.0, yaw=0.0, r_id=1)

    def test_shield_orientation_faces_away_from_enemy(self):
        # Adversário na frente do robô em (0.5, 0.0)
        # O vetor da ameaça é 0 rad. O robô deve virar de costas: target_yaw = pi
        self.bb.threatening_enemy = MockRobotPos(0.5, 0.0, r_id=9)
        self.bb.enemies = [self.bb.threatening_enemy]

        action = ActionShieldBall()
        status = action.tick(self.bb)

        self.assertEqual(status, NodeState.RUNNING)
        cmd = self.action_client.last_command
        self.assertIsNotNone(cmd)
        # Driblador deve estar acionado firme (1500 RPM)
        self.assertEqual(cmd["dribbler_speed"], 1500.0)
        # Rotação angular deve estar levando o robô para longe do ângulo 0 (direção de pi)
        self.assertGreater(abs(cmd["vw"]), 0.1)

    def test_shield_timeout_triggers_failure(self):
        # Teste do odômetro/tempo limite da Divisão A (máx 1.5s)
        action = ActionShieldBall(max_duration=0.1)
        self.bb.threatening_enemy = MockRobotPos(0.4, 0.0, r_id=9)
        self.bb.enemies = [self.bb.threatening_enemy]

        # Primeiro tick inicia a blindagem
        status1 = action.tick(self.bb)
        self.assertEqual(status1, NodeState.RUNNING)

        # Espera expirar o tempo limite configurado
        time.sleep(0.15)
        status2 = action.tick(self.bb)
        # Deve falhar para acionar o passe de recuo e evitar falta de 1 metro
        self.assertEqual(status2, NodeState.FAILURE)

    def test_shield_distance_limit_triggers_failure(self):
        # Teste do odômetro de distância (máx 0.8m)
        action = ActionShieldBall(max_distance=0.5)
        self.bb.threatening_enemy = MockRobotPos(0.4, 0.0, r_id=9)
        self.bb.enemies = [self.bb.threatening_enemy]

        status1 = action.tick(self.bb)
        self.assertEqual(status1, NodeState.RUNNING)

        # Simula que o robô já se deslocou 0.6m
        self.bb.my_pos = MockRobotPos(0.6, 0.0, yaw=0.0, r_id=1)
        status2 = action.tick(self.bb)
        self.assertEqual(status2, NodeState.FAILURE)


class TestFindShootingAngleFrontalCheck(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action_client = MockActionClient()
        self.bb = Blackboard(self.controller, self.action_client)
        self.bb.my_id = 1
        self.bb.enemy_goal_x = 6.0
        self.bb.obstacles = []
        self.bb.my_pos = MockRobotPos(2.0, 0.0, yaw=0.0, r_id=1)

    def test_front_enemy_aborts_strafe_to_protect_ball(self):
        # Robô mirando no gol em x=6.0 (yaw=0). Um adversário fecha a 40cm diretamente à frente.
        self.bb.enemies = [MockRobotPos(2.4, 0.0, r_id=9)]
        strafe = ActionFindShootingAngle(max_duration=1.5)

        status = strafe.tick(self.bb)
        # Deve falhar imediatamente para evitar o choque de roletes e exposição da bola
        self.assertEqual(status, NodeState.FAILURE)


class TestAttackerTreeIntegration(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action_client = MockActionClient()
        self.bb = Blackboard(self.controller, self.action_client)
        self.bb.my_id = 1
        self.bb.enemy_goal_x = 6.0
        self.bb.obstacles = []
        self.bb.referee_command = "RUNNING"
        self.bb.my_pos = MockRobotPos(2.0, 0.0, yaw=0.0, r_id=1)
        self.bb.ball_pos = type('Ball', (), {'x': 2.05, 'y': 0.0, 'v_x': 0.0, 'v_y': 0.0})()
        self.tree = build_attacker_tree()

    def test_attacker_under_pressure_selects_shield_ball(self):
        # Marcador fechando a 0.40m
        self.bb.enemies = [MockRobotPos(2.40, 0.0, r_id=9)]
        
        status = self.tree.tick(self.bb)
        # O ramo ofensivo deve estar rodando (RUNNING)
        self.assertEqual(status, NodeState.RUNNING)
        # O comando enviado deve ter acionado o driblador a 1500 RPM para segurar a bola
        cmd = self.action_client.last_command
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd["dribbler_speed"], 1500.0)
        # A velocidade angular deve estar girando para afastar a frente do adversário
        self.assertNotEqual(cmd["vw"], 0.0)


if __name__ == '__main__':
    unittest.main()

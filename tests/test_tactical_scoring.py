import os
import sys
import unittest

# Garante inclusão do diretório raiz e de proto_msg no path de importação
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(PROJECT_ROOT, "proto_msg"))

from world.world_model import WorldModel
from world.Blackboard import Blackboard
from navigation.APF import ProportionalController
from communication.ActionClient import ActionClient

from strategy.shot_scoring import get_best_shot_decision, _calc_progression as calc_shot_progression
from strategy.pass_scoring import get_best_pass_decision, _calc_progression as calc_pass_progression
from strategy.space_scoring import get_best_free_space_decision
from strategy.Maestro import maestro_distribui_papeis
from behavior_tree.conditions import (
    ConditionEvaluateShot, ConditionEvaluatePass, ConditionEvaluateFreeSpace, ConditionIsPassArriving
)
from behavior_tree.actions import ActionGoToBall, ActionPassBall, ActionPositionForPass
from behavior_tree.core import NodeState
import time


class TestShotScoring(unittest.TestCase):
    def setUp(self):
        self.world = WorldModel(is_yellow=True) # enemy_goal_x = -6.0
        self.world.ball.x = -4.5
        self.world.ball.y = 0.0
        self.world.ball.visible = True

        self.robot = self.world.get_robot(1)
        self.robot.x = -4.3
        self.robot.y = 0.0
        self.robot.yaw = 3.14159
        self.robot.visible = True

    def test_open_goal(self):
        decision = get_best_shot_decision(self.world, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertIsNotNone(decision.best_target_y)
        self.assertGreaterEqual(decision.best_candidate.score, 0.60)

    def test_center_blocked_picks_corner(self):
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -5.5
        opp.y = 0.0
        opp.visible = True

        decision = get_best_shot_decision(self.world, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertGreater(abs(decision.best_candidate.target_y), 0.10)

    def test_all_targets_blocked(self):
        for i, y in enumerate([-0.45, -0.30, -0.15, 0.00, 0.15, 0.30, 0.45]):
            o = self.world.get_robot(i, is_opponent=True)
            o.x = -5.0
            o.y = y
            o.visible = True

        decision = get_best_shot_decision(self.world, 1)
        self.assertIsNone(decision.best_candidate)
        self.assertIsNone(decision.best_target_y)

    def test_teammate_blocks_shot(self):
        mate = self.world.get_robot(2, is_opponent=False)
        mate.x = -5.5
        mate.y = 0.0
        mate.visible = True

        decision = get_best_shot_decision(self.world, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertGreater(abs(decision.best_candidate.target_y), 0.10)

    def test_progression_advancement(self):
        data_near = {"bx": -5.5, "goal_x": -6.0}
        data_mid = {"bx": 0.0, "goal_x": -6.0}
        score_near = calc_shot_progression(0.0, data_near)
        score_mid = calc_shot_progression(0.0, data_mid)
        self.assertGreater(score_near, score_mid)


class TestPassScoring(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action = ActionClient(port=10399)
        self.bb = Blackboard(self.controller, self.action)

        self.world = WorldModel(is_yellow=True)
        self.bb.world_model = self.world

        # Passador ID 1 (Atacante)
        self.passer = self.world.get_robot(1)
        self.passer.x = -2.0
        self.passer.y = 0.0
        self.passer.yaw = 3.14159
        self.passer.visible = True
        self.bb.my_id = 1

        # Receptor 1 (ID 2 - ATACANTE_APOIO_ESQ)
        self.r1 = self.world.get_robot(2)
        self.r1.x = -3.5
        self.r1.y = 1.5
        self.r1.yaw = 0.0
        self.r1.visible = True

        # Receptor 2 (ID 3 - ATACANTE_APOIO_DIR)
        self.r2 = self.world.get_robot(3)
        self.r2.x = -3.5
        self.r2.y = -1.5
        self.r2.yaw = 0.0
        self.r2.visible = True

        self.bb.papeis = {
            0: "GOLEIRO",
            1: "ATACANTE",
            2: "ATACANTE_APOIO_ESQ",
            3: "ATACANTE_APOIO_DIR"
        }

    def tearDown(self):
        if hasattr(self.action, 'sock') and self.action.sock:
            self.action.sock.close()

    def test_open_pass_selection(self):
        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertIn(decision.best_target_robot, [2, 3])
        self.assertGreaterEqual(decision.best_candidate.score, 0.50)

    def test_blocked_passing_lane_rejected(self):
        # Bloqueia a linha para o receptor 2
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -2.75
        opp.y = -0.75
        opp.visible = True

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertEqual(decision.best_target_robot, 2)

    def test_marked_receiver_loses_to_open_receiver(self):
        # Marca fortemente o receptor 1
        opp1 = self.world.get_robot(1, is_opponent=True)
        opp1.x = -3.5
        opp1.y = 1.2
        opp1.visible = True

        opp2 = self.world.get_robot(2, is_opponent=True)
        opp2.x = -3.2
        opp2.y = 1.5
        opp2.visible = True

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertEqual(decision.best_target_robot, 3)

    def test_all_passing_lanes_blocked(self):
        # Bloqueia receptor 2
        opp3 = self.world.get_robot(3, is_opponent=True)
        opp3.x = -2.75
        opp3.y = -0.75
        opp3.visible = True

        # Bloqueia receptor 1
        opp4 = self.world.get_robot(4, is_opponent=True)
        opp4.x = -2.75
        opp4.y = 0.75
        opp4.visible = True

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNone(decision.best_candidate)
        self.assertIsNone(decision.best_target_point)

    def test_condition_evaluate_pass_bt_integration(self):
        cond = ConditionEvaluatePass()
        status = cond.tick(self.bb)
        self.assertEqual(status, NodeState.SUCCESS)
        self.assertIsNotNone(self.bb.pass_target_point)
    def test_forward_pass_preferred_over_backward_pass(self):
        # Receptor 2 à frente (-3.5), Receptor 3 atrás (-1.0)
        self.r1.x = -3.5 # à frente (gol inimigo é em -6.0)
        self.r2.x = -1.0 # atrás
        self.r2.y = 0.5

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        # Deve escolher o receptor à frente devido à penalização da progressão negativa
        self.assertEqual(decision.best_target_robot, 2)

    def test_only_forward_filter(self):
        # Bloqueia receptor 2 à frente (-3.5, 1.5) com obstáculo em (-2.75, 0.75)
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -2.75
        opp.y = 0.75
        opp.visible = True

        self.r2.x = -1.0 # receptor 3 atrás (progression < 0.50)
        self.r2.y = 0.5

        # Com only_forward=True, não deve selecionar o receptor recuado
        cond_forward = ConditionEvaluatePass(only_forward=True)
        status = cond_forward.tick(self.bb)
        self.assertEqual(status, NodeState.FAILURE)

        # Com only_forward=False, o passe para trás é permitido como alternativa
        cond_any = ConditionEvaluatePass(only_forward=False)
        status_any = cond_any.tick(self.bb)
        self.assertEqual(status_any, NodeState.SUCCESS)
        self.assertEqual(self.bb.pass_target_robot, 3)

    def test_pass_lockout_prevents_goto_ball(self):
        # Configura posições no blackboard
        self.bb.my_pos = self.passer
        self.bb.ball_pos = self.world.ball

        # Simula que o robô 1 acabou de realizar um passe há 0.2 segundos
        self.bb.last_pass_time = time.time()
        self.bb.last_passer_id = 1
        self.bb.my_id = 1

        goto_ball = ActionGoToBall()
        status = goto_ball.tick(self.bb)

        # Deve retornar RUNNING mas com comandos zerados para não perseguir a bola
        self.assertEqual(status, NodeState.RUNNING)

    def test_maestro_pass_in_progress_promotes_receiver(self):
        # Robô 1 era o atacante, mas efetuou um passe para o robô 2
        last_roles = {1: "ATACANTE", 2: "ATACANTE_APOIO_ESQ"}
        
        # Cria objetos de robôs simples para o Maestro
        class SimpleRobot:
            def __init__(self, r_id, x, y):
                self.id = r_id
                self.pos = type('Pos', (), {'x': x, 'y': y})()

        robots = [
            SimpleRobot(0, 5.5, 0.0), # Goleiro
            SimpleRobot(1, -2.0, 0.0), # Passador
            SimpleRobot(2, -3.5, 1.5), # Receptor
        ]
        ball = type('Ball', (), {'x': -2.2, 'y': 0.1})() # Bola ainda próxima do passador (-2.2)

        # Sem passe ativo: o robô 1 mantém ATACANTE por proximidade + histerese (50cm)
        papeis_sem_passe = maestro_distribui_papeis(robots, ball, -6.0, last_roles)
        self.assertEqual(papeis_sem_passe[1], "ATACANTE")

        # Com passe em trânsito (< 1.2s) para o robô 2: Maestro transfere ATACANTE para o robô 2
        papeis_com_passe = maestro_distribui_papeis(
            robots, ball, -6.0, last_roles,
            last_pass_time=time.time(), pass_target_robot=2, last_passer_id=1
        )
        self.assertEqual(papeis_com_passe[2], "ATACANTE")

    def test_tight_marking_vetoes_receiver(self):
        # Coloca oponente colado (30cm) no receptor 2 (-3.5, 1.5)
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -3.5
        opp.y = 1.2 # Distância de 0.30m (<= 0.35m)
        opp.visible = True

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        # Receptor 2 deve ser vetado e o passe direcionado ao receptor 3
        self.assertEqual(decision.best_target_robot, 3)

        cand2 = next(c for c in decision.all_candidates if c.receiver_id == 2)
        self.assertEqual(cand2.score, 0.0)
        self.assertIn("marcação colada", cand2.reason)

    def test_opponent_near_receiver_blocking_line_rejected(self):
        # Coloca oponente a 20cm da linha de passe bem próximo do receptor 3 (-3.5, -1.5)
        # Passador em (-2.0, 0.0), segmento dx = -1.5, dy = -1.5
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -3.35 # t ~ 0.90
        opp.y = -1.35
        opp.visible = True

        decision = get_best_pass_decision(self.world, self.bb, 1)
        self.assertIsNotNone(decision.best_candidate)
        self.assertEqual(decision.best_target_robot, 2)

    def test_pass_arriving_extended_range(self):
        # Receptor em (-3.5, 1.5). Bola em (0.5, 1.5) viajando a 2.5 m/s em direção ao receptor (-X)
        self.bb.my_pos = type('RobotMock', (), {
            'id': 2, 'yaw': 0.0,
            'pos': type('Pos', (), {'x': -3.5, 'y': 1.5})()
        })()
        self.bb.my_id = 2
        self.bb.pass_in_progress_target = 2
        self.bb.ball_pos = type('Ball', (), {'x': 0.5, 'y': 1.5})()
        self.bb.ball_vel_x = -2.5
        self.bb.ball_vel_y = 0.0

        cond = ConditionIsPassArriving()
        # Distância é 4.0 metros (> 3.0m anterior, dentro do novo limite de 5.5m)
        status = cond.tick(self.bb)
        self.assertEqual(status, NodeState.SUCCESS)

    def test_goto_ball_active_dribbler_on_pass_reception(self):
        # Simula robô receptor se tornando Atacante durante passe em andamento
        self.bb.my_pos = type('RobotMock', (), {
            'id': 2, 'yaw': 0.0,
            'pos': type('Pos', (), {'x': -3.5, 'y': 1.5})()
        })()
        self.bb.my_id = 2
        self.bb.pass_in_progress_target = 2
        self.bb.last_pass_time = time.time()
        self.bb.ball_pos = self.world.ball

        # Mock na função send_command para verificar dribbler_speed
        last_sent = {}
        def mock_send_command(**kwargs):
            last_sent.update(kwargs)

        self.bb.action.send_command = mock_send_command
        goto_ball = ActionGoToBall()
        goto_ball.tick(self.bb)

        # Driblador deve estar ativo a 1500 RPM para recepção limpa
        self.assertEqual(last_sent.get('dribbler_speed'), 1500.0)


class TestFreeSpacePositioning(unittest.TestCase):
    def setUp(self):
        self.controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
        self.action = ActionClient(port=10397)
        self.bb = Blackboard(self.controller, self.action)

        self.world = WorldModel(is_yellow=True) # enemy_goal_x = -6.0
        self.bb.world_model = self.world

        # Bola na zona intermediária
        self.world.ball.x = -2.0
        self.world.ball.y = 0.0
        self.world.ball.visible = True
        self.bb.ball_pos = self.world.ball

        # Atacante de apoio ID 2
        self.robot = self.world.get_robot(2)
        self.robot.x = -3.0
        self.robot.y = 1.5
        self.robot.yaw = 0.0
        self.robot.visible = True
        self.bb.my_id = 2
        self.bb.my_pos = type('RoboMock', (), {
            'pos': type('Pos', (), {'x': -3.0, 'y': 1.5})(),
            'yaw': 0.0
        })()

    def tearDown(self):
        if hasattr(self.action, 'sock') and self.action.sock:
            self.action.sock.close()

    def test_free_space_identifies_open_position(self):
        decision = get_best_free_space_decision(self.world, self.bb, 2, side_y=1.5)
        self.assertIsNotNone(decision.best_candidate)
        self.assertIsNotNone(decision.best_target_point)
        # Deve estar na ala esquerda (y > 0)
        self.assertGreater(decision.best_target_point[1], 0.0)
        self.assertGreaterEqual(decision.best_candidate.score, 0.35)

    def test_free_space_avoids_blocked_lane(self):
        # Bloqueia a linha da bola (-2.0, 0.0) para a região (-3.5, 1.5)
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -2.75
        opp.y = 0.75
        opp.visible = True

        decision = get_best_free_space_decision(self.world, self.bb, 2, side_y=1.5)
        self.assertIsNotNone(decision.best_candidate)
        # A posição escolhida deve contornar o bloqueio e manter pass_line_score > 0
        self.assertGreater(decision.best_candidate.pass_line_score, 0.0)

    def test_free_space_hysteresis_prevents_unnecessary_switch(self):
        # Primeira decisão
        dec1 = get_best_free_space_decision(self.world, self.bb, 2, side_y=1.5)
        target1 = dec1.best_target_point

        # Nova decisão fornecendo target1 como posição atual (ganha stability_bonus de 0.15)
        dec2 = get_best_free_space_decision(self.world, self.bb, 2, current_target=target1, side_y=1.5)
        self.assertEqual(dec2.best_target_point, target1)

    def test_condition_evaluate_free_space_integration(self):
        cond = ConditionEvaluateFreeSpace(side_y=1.5)
        status = cond.tick(self.bb)
        self.assertEqual(status, NodeState.SUCCESS)
        self.assertIsNotNone(self.bb.free_space_target)
        self.assertIsNotNone(self.bb.free_space_decision)

    def test_action_position_for_pass_consumes_free_space_target(self):
        # Configura um alvo explícito vindo do scoring
        self.bb.free_space_target = (-4.0, 2.0)
        action = ActionPositionForPass(lado_y=1.5)
        status = action.tick(self.bb)
        self.assertEqual(status, NodeState.RUNNING)

    def test_free_space_hard_blocks_candidate(self):
        # Cria um candidato forçadamente na linha de um adversário
        from strategy.space_scoring import evaluate_position
        from strategy.tactical_utils import PositionCandidate
        
        opp = self.world.get_robot(0, is_opponent=True)
        opp.x = -2.5
        opp.y = 0.5
        opp.visible = True

        candidate = PositionCandidate(target_point=(-3.0, 1.0))
        shared_data = {
            'ball_pos': (-2.0, 0.0),
            'goal_x': -6.0,
            'opponents': [(-2.5, 0.5)],
            'teammates': [],
            'pass_obstacles': [(-2.5, 0.5)],
            'robot_id': 2
        }
        evaluate_position(candidate, shared_data)
        self.assertEqual(candidate.score, 0.0)
        self.assertIn("Linha de passe bloqueada", candidate.reason)

    def test_free_space_teammate_blocks_pass_line(self):
        # Coloca companheiro ID 3 no meio da linha para uma certa região
        mate = self.world.get_robot(3, is_opponent=False)
        mate.x = -2.75
        mate.y = 0.75
        mate.visible = True

        decision = get_best_free_space_decision(self.world, self.bb, 2, side_y=1.5)
        self.assertIsNotNone(decision.best_candidate)
        # Deve escolher uma posição cuja linha de passe não bata no companheiro
        self.assertGreater(decision.best_candidate.pass_line_score, 0.0)

    def test_free_space_avoids_clumping_with_teammates(self):
        # Se um companheiro já estiver na posição (-4.8, 1.5)
        mate = self.world.get_robot(3, is_opponent=False)
        mate.x = -4.8
        mate.y = 1.5
        mate.visible = True

        decision = get_best_free_space_decision(self.world, self.bb, 2, side_y=1.5)
        self.assertIsNotNone(decision.best_candidate)
        # O melhor alvo não deve ser exatamente o mesmo do companheiro
        self.assertNotEqual(decision.best_target_point, (-4.8, 1.5))


if __name__ == "__main__":
    unittest.main()


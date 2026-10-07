import os
import sys
import unittest

# Garante inclusão do diretório raiz e de proto_msg no path de importação
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(PROJECT_ROOT, "proto_msg"))

from world.world_model import WorldModel
from strategy.shot_scoring import get_best_shot_decision, _calc_progression as calc_shot_progression


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


if __name__ == "__main__":
    unittest.main()

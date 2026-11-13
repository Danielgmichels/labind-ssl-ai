import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(PROJECT_ROOT, "proto_msg"))

from world.world_model import WorldModel
from world.Blackboard import Blackboard
from navigation.APF import ProportionalController
from communication.ActionClient import ActionClient
from strategy.space_scoring import get_best_free_space_decision, generate_free_space_candidates, evaluate_position
from strategy.tactical_utils import TACTICAL_CONFIG


def render_space_grid(world, bb, robot_id=2, side_y=1.5):
    """
    Renderiza um mapa de calor ASCII no terminal mostrando o campo e os scores
    de cada candidato no grid para visualização imediata da tomada de decisão.
    """
    decision = get_best_free_space_decision(world, bb, robot_id, side_y=side_y)
    best_target = decision.best_target_point

    ball = world.get_ball()
    opponents = [(opp.x, opp.y) for opp in world.get_opponent_team().values() if opp.visible]
    teammates = [(m.x, m.y) for r_id, m in world.get_my_team().items() if m.visible and r_id != robot_id]

    print("\n" + "=" * 65)
    print("        MAPA DE CALOR: FREE-SPACE POSITIONING ENGINE (SSL)")
    print("=" * 65)
    print(f"Bola: ({ball.x:.2f}, {ball.y:.2f}) | Gol Inimigo x: {world.field.enemy_goal_x:.2f}")
    print(f"Oponentes: {opponents}")
    print(f"Companheiros: {teammates}")
    best_score = decision.best_candidate.score if decision.best_candidate else 0.0
    print(f"Alvo Escolhido: {best_target} (Score: {best_score:.2f})")
    print("-" * 65)

    # Organiza candidatos por coordenadas
    scores_dict = {c.target_point: c.score for c in decision.all_candidates}

    xs = sorted(list(set(c.target_point[0] for c in decision.all_candidates)))
    ys = sorted(list(set(c.target_point[1] for c in decision.all_candidates)), reverse=True)

    print("   Y \\ X  " + "".join(f"{x:6.1f}" for x in xs))
    for y in ys:
        row_str = f"{y:6.1f} | "
        for x in xs:
            pt = (round(x, 2), round(y, 2))
            # Verifica se há oponente ou bola perto
            is_opp = any(abs(ox - x) < 0.25 and abs(oy - y) < 0.25 for ox, oy in opponents)
            is_ball = (abs(ball.x - x) < 0.25 and abs(ball.y - y) < 0.25)
            is_best = (best_target is not None and abs(best_target[0] - x) < 0.01 and abs(best_target[1] - y) < 0.01)

            if is_best:
                row_str += "  \033[92m★WIN\033[0m"
            elif is_ball:
                row_str += "  \033[93m●BAL\033[0m"
            elif is_opp:
                row_str += "  \033[91m▲OPP\033[0m"
            elif pt in scores_dict:
                sc = scores_dict[pt]
                if sc <= 0.0:
                    row_str += "  \033[90m .BL\033[0m"
                elif sc < 0.45:
                    row_str += f"  \033[31m{sc:4.2f}\033[0m"
                elif sc < 0.70:
                    row_str += f"  \033[33m{sc:4.2f}\033[0m"
                else:
                    row_str += f"  \033[32m{sc:4.2f}\033[0m"
            else:
                row_str += "      "
        print(row_str)

    print("=" * 65)
    print("Legenda: ★WIN = Alvo Escolhido | ▲OPP = Oponente | ●BAL = Bola | .BL = Bloqueado (Score 0)")
    print("         Verde: Score >= 0.70  | Amarelo: 0.45 <= Score < 0.70 | Vermelho: < 0.45")
    print("=" * 65 + "\n")


if __name__ == "__main__":
    controller = ProportionalController(2.0, 2.0, 2.5, 5.0)
    action = ActionClient(port=10398)
    bb = Blackboard(controller, action)
    world = WorldModel(is_yellow=True) # Gol adversário em -6.0
    bb.world_model = world

    # Cenário de Teste:
    # Bola na intermediária (-2.0, 0.0)
    world.ball.x = -2.0
    world.ball.y = 3.0
    world.ball.visible = True
    bb.ball_pos = world.ball

    # Robô 2 avaliando apoio na ala esquerda (side_y > 0)
    robot = world.get_robot(2)
    robot.x = -3.3
    robot.y = 1.5
    robot.visible = True
    bb.my_id = 2

    # Oponente bloqueando diagonal em (-3.0, 1.0)
    opp = world.get_robot(0, is_opponent=True)
    opp.x = -3.0
    opp.y = 3.0
    opp.visible = True

    # Oponente de marcação em (-4.5, 2.0)
    opp2 = world.get_robot(1, is_opponent=True)
    opp2.x = -4.5
    opp2.y = 2.0
    opp2.visible = True

    render_space_grid(world, bb, robot_id=2, side_y=1.5)

    if hasattr(action, 'sock') and action.sock:
        action.sock.close()

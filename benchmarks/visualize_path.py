import os
import sys
import time
import math
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Tuple, Dict, Any, Optional

# Garante acesso à raiz do projeto e proto_msg
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
PROTO_MSGS_DIR = os.path.join(PROJECT_ROOT, "proto_msg")

if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)
if PROTO_MSGS_DIR not in sys.path:
    sys.path.append(PROTO_MSGS_DIR)

# Auto-injeção das dependências do ambiente virtual local (.venv), garantindo acesso a zmq, matplotlib, etc.
venv_site_packages = os.path.join(PROJECT_ROOT, ".venv", "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
if os.path.isdir(venv_site_packages) and venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

from navigation.factory import get_path_planner, _PLANNER_REGISTRY


def draw_ssl_field(ax, field_length: float = 12.0, field_width: float = 9.0):
    """Desenha a geometria oficial do campo da RoboCup SSL em um eixo Matplotlib."""
    half_l = field_length / 2.0
    half_w = field_width / 2.0

    # Fundo verde do carpete
    ax.set_facecolor("#1e5a22")

    # Linhas externas do campo (retângulo principal)
    rect = patches.Rectangle(
        (-half_l, -half_w), field_length, field_width,
        linewidth=2, edgecolor="white", facecolor="none", zorder=2
    )
    ax.add_patch(rect)

    # Linha do meio de campo
    ax.plot([0, 0], [-half_w, half_w], color="white", linewidth=2, zorder=2)

    # Círculo central (raio 0.5m na SSL)
    center_circle = patches.Circle((0, 0), 0.5, edgecolor="white", facecolor="none", linewidth=2, zorder=2)
    ax.add_patch(center_circle)
    ax.plot(0, 0, marker="o", markersize=4, color="white", zorder=2)

    # Áreas de Defesa (Retângulos padrão SSL: 1.0m profundidade x 2.4m largura)
    # Área Esquerda
    area_left = patches.Rectangle(
        (-half_l, -1.2), 1.0, 2.4,
        linewidth=2, edgecolor="white", facecolor="none", zorder=2
    )
    ax.add_patch(area_left)

    # Área Direita
    area_right = patches.Rectangle(
        (half_l - 1.0, -1.2), 1.0, 2.4,
        linewidth=2, edgecolor="white", facecolor="none", zorder=2
    )
    ax.add_patch(area_right)

    # Caixas das Balizas / Gols (0.2m profundidade x 1.0m largura)
    goal_left = patches.Rectangle((-half_l - 0.2, -0.5), 0.2, 1.0, edgecolor="yellow", facecolor="none", linewidth=2, zorder=2)
    goal_right = patches.Rectangle((half_l, -0.5), 0.2, 1.0, edgecolor="blue", facecolor="none", linewidth=2, zorder=2)
    ax.add_patch(goal_left)
    ax.add_patch(goal_right)

    # Limites navegáveis de segurança (Field Bounding Box: 5.8m x 4.3m)
    safe_box = patches.Rectangle(
        (-5.8, -4.3), 11.6, 8.6,
        linewidth=1, edgecolor="#88ff88", facecolor="none", linestyle="--", alpha=0.5, zorder=2
    )
    ax.add_patch(safe_box)

    ax.set_xlim(-half_l - 0.6, half_l + 0.6)
    ax.set_ylim(-half_w - 0.6, half_w + 0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("X (metros)")
    ax.set_ylabel("Y (metros)")
    ax.grid(True, linestyle=":", alpha=0.3, color="white")


def plot_path_scenario(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Dict[str, Any]],
    forbidden_zones: Optional[List[Dict[str, float]]] = None,
    ball: Optional[Tuple[float, float]] = None,
    active_robot_info: Optional[Dict[str, Any]] = None,
    path: Optional[List[Tuple[float, float]]] = None,
    title: str = "Planejamento de Trajetória SSL",
    save_path: Optional[str] = None,
    show: bool = True
):
    """Renderiza a visualização 2D completa do cenário de navegação."""
    fig, ax = plt.subplots(figsize=(12, 8))
    draw_ssl_field(ax)

    # 1. Desenha as Zonas Proibidas (Áreas de Gol)
    if forbidden_zones:
        for zone in forbidden_zones:
            zx = zone['xmin']
            zy = zone['ymin']
            zw = zone['xmax'] - zone['xmin']
            zh = zone['ymax'] - zone['ymin']
            box = patches.Rectangle(
                (zx, zy), zw, zh,
                edgecolor="#ff8800", facecolor="#ff8800", alpha=0.25, linestyle="--", linewidth=1.5, zorder=3
            )
            ax.add_patch(box)

    # 2. Desenha os obstáculos (robôs não-ativos)
    for obs in obstacles:
        ox, oy = obs['x'], obs['y']
        phys_r = obs.get('radius', 0.09)
        safe_r = 0.22  # Raio inflado padrão para robô (2 * r_bot + folga)
        is_yellow = obs.get('is_yellow', False)

        # Círculo inflado de segurança
        inflated_circle = patches.Circle(
            (ox, oy), safe_r, edgecolor="#ff4444", facecolor="#ff4444", alpha=0.20, linestyle="--", zorder=3
        )
        ax.add_patch(inflated_circle)

        # Corpo físico do robô (Distingue cor de time se disponível)
        bot_color = "#e6b800" if is_yellow else "#0077cc"
        phys_circle = patches.Circle(
            (ox, oy), phys_r, edgecolor="white", facecolor=bot_color, alpha=0.9, zorder=4
        )
        ax.add_patch(phys_circle)

        # Rótulo de ID
        if 'id' in obs:
            ax.text(ox, oy, str(obs['id']), color="white", fontsize=8, weight="bold",
                    ha="center", va="center", zorder=5)

        # Vetor de velocidade se houver
        vx = obs.get('vx', 0.0)
        vy = obs.get('vy', 0.0)
        if abs(vx) > 0.05 or abs(vy) > 0.05:
            ax.arrow(ox, oy, vx * 0.2, vy * 0.2, head_width=0.08, head_length=0.08, fc="yellow", ec="yellow", zorder=5)

    # 3. Desenha o Robô Ativo Selecionado (Destaque visual especial)
    if active_robot_info is not None:
        r_id = active_robot_info.get('id', '?')
        is_yellow = active_robot_info.get('is_yellow', False)
        bot_color = "#e6b800" if is_yellow else "#0077cc"
        team_str = "Amarelo" if is_yellow else "Azul"

        # Halo verde neon indicando robô ativo
        halo = patches.Circle(
            (start[0], start[1]), 0.14, edgecolor="#00ff66", facecolor="none", linewidth=2.5, linestyle="-", zorder=6
        )
        ax.add_patch(halo)
        # Corpo do robô ativo
        active_body = patches.Circle(
            (start[0], start[1]), 0.09, edgecolor="white", facecolor=bot_color, alpha=1.0, zorder=7
        )
        ax.add_patch(active_body)
        ax.text(start[0], start[1], str(r_id), color="white", fontsize=9, weight="bold",
                ha="center", va="center", zorder=8)
        ax.text(start[0], start[1] - 0.25, f"Ativo: {team_str} #{r_id}", color="#00ff66", fontsize=9,
                weight="bold", ha="center", zorder=8)
    else:
        ax.plot(start[0], start[1], marker="o", markersize=10, color="#00ff00", label="Início (Start)", zorder=6)

    # 4. Desenha a Bola
    if ball is not None:
        ball_circle = patches.Circle(
            (ball[0], ball[1]), 0.043, edgecolor="black", facecolor="#ff6600", linewidth=1.5, zorder=6
        )
        ax.add_patch(ball_circle)
        ax.text(ball[0], ball[1] + 0.15, "Bola", color="#ffaa55", fontsize=9, weight="bold", ha="center", zorder=6)

    # 5. Desenha a Meta (Alvo)
    ax.plot(goal[0], goal[1], marker="*", markersize=14, color="#ffff00", label="Alvo (Goal)", zorder=7)

    # 6. Desenha o Caminho Planejado
    if path and len(path) > 0:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, color="#00ffff", linewidth=2.5, linestyle="-", label="Caminho Planejado", zorder=5)
        ax.scatter(px, py, color="#ffffff", s=25, zorder=6)

    ax.set_title(title, fontsize=14, color="black", weight="bold")
    ax.legend(loc="upper right", framealpha=0.85)

    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"[visualize_path] Gráfico salvo em: {save_path}")

    if show:
        plt.show()
    plt.close(fig)


def get_default_benchmark_scenario() -> Tuple[Tuple[float, float], Tuple[float, float], List[Dict[str, Any]]]:
    """Retorna um cenário clássico de teste com múltiplos robôs obstruindo o trajeto."""
    start = (-3.5, 0.0)
    goal = (3.5, 0.0)
    obstacles = [
        {'x': -2.0, 'y': 0.1, 'radius': 0.09, 'vx': 0.0, 'vy': 0.0},
        {'x': -0.5, 'y': -0.2, 'radius': 0.09, 'vx': 0.0, 'vy': 0.0},
        {'x': 0.8, 'y': 0.3, 'radius': 0.09, 'vx': 0.0, 'vy': 0.0},
        {'x': 2.0, 'y': -0.1, 'radius': 0.09, 'vx': 0.0, 'vy': 0.0},
        {'x': -0.5, 'y': 1.2, 'radius': 0.09, 'vx': 0.0, 'vy': -0.5},
        {'x': 1.0, 'y': -1.4, 'radius': 0.09, 'vx': 0.0, 'vy': 0.5},
    ]
    return start, goal, obstacles


def capture_live_grsim_state(ip: str = "127.0.0.1", port: int = 5558, timeout: float = 1.5):
    """Conecta ao VisionClient e obtém o estado ao vivo atual do grSim."""
    try:
        from communication.VisionClient import VisionClient
        vc = VisionClient(ip=ip, port=port)
    except Exception as e:
        print(f"[visualize_path] Falha ao conectar ao VisionClient: {e}")
        return None

    t0 = time.time()
    state = None
    while time.time() - t0 < timeout:
        state = vc.get_latest_state()
        if state is not None:
            break
        time.sleep(0.03)

    return state


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualizador 2D do Campo SSL e Planejamento de Trajetória")
    parser.add_argument("--planner", type=str, default="C_VG", help="Nome do planejador (C_VG, VG_ASTAR, RRT_STAR, etc.)")
    parser.add_argument("--live", action="store_true", help="Captura o snapshot real das posições atuais do grSim via VisionClient")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP do VisionClient (padrão 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5558, help="Porta do VisionClient ZMQ (padrão 5558)")
    parser.add_argument("-t", "--team", type=str, default="blue", help="Time do robô ativo: 'blue'/'azul' ou 'yellow'/'amarelo' (padrão: blue)")
    parser.add_argument("-r", "--robot-id", "--robot", type=int, default=None, help="ID do robô para testar (ex: 0, 1, 2...; se omitido, escolhe o mais próximo da bola)")
    parser.add_argument("-gx", "--goal-x", type=float, default=None, help="Coordenada X customizada da meta")
    parser.add_argument("-gy", "--goal-y", type=float, default=None, help="Coordenada Y customizada da meta")
    parser.add_argument("--target", type=str, default=None, choices=["ball", "enemy_goal", "our_goal", "center"], help="Tipo de alvo pré-definido: 'ball', 'enemy_goal', 'our_goal', 'center'")
    parser.add_argument("--save", type=str, default=None, help="Caminho para salvar imagem (ex: benchmarks/results/test.png)")
    parser.add_argument("--no-show", action="store_true", help="Não exibir a janela interativa do matplotlib")
    args = parser.parse_args()

    # Normalização do time escolhido
    team_raw = str(args.team).lower().strip()
    is_blue_selected = team_raw in ["blue", "azul", "b"]
    team_name = "Azul" if is_blue_selected else "Amarelo"

    forbidden_zones = [
        {'xmin': -6.0, 'xmax': -5.0, 'ymin': -1.2, 'ymax': 1.2},
        {'xmin': 5.0, 'xmax': 6.0, 'ymin': -1.2, 'ymax': 1.2}
    ]

    ball_pos = None
    start = None
    goal = None
    obstacles = []
    active_bot = None
    goal_desc = "Meta"

    if args.live:
        print(f"[visualize_path] Escutando pacotes de visão ao vivo do grSim em {args.ip}:{args.port}...")
        state = capture_live_grsim_state(ip=args.ip, port=args.port)
        if state is not None and hasattr(state, 'last_seen_world'):
            world_data = state.last_seen_world

            # Bola
            if hasattr(world_data, 'ball') and world_data.ball.visible:
                ball_pos = (world_data.ball.pos.x, world_data.ball.pos.y)

            # Robôs Amarelos
            yellow_bots = []
            for r in getattr(world_data, 'yellow', []):
                if abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001:
                    yellow_bots.append({'id': r.id, 'x': r.pos.x, 'y': r.pos.y, 'is_yellow': True, 'radius': 0.09})

            # Robôs Azuis
            blue_bots = []
            for r in getattr(world_data, 'blue', []):
                if abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001:
                    blue_bots.append({'id': r.id, 'x': r.pos.x, 'y': r.pos.y, 'is_yellow': False, 'radius': 0.09})

            # Listagem de robôs detectados
            blue_ids = sorted([b['id'] for b in blue_bots])
            yellow_ids = sorted([b['id'] for b in yellow_bots])
            print(f"[visualize_path] Robôs detectados no simulador:")
            print(f"  -> Time Azul:    {blue_ids if blue_ids else 'Nenhum'}")
            print(f"  -> Time Amarelo: {yellow_ids if yellow_ids else 'Nenhum'}")
            print(f"  -> Time Selecionado para Teste: {team_name.upper()}")

            my_team = blue_bots if is_blue_selected else yellow_bots
            other_team = yellow_bots if is_blue_selected else blue_bots

            # Escolhe o robô ativo
            if args.robot_id is not None:
                for b in my_team:
                    if b['id'] == args.robot_id:
                        active_bot = b
                        break
                if active_bot is None:
                    print(f"[visualize_path] AVISO: Robô ID {args.robot_id} não encontrado no time {team_name}!")
                    if my_team:
                        print(f"[visualize_path] Robôs disponíveis no time {team_name}: {[b['id'] for b in my_team]}")
                    other_has = [b['id'] for b in other_team if b['id'] == args.robot_id]
                    if other_has:
                        other_name = "Amarelo" if is_blue_selected else "Azul"
                        print(f"[visualize_path] Dica: O robô ID {args.robot_id} está no time {other_name}! Use '--team {other_name.lower()}'.")

            if active_bot is None and my_team:
                if ball_pos is not None:
                    active_bot = min(my_team, key=lambda b: math.hypot(b['x'] - ball_pos[0], b['y'] - ball_pos[1]))
                    print(f"[visualize_path] Robô ativo automático (mais perto da bola): ID {active_bot['id']}")
                else:
                    active_bot = my_team[0]
                    print(f"[visualize_path] Robô ativo automático (primeiro do time): ID {active_bot['id']}")

            if active_bot is not None:
                start = (active_bot['x'], active_bot['y'])
                for b in my_team:
                    if b['id'] != active_bot['id']:
                        obstacles.append(b)
                obstacles.extend(other_team)
                print(f"[visualize_path] Robô ativo em teste: ID {active_bot['id']} ({team_name}) em ({start[0]:.2f}, {start[1]:.2f})")

            # Meta / Alvo
            if args.goal_x is not None and args.goal_y is not None:
                goal = (args.goal_x, args.goal_y)
                goal_desc = f"Ponto ({goal[0]:.2f}, {goal[1]:.2f})"
            elif args.target == "enemy_goal":
                enemy_gx = 6.0 if is_blue_selected else -6.0
                goal = (enemy_gx, 0.0)
                goal_desc = f"Gol Inimigo ({goal[0]:.2f}, {goal[1]:.2f})"
            elif args.target == "our_goal":
                our_gx = -6.0 if is_blue_selected else 6.0
                goal = (our_gx, 0.0)
                goal_desc = f"Nosso Gol ({goal[0]:.2f}, {goal[1]:.2f})"
            elif args.target == "center":
                goal = (0.0, 0.0)
                goal_desc = "Centro do Campo (0.0, 0.0)"
            elif ball_pos is not None:
                goal = ball_pos
                goal_desc = f"Bola ({goal[0]:.2f}, {goal[1]:.2f})"
            else:
                goal = (0.0, 0.0)
                goal_desc = "Centro do Campo (0.0, 0.0)"

            print(f"[visualize_path] Alvo da rota: {goal_desc}")
            print(f"[visualize_path] Snapshot grSim capturado com sucesso! Total robôs em campo: {len(obstacles) + (1 if active_bot else 0)}")
        else:
            print("[visualize_path] Aviso: Nenhum dado recebido do grSim (simulador desligado?). Usando cenário padrão.")

    if start is None or goal is None:
        start, goal, obstacles = get_default_benchmark_scenario()
        active_bot = {'id': 0, 'is_yellow': False}
        goal_desc = "Meta Padrão (3.5, 0.0)"

    path = None
    if args.planner.upper() != "MOCK":
        try:
            planner = get_path_planner(args.planner)
            path = planner.plan(start, goal, obstacles, forbidden_zones=forbidden_zones)
            print(f"[visualize_path] Rota calculada com sucesso usando '{args.planner.upper()}': {len(path)} waypoints")
        except Exception as e:
            print(f"[visualize_path] Erro no planejador '{args.planner}' ({e}). Usando reta baseline.")
            path = [start, goal]
    else:
        path = [start, (-2.0, 0.6), (0.0, 0.6), (2.0, 0.6), goal]

    plot_title = f"Campo SSL 2D ({'grSim Live' if args.live and ball_pos else 'Benchmark'}) - Robô {active_bot['id'] if active_bot else '?'} ({team_name}) -> {goal_desc} [{args.planner.upper()}]"

    plot_path_scenario(
        start=start,
        goal=goal,
        obstacles=obstacles,
        forbidden_zones=forbidden_zones,
        ball=ball_pos,
        active_robot_info=active_bot,
        path=path,
        title=plot_title,
        save_path=args.save,
        show=not args.no_show
    )

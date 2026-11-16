import os
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Tuple, Dict, Any, Optional

# Garante acesso à raiz do projeto
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

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

    # Áreas de Defesa (Retângulos padrão SSL: 1.0m profundidade x 2.0m largura)
    # Área Esquerda
    area_left = patches.Rectangle(
        (-half_l, -1.0), 1.0, 2.0,
        linewidth=2, edgecolor="white", facecolor="none", zorder=2
    )
    ax.add_patch(area_left)

    # Área Direita
    area_right = patches.Rectangle(
        (half_l - 1.0, -1.0), 1.0, 2.0,
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
    path: Optional[List[Tuple[float, float]]] = None,
    title: str = "Planejamento de Trajetória SSL",
    save_path: Optional[str] = None,
    show: bool = True
):
    """Renderiza a visualização 2D completa do cenário de navegação."""
    fig, ax = plt.subplots(figsize=(12, 8))
    draw_ssl_field(ax)

    # 1. Desenha os obstáculos
    for obs in obstacles:
        ox, oy = obs['x'], obs['y']
        phys_r = obs.get('radius', 0.09)
        safe_r = 0.22 # Raio inflado padrão para robô (2 * r_bot + folga)

        # Círculo inflado de segurança
        inflated_circle = patches.Circle(
            (ox, oy), safe_r, edgecolor="#ff4444", facecolor="#ff4444", alpha=0.25, linestyle="--", zorder=3
        )
        ax.add_patch(inflated_circle)

        # Corpo físico do robô
        phys_circle = patches.Circle(
            (ox, oy), phys_r, edgecolor="black", facecolor="#dd2222", alpha=0.9, zorder=4
        )
        ax.add_patch(phys_circle)

        # Vetor de velocidade se houver
        vx = obs.get('vx', 0.0)
        vy = obs.get('vy', 0.0)
        if abs(vx) > 0.05 or abs(vy) > 0.05:
            ax.arrow(ox, oy, vx * 0.2, vy * 0.2, head_width=0.08, head_length=0.08, fc="yellow", ec="yellow", zorder=5)

    # 2. Desenha o Ponto de Partida e Meta
    ax.plot(start[0], start[1], marker="o", markersize=10, color="#00ff00", label="Início (Start)", zorder=6)
    ax.plot(goal[0], goal[1], marker="*", markersize=14, color="#ffff00", label="Alvo (Goal)", zorder=6)

    # 3. Desenha o Caminho Planejado
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualizador 2D do Campo SSL e Planejamento de Trajetória")
    parser.add_argument("--planner", type=str, default="MOCK", help="Nome do planejador (C_VG, VG_ASTAR, RRT_STAR, etc.)")
    parser.add_argument("--save", type=str, default=None, help="Caminho para salvar imagem (ex: benchmarks/results/test.png)")
    parser.add_argument("--no-show", action="store_true", help="Não exibir a janela interativa do matplotlib")
    args = parser.parse_args()

    start, goal, obstacles = get_default_benchmark_scenario()
    path = None

    if args.planner.upper() != "MOCK":
        try:
            planner = get_path_planner(args.planner)
            path = planner.plan(start, goal, obstacles, forbidden_zones=[])
        except Exception as e:
            print(f"[visualize_path] Aviso: Não foi possível planejar com '{args.planner}' ({e}). Usando reta baseline.")
            path = [start, goal]
    else:
        path = [start, (-2.0, 0.6), (0.0, 0.6), (2.0, 0.6), goal]

    plot_path_scenario(
        start=start,
        goal=goal,
        obstacles=obstacles,
        path=path,
        title=f"Campo SSL 2D - Algoritmo: {args.planner.upper()}",
        save_path=args.save,
        show=not args.no_show
    )

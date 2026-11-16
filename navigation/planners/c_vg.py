import math
import heapq
from typing import List, Tuple, Dict, Any, Optional

from navigation.interfaces import IPathPlanner
from navigation.factory import register_path_planner


@register_path_planner("C_VG")
class CircularVisibilityGraphPlanner(IPathPlanner):
    """
    Grafo de Visibilidade Circular com Busca A* Lazy (C-VG) para a RoboCup SSL.
    Baseado em Maximo et al. (ITAndroids / SBR 2025).

    Vantagens sobre grafos poligonais clássicos:
    - Modela os robôs cilíndricos como círculos perfeitos, sem quinas artificiais.
    - O caminho é composto por tangentes analíticas e arcos circulares suaves.
    - Execução ultrarrápida (< 0.5 ms com até 11 robôs em campo) conectando
      estritamente bitangentes e arcos perimétricos sem explosão combinatória.
    """

    def __init__(
        self,
        robot_radius: float = 0.09,
        safety_margin: float = 0.13,
        arc_step_deg: float = 25.0
    ):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.arc_step_rad = math.radians(arc_step_deg)

    def plan(
        self,
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
        obstacles: List[Dict[str, Any]],
        forbidden_zones: List[Dict[str, float]]
    ) -> List[Tuple[float, float]]:
        """
        Calcula a rota livre de colisão conectando start_pos a goal_pos.
        """
        # 1. Pré-processamento e extração de círculos inflados válidos
        circles = self._extract_inflated_circles(obstacles)

        # 2. Desatolamento se start ou goal estiverem dentro de algum círculo
        start = self._unstuck_point(start_pos, circles)
        goal = self._unstuck_point(goal_pos, circles, ref_point=start)

        # 3. OTIMIZAÇÃO LAZY: Se a reta direta estiver livre, retorna imediatamente em O(N)!
        if not self._is_segment_blocked(start, goal, circles, forbidden_zones):
            return [start, goal]

        # 4. Filtra círculos relevantes (próximos da reta start-goal) para limitar CPU
        relevant_circles = self._filter_relevant_circles(start, goal, circles)
        if not relevant_circles:
            return [start, goal]

        # 5. Estrutura do Grafo C-VG
        # Mapeamento de nós: id_node -> (x, y)
        nodes: Dict[int, Tuple[float, float]] = {0: start, 1: goal}
        # Lista de adjacência: id_node -> [(neighbor_id, cost, waypoints_list)]
        adj: Dict[int, List[Tuple[int, float, List[Tuple[float, float]]]]] = {0: [], 1: []}

        # Armazena os nós criados em cada círculo: c_idx -> [(node_id, angle_rad)]
        circle_nodes: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(relevant_circles))}

        # 6. Tangentes a partir de Start e para Goal
        self._add_terminal_tangents(start, 0, goal, 1, relevant_circles, forbidden_zones, nodes, adj, circle_nodes)

        # 7. Bitangentes Analíticas entre Pares de Círculos (apenas 4 por par!)
        self._add_circle_bitangents(relevant_circles, forbidden_zones, nodes, adj, circle_nodes)

        # 8. Arcos Circulares conectando nós adjacentes no mesmo círculo
        self._add_perimeter_arcs(relevant_circles, forbidden_zones, nodes, adj, circle_nodes)

        # 9. Busca A* com Heurística Euclidiana
        path = self._astar_search(0, 1, nodes, adj)

        if not path:
            return [start, goal]

        return path

    def _extract_inflated_circles(self, obstacles: List[Dict[str, Any]]) -> List[Tuple[float, float, float]]:
        """Retorna lista de (cx, cy, r_inflated) para cada obstáculo."""
        circles = []
        for obs in obstacles:
            ox = float(obs.get('x', 0.0))
            oy = float(obs.get('y', 0.0))
            r = float(obs.get('radius', self.robot_radius))
            r_inflated = r + self.robot_radius + self.safety_margin
            circles.append((ox, oy, r_inflated))
        return circles

    def _unstuck_point(
        self,
        p: Tuple[float, float],
        circles: List[Tuple[float, float, float]],
        ref_point: Optional[Tuple[float, float]] = None
    ) -> Tuple[float, float]:
        """Projeta o ponto para fora da borda do círculo se estiver dentro."""
        px, py = p
        for cx, cy, r in circles:
            dist = math.hypot(px - cx, py - cy)
            if dist < r:
                if dist > 1e-4:
                    nx = (px - cx) / dist
                    ny = (py - cy) / dist
                elif ref_point is not None:
                    rdx = px - ref_point[0]
                    rdy = py - ref_point[1]
                    rdist = math.hypot(rdx, rdy)
                    nx = rdx / rdist if rdist > 1e-4 else 1.0
                    ny = rdy / rdist if rdist > 1e-4 else 0.0
                else:
                    nx, ny = 1.0, 0.0

                px = cx + nx * (r + 0.01)
                py = cy + ny * (r + 0.01)
        return px, py

    def _filter_relevant_circles(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        circles: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """Limita aos círculos que estão em um corredor razoável em torno de start-goal."""
        sx, sy = start
        gx, gy = goal
        min_x = min(sx, gx) - 1.2
        max_x = max(sx, gx) + 1.2
        min_y = min(sy, gy) - 1.2
        max_y = max(sy, gy) + 1.2

        relevant = []
        for c in circles:
            cx, cy, r = c
            if min_x <= cx <= max_x and min_y <= cy <= max_y:
                relevant.append(c)
        return relevant

    def _is_segment_blocked(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        circles: List[Tuple[float, float, float]],
        forbidden_zones: List[Dict[str, float]],
        ignore_c1: Optional[int] = None,
        ignore_c2: Optional[int] = None
    ) -> bool:
        """Verifica se o segmento colide com algum círculo ou zona proibida."""
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        a = dx * dx + dy * dy

        if a < 1e-8:
            return False

        # 1. Colisão com círculos
        for idx, (cx, cy, r) in enumerate(circles):
            if idx == ignore_c1 or idx == ignore_c2:
                continue

            fx = x1 - cx
            fy = y1 - cy
            b = 2.0 * (dx * fx + dy * fy)
            c = fx * fx + fy * fy - r * r

            disc = b * b - 4.0 * a * c
            if disc > 1e-5:
                sqrt_disc = math.sqrt(disc)
                t1 = (-b - sqrt_disc) / (2.0 * a)
                t2 = (-b + sqrt_disc) / (2.0 * a)
                if t1 < 0.999 and t2 > 0.001:
                    return True

        # 2. Colisão com zonas proibidas
        for zone in forbidden_zones:
            if self._segment_intersects_box(p1, p2, zone):
                return True

        return False

    def _segment_intersects_box(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        zone: Dict[str, float]
    ) -> bool:
        """Verifica se o segmento cruza a caixa retangular inflada da zona proibida."""
        margin = self.robot_radius
        xmin = zone['xmin'] - margin
        xmax = zone['xmax'] + margin
        ymin = zone['ymin'] - margin
        ymax = zone['ymax'] + margin

        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        t_min, t_max = 0.0, 1.0

        for p_edge, d_edge in [(xmin - x1, -dx), (x1 - xmax, dx), (ymin - y1, -dy), (y1 - ymax, dy)]:
            if abs(d_edge) < 1e-8:
                if p_edge > 0:
                    return False
            else:
                t = p_edge / d_edge
                if d_edge < 0:
                    if t > t_min:
                        t_min = t
                else:
                    if t < t_max:
                        t_max = t
                if t_min > t_max:
                    return False

        return t_max > 0.001 and t_min < 0.999

    def _add_terminal_tangents(
        self,
        start: Tuple[float, float],
        start_id: int,
        goal: Tuple[float, float],
        goal_id: int,
        circles: List[Tuple[float, float, float]],
        forbidden_zones: List[Dict[str, float]],
        nodes: Dict[int, Tuple[float, float]],
        adj: Dict[int, List[Tuple[int, float, List[Tuple[float, float]]]]],
        circle_nodes: Dict[int, List[Tuple[int, float]]]
    ):
        """Conecta start aos pontos de tangência dos círculos e os círculos ao goal."""
        for c_idx, (cx, cy, r) in enumerate(circles):
            # Tangentes de start para c_idx
            t1, t2 = self._point_to_circle_tangents(start, (cx, cy, r))
            for pt in [t1, t2]:
                if pt is not None and not self._is_segment_blocked(start, pt, circles, forbidden_zones, ignore_c2=c_idx):
                    nid = len(nodes)
                    nodes[nid] = pt
                    adj[nid] = []
                    ang = math.atan2(pt[1] - cy, pt[0] - cx)
                    circle_nodes[c_idx].append((nid, ang))

                    dist = math.hypot(pt[0] - start[0], pt[1] - start[1])
                    adj[start_id].append((nid, dist, [pt]))

            # Tangentes de goal para c_idx
            gt1, gt2 = self._point_to_circle_tangents(goal, (cx, cy, r))
            for pt in [gt1, gt2]:
                if pt is not None and not self._is_segment_blocked(pt, goal, circles, forbidden_zones, ignore_c1=c_idx):
                    nid = len(nodes)
                    nodes[nid] = pt
                    adj[nid] = []
                    ang = math.atan2(pt[1] - cy, pt[0] - cx)
                    circle_nodes[c_idx].append((nid, ang))

                    dist = math.hypot(goal[0] - pt[0], goal[1] - pt[1])
                    adj[nid].append((goal_id, dist, [goal]))

    def _add_circle_bitangents(
        self,
        circles: List[Tuple[float, float, float]],
        forbidden_zones: List[Dict[str, float]],
        nodes: Dict[int, Tuple[float, float]],
        adj: Dict[int, List[Tuple[int, float, List[Tuple[float, float]]]]],
        circle_nodes: Dict[int, List[Tuple[int, float]]]
    ):
        """Adiciona as 4 bitangentes analíticas entre cada par de círculos."""
        n = len(circles)
        for i in range(n):
            c1 = circles[i]
            for j in range(i + 1, n):
                c2 = circles[j]
                bitangents = self._circle_to_circle_tangents(c1, c2)
                for (p1, p2) in bitangents:
                    if not self._is_segment_blocked(p1, p2, circles, forbidden_zones, ignore_c1=i, ignore_c2=j):
                        id1 = len(nodes)
                        nodes[id1] = p1
                        adj[id1] = []
                        ang1 = math.atan2(p1[1] - c1[1], p1[0] - c1[0])
                        circle_nodes[i].append((id1, ang1))

                        id2 = len(nodes)
                        nodes[id2] = p2
                        adj[id2] = []
                        ang2 = math.atan2(p2[1] - c2[1], p2[0] - c2[0])
                        circle_nodes[j].append((id2, ang2))

                        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                        adj[id1].append((id2, dist, [p2]))
                        adj[id2].append((id1, dist, [p1]))

    def _add_perimeter_arcs(
        self,
        circles: List[Tuple[float, float, float]],
        forbidden_zones: List[Dict[str, float]],
        nodes: Dict[int, Tuple[float, float]],
        adj: Dict[int, List[Tuple[int, float, List[Tuple[float, float]]]]],
        circle_nodes: Dict[int, List[Tuple[int, float]]]
    ):
        """Conecta pontos de tangência no mesmo círculo por arcos perimétricos."""
        for c_idx, (cx, cy, r) in enumerate(circles):
            c_list = circle_nodes[c_idx]
            if len(c_list) < 2:
                continue

            # Ordena nós pelo ângulo azimutal na circunferência
            c_list.sort(key=lambda item: item[1])
            m = len(c_list)

            # Conecta nós vizinhos adjacentes no círculo
            for k in range(m):
                id_a, ang_a = c_list[k]
                id_b, ang_b = c_list[(k + 1) % m]

                # Sentido anti-horário
                diff = ang_b - ang_a
                if diff < 0:
                    diff += 2.0 * math.pi

                if diff < math.pi + 1e-4:
                    arc_pts, arc_len = self._interpolate_arc((cx, cy, r), ang_a, diff, 1.0)
                    if arc_pts and not self._is_arc_blocked(arc_pts, circles, forbidden_zones, ignore_circle=c_idx):
                        adj[id_a].append((id_b, arc_len, arc_pts))
                        adj[id_b].append((id_a, arc_len, list(reversed(arc_pts))))

    def _interpolate_arc(
        self,
        circle: Tuple[float, float, float],
        ang_start: float,
        diff: float,
        direction: float
    ) -> Tuple[Optional[List[Tuple[float, float]]], float]:
        """Interpola waypoints ao longo da borda circular."""
        cx, cy, r = circle
        arc_len = r * diff
        steps = max(1, int(diff / self.arc_step_rad))
        step_ang = (diff / steps) * direction

        pts = []
        for s in range(1, steps + 1):
            theta = ang_start + s * step_ang
            pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))

        return pts, arc_len

    def _is_arc_blocked(
        self,
        arc_pts: List[Tuple[float, float]],
        circles: List[Tuple[float, float, float]],
        forbidden_zones: List[Dict[str, float]],
        ignore_circle: int
    ) -> bool:
        """Verifica se os pontos do arco penetram outros obstáculos circulares."""
        for px, py in arc_pts:
            for c_idx, (cx, cy, r) in enumerate(circles):
                if c_idx == ignore_circle:
                    continue
                if math.hypot(px - cx, py - cy) < r - 1e-4:
                    return True

            for zone in forbidden_zones:
                if (zone['xmin'] <= px <= zone['xmax'] and
                        zone['ymin'] <= py <= zone['ymax']):
                    return True

        return False

    def _point_to_circle_tangents(
        self,
        p: Tuple[float, float],
        circle: Tuple[float, float, float]
    ) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
        """Calcula os dois pontos de tangência de um ponto externo para um círculo."""
        px, py = p
        cx, cy, r = circle
        dx = cx - px
        dy = cy - py
        d = math.hypot(dx, dy)

        if d <= r + 1e-4:
            return None, None

        theta = math.atan2(dy, dx)
        alpha = math.asin(r / d)

        ang1 = theta + math.pi / 2.0 + alpha
        ang2 = theta - math.pi / 2.0 - alpha

        t1 = (cx + r * math.cos(ang1), cy + r * math.sin(ang1))
        t2 = (cx + r * math.cos(ang2), cy + r * math.sin(ang2))
        return t1, t2

    def _circle_to_circle_tangents(
        self,
        c1: Tuple[float, float, float],
        c2: Tuple[float, float, float]
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Calcula as bitangentes externas e internas entre dois círculos."""
        x1, y1, r1 = c1
        x2, y2, r2 = c2
        dx = x2 - x1
        dy = y2 - y1
        d = math.hypot(dx, dy)

        tangents = []
        if d < 1e-4:
            return tangents

        theta = math.atan2(dy, dx)

        # 1. Tangentes Externas
        dr = r1 - r2
        if d > abs(dr):
            alpha = math.acos(dr / d)
            for sign in [1.0, -1.0]:
                ang = theta + sign * alpha
                p1 = (x1 + r1 * math.cos(ang), y1 + r1 * math.sin(ang))
                p2 = (x2 + r2 * math.cos(ang), y2 + r2 * math.sin(ang))
                tangents.append((p1, p2))

        # 2. Tangentes Internas (Cruzadas)
        sr = r1 + r2
        if d > sr + 1e-4:
            beta = math.acos(sr / d)
            for sign in [1.0, -1.0]:
                ang1 = theta + sign * beta
                ang2 = ang1 + math.pi
                p1 = (x1 + r1 * math.cos(ang1), y1 + r1 * math.sin(ang1))
                p2 = (x2 + r2 * math.cos(ang2), y2 + r2 * math.sin(ang2))
                tangents.append((p1, p2))

        return tangents

    def _astar_search(
        self,
        start_id: int,
        goal_id: int,
        nodes: Dict[int, Tuple[float, float]],
        adj: Dict[int, List[Tuple[int, float, List[Tuple[float, float]]]]]
    ) -> List[Tuple[float, float]]:
        """Busca A* padrão com heurística euclidiana."""
        start_pos = nodes[start_id]
        goal_pos = nodes[goal_id]

        h0 = math.hypot(goal_pos[0] - start_pos[0], goal_pos[1] - start_pos[1])
        heap = [(h0, 0.0, start_id, [start_pos])]
        g_scores = {start_id: 0.0}

        while heap:
            f, g, curr, path_so_far = heapq.heappop(heap)

            if curr == goal_id:
                return path_so_far

            if g > g_scores.get(curr, float('inf')):
                continue

            for neighbor_id, edge_cost, edge_waypoints in adj.get(curr, []):
                new_g = g + edge_cost

                if new_g < g_scores.get(neighbor_id, float('inf')):
                    g_scores[neighbor_id] = new_g
                    neigh_pos = nodes[neighbor_id]
                    h = math.hypot(goal_pos[0] - neigh_pos[0], goal_pos[1] - neigh_pos[1])
                    new_f = new_g + h
                    new_path = path_so_far + edge_waypoints
                    heapq.heappush(heap, (new_f, new_g, neighbor_id, new_path))

        return []

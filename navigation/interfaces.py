from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any, Optional

class IPathPlanner(ABC):
    """
    Interface abstrata para planejadores de caminho geométrico na RoboCup SSL.
    Recebe posições e obstáculos e retorna uma lista ordenada de waypoints livres de colisão.
    """

    @abstractmethod
    def plan(
        self,
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
        obstacles: List[Dict[str, Any]],
        forbidden_zones: List[Dict[str, float]]
    ) -> List[Tuple[float, float]]:
        """
        Calcula rota espacial livre de colisão entre start_pos e goal_pos.

        :param start_pos: Tupla (x, y) com a coordenada de partida no plano global.
        :param goal_pos: Tupla (x, y) com a coordenada de destino no plano global.
        :param obstacles: Lista de dicionários representando obstáculos circulares,
                          cada um contendo {'x': float, 'y': float, 'radius': float, ...}.
        :param forbidden_zones: Lista de dicionários definindo caixas retangulares proibidas,
                                cada um contendo {'xmin': float, 'xmax': float, 'ymin': float, 'ymax': float}.
        :return: Lista de waypoints [(x, y), ...] do início até a meta.
        """
        pass


class ITrajectoryGenerator(ABC):
    """
    Interface abstrata para geradores de trajetória temporal na RoboCup SSL.
    Converte um caminho geométrico (waypoints) em referências contínuas no tempo
    e gera comandos de velocidade locais (v_forward, v_left, vw) respeitando limites físicos.
    """

    @abstractmethod
    def compute_step(
        self,
        path: List[Tuple[float, float]],
        robot_pose: Tuple[float, float, float],
        target_yaw: Optional[float] = None,
        v0: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[float, float, float]:
        """
        Executa um passo do controlador de trajetória temporal.

        :param path: Sequência de waypoints [(x, y), ...] a seguir.
        :param robot_pose: Tupla (x, y, yaw) com a pose atual do robô no campo.
        :param target_yaw: Orientação angular final desejada (radianos). Se None, aponta para o caminho.
        :param v0: Vetor de velocidade linear estimada atual do robô (vx, vy) no referencial global.
        :return: Tupla (v_forward, v_left, vw) com velocidades no referencial local do robô.
        """
        pass

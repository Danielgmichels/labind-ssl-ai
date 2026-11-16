import os
import sys
import socket
from typing import List, Tuple, Optional

# Garante acesso à pasta proto_msg interna do projeto
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
PROTO_MSGS_DIR = os.path.join(PROJECT_ROOT, "proto_msg")

if PROTO_MSGS_DIR not in sys.path:
    sys.path.append(PROTO_MSGS_DIR)

try:
    import grSim_Packet_pb2
except ImportError as e:
    grSim_Packet_pb2 = None


def reset_field(
    ball: Tuple[float, float] = (0.0, 0.0),
    ball_vel: Tuple[float, float] = (0.0, 0.0),
    robots_yellow: Optional[List[Tuple[int, float, float, float]]] = None,
    robots_blue: Optional[List[Tuple[int, float, float, float]]] = None,
    ip: str = "127.0.0.1",
    port: int = 20011
) -> bool:
    """
    Reposiciona a bola e os robôs instantaneamente no grSim através do comando de Replacement (UDP).

    :param ball: Tupla (x, y) com a nova posição da bola no campo.
    :param ball_vel: Tupla (vx, vy) com a velocidade da bola.
    :param robots_yellow: Lista de tuplas (id, x, y, yaw) para os robôs amarelos.
    :param robots_blue: Lista de tuplas (id, x, y, yaw) para os robôs azuis.
    :param ip: Endereço IP do simulador grSim.
    :param port: Porta de controle de substituição do grSim (padrão: 20011).
    :return: True se o pacote foi montado e enviado com sucesso, False caso contrário.
    """
    if grSim_Packet_pb2 is None:
        print("[benchmarks/runner] Erro: grSim_Packet_pb2 não pôde ser importado.")
        return False

    if robots_yellow is None:
        robots_yellow = []
    if robots_blue is None:
        robots_blue = []

    packet = grSim_Packet_pb2.grSim_Packet()
    packet.replacement.ball.x = float(ball[0])
    packet.replacement.ball.y = float(ball[1])
    packet.replacement.ball.vx = float(ball_vel[0])
    packet.replacement.ball.vy = float(ball_vel[1])

    for r_id, x, y, yaw in robots_yellow:
        r = packet.replacement.robots.add()
        r.id = int(r_id)
        r.x = float(x)
        r.y = float(y)
        r.dir = float(yaw)
        r.yellowteam = True
        r.turnon = True

    for r_id, x, y, yaw in robots_blue:
        r = packet.replacement.robots.add()
        r.id = int(r_id)
        r.x = float(x)
        r.y = float(y)
        r.dir = float(yaw)
        r.yellowteam = False
        r.turnon = True

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(packet.SerializeToString(), (ip, port))
        sock.close()
        return True
    except Exception as ex:
        print(f"[benchmarks/runner] Falha no envio do socket UDP para {ip}:{port} - {ex}")
        return False


if __name__ == "__main__":
    print("Testando teletransporte inicial do grSim na porta 20011...")
    sucesso = reset_field(
        ball=(0.0, 0.0),
        robots_yellow=[(0, -3.5, 0.0, 0.0)],
        robots_blue=[(0, 0.0, 0.0, 3.14)]
    )
    if sucesso:
        print("Comando de reset enviado com sucesso!")
    else:
        print("Falha ao enviar comando de reset.")

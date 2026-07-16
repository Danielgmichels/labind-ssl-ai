import socket
import sys
import os

# Aponta para a pasta proto_msg onde compilamos os arquivos
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_MSGS_DIR = os.path.join(CURRENT_DIR, "proto_msg")
sys.path.append(PROTO_MSGS_DIR)

try:
    import grSim_Packet_pb2
except ImportError as e:
    print(f"Erro ao importar os protos do grSim. Tem certeza que compilou na pasta certa? Erro: {e}")
    sys.exit(1)

def teleporta_bola(x, y, vx=0.0, vy=0.0, ip="127.0.0.1", port=20011):
    """Teleporta a bola magicamente no grSim e zera a velocidade (congela ela no lugar)."""
    packet = grSim_Packet_pb2.grSim_Packet()
    
    # A estrutura de reposicionamento (Replacement)
    packet.replacement.ball.x = x
    packet.replacement.ball.y = y
    packet.replacement.ball.vx = vx
    packet.replacement.ball.vy = vy
    
    # Envia pela porta UDP de controle do simulador
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(packet.SerializeToString(), (ip, port))
    print(f"BOLA POSICIONADA -> X: {x} | Y: {y}")

if __name__ == "__main__":
    # Testa o teleporte (Coloca a bola no escanteio superior da nossa defesa)
    teleporta_bola(x=-6.0, y=4.5)
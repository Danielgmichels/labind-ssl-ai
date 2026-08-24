import socket
import struct
from state import ssl_gc_referee_message_pb2
# ==========================================
# MÓDULO 5: ÁRBITRO (Rede Multicast)
# ==========================================
class RefereeClient:
    def __init__(self, ip="224.5.23.1", port=10003):
        # Configuração de Socket Multicast (Assina o canal do Juiz)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', port))
        
        mreq = struct.pack("4sl", socket.inet_aton(ip), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        # Usa modo não-bloqueante para podermos esvaziar a fila como fizemos na Visão
        self.sock.setblocking(False)

    def get_latest_command(self):
        latest_data = None
        try:
            while True:
                latest_data = self.sock.recv(65535)
        except BlockingIOError:
            pass 
            
        if latest_data is not None:
            try:
                msg = ssl_gc_referee_message_pb2.Referee()
                msg.ParseFromString(latest_data)
                
                comando = ssl_gc_referee_message_pb2.Referee.Command.Name(msg.command)
                estagio = ssl_gc_referee_message_pb2.Referee.Stage.Name(msg.stage)
                
                # --- NOVO: Extrai a posição alvo do juiz ---
                pos_x, pos_y = None, None
                # Verifica se o juiz enviou uma coordenada neste pacote
                if msg.HasField("designated_position"):
                    # O Juiz da SSL envia as coordenadas em milímetros. 
                    # Dividimos por 1000 para converter para a matemática em metros do nosso APF!
                    pos_x = msg.designated_position.x / 1000.0
                    pos_y = msg.designated_position.y / 1000.0
                    
                # Retorna os 4 valores agora
                return comando, estagio, pos_x, pos_y
            except Exception as e:
                print(f"Erro ao processar pacote do juiz: {e}")
        return None, None, None, None
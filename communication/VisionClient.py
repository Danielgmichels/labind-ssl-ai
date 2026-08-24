import zmq
import State_pb2# ==========================================
# MÓDULO 1: VISÃO (Rede ZMQ) - CORRIGIDO
# ==========================================
class VisionClient:
    def __init__(self, ip="127.0.0.1", port=5558):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        
        # A SOLUÇÃO DO CONGELAMENTO: 
        # CONFLATE garante que o socket descarte frames antigos e 
        # mantenha estritamente o pacote mais novo na memória.
        self.socket.setsockopt(zmq.CONFLATE, 1) 
        
        self.socket.connect(f"tcp://{ip}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

    def get_latest_state(self):
        latest_data = None
        try:
            # Com o CONFLATE ativado, drenamos a fila instantaneamente
            while True:
                latest_data = self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            pass 
            
        if latest_data is not None:
            try:
                state = State_pb2.State()
                state.ParseFromString(latest_data)
                return state
            except Exception as e:
                print(f"Erro no Parse da Visão: {e}")
        return None
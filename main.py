import sys
import time
import math
import socket
import zmq
import os
import struct
import argparse
from behavior_tree import *

# Descobre onde o main.py está e aponta para a pasta proto_msg interna
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_MSGS_DIR = os.path.join(CURRENT_DIR, "proto_msg")

sys.path.append(CURRENT_DIR)
sys.path.append(PROTO_MSGS_DIR) # <-- O Python agora só olha para dentro do próprio projeto!

try:
    import State_pb2
    import ssl_simulation_robot_control_pb2 
    from state import ssl_gc_referee_message_pb2
    
except ImportError as e:
    print(f"Erro crítico na importação dos Protobufs: {e}")
    sys.exit(1)


# ==========================================
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
    

# ==========================================
# MÓDULO 2: CONTROLE (O Cérebro / APF Avançado)
# ==========================================
class ProportionalController:
    def __init__(self, kp_linear=2.0, kp_angular=2.0, max_vel=2.5, max_angular_vel=5.0):
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_vel = max_vel
        self.max_angular_vel = max_angular_vel
        
        # Valores drasticamente reduzidos para suportar o campo lotado (12 robôs)
        self.dist_segura = 0.12  # Raio do robô + folga mínima
        self.kr_repulsao = 0.005  # Repulsão base muito mais suave

    def calculate_velocity(self, robot_x, robot_y, robot_yaw, ball_x, ball_y, obstacles):
        # 1. FORÇA DE ATRAÇÃO
        attr_x = ball_x - robot_x  
        attr_y = ball_y - robot_y  
        
        dist_bola = math.hypot(attr_x, attr_y)
        
        vx_global = attr_x * self.kp_linear
        vy_global = attr_y * self.kp_linear
        
        # O Limitador da Atração (Anti-trator)
        mag_attr = math.hypot(vx_global, vy_global)
        if mag_attr > self.max_vel:
            vx_global = (vx_global / mag_attr) * self.max_vel
            vy_global = (vy_global / mag_attr) * self.max_vel
            mag_attr = self.max_vel 

        # ==========================================
        # A MÁGICA DA VELOCIDADE (Bolha Dinâmica - COMPACTA)
        # ==========================================
        fator_velocidade = mag_attr / self.max_vel 
        
        # Aumenta no máximo +15cm de segurança quando estiver rápido (Total 30cm)
        dist_segura_dinamica = self.dist_segura + (fator_velocidade * 0.15)
        
        # Aumenta a força repulsiva de forma sutil
        kr_dinamico = self.kr_repulsao + (fator_velocidade * 0.08)
        
        # 2. FORÇA DE REPULSÃO (Com correção GNRON e Paredes Sólidas)
        rep_x = 0.0
        rep_y = 0.0
        
        fator_foco = min(dist_bola / dist_segura_dinamica, 1.0)
        
        for obs in obstacles:
            dx = robot_x - obs.pos.x  
            dy = robot_y - obs.pos.y
            dist_obs = math.hypot(dx, dy)
            
            # A MÁGICA: É uma parede virtual ou um robô físico?
            is_wall = not hasattr(obs, 'id')
            
            if is_wall:
                # PAREDE DE CONCRETO: Empurra muito forte e de mais longe
                dist_segura_atual = 0.90  # Sente a parede a 40cm de distância
                kr_atual = 3.0            # Força brutal (ignora a atração da bola)
                fator_foco_atual = 1.0    # Sempre empurra com 100% de prioridade
            else:
                # ROBÔ DE ESPUMA: Repulsão suave configurada anteriormente
                dist_segura_atual = dist_segura_dinamica
                kr_atual = kr_dinamico
                fator_foco_atual = fator_foco
            
            # Aplica a força dependendo do tipo de obstáculo
            if 0.01 < dist_obs < dist_segura_atual:
                dist_calc = max(dist_obs, 0.15) 
                
                forca = (kr_atual / (dist_calc ** 2)) * fator_foco_atual
                
                rep_x += (dx / dist_obs) * forca
                rep_y += (dy / dist_obs) * forca
                
        # 3. SOMA VETORIAL GLOBAL
        vx_global += rep_x
        vy_global += rep_y

        # 4. ROTAÇÃO
        target_angle = math.atan2(attr_y, attr_x)
        erro_angular = target_angle - robot_yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        if abs(erro_angular) < 0.05:
            vw = 0.0
        else:
            vw = erro_angular * self.kp_angular
            
        if vw > self.max_angular_vel: vw = self.max_angular_vel
        elif vw < -self.max_angular_vel: vw = -self.max_angular_vel
        
        # 5. TRANSLAÇÃO LOCAL (Matriz de Rotação)
        v_forward = vx_global * math.cos(robot_yaw) + vy_global * math.sin(robot_yaw)
        v_left = -vx_global * math.sin(robot_yaw) + vy_global * math.cos(robot_yaw)
        
        magnitude = math.sqrt(v_forward**2 + v_left**2)
        if magnitude > self.max_vel:
            v_forward = (v_forward / magnitude) * self.max_vel
            v_left = (v_left / magnitude) * self.max_vel
            
        return v_forward, v_left, vw
        
# ==========================================
# MÓDULO 3: AÇÃO (Rede UDP) - LOCAL VELOCITY
# ==========================================
class ActionClient:
    def __init__(self, ip="127.0.0.1", port=10302):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Atualizamos kick_z_speed para kick_angle, seguindo o protocolo oficial da SSL
    def send_command(self, robot_id, v_forward, v_left, vw, kick_speed=0.0, kick_angle=0.0, dribbler_speed=0.0):
        packet = ssl_simulation_robot_control_pb2.RobotControl()
        command = packet.robot_commands.add()
        command.id = robot_id
        
        command.move_command.local_velocity.forward = v_forward
        command.move_command.local_velocity.left = v_left
        command.move_command.local_velocity.angular = vw
        
        # O Chute (Força em m/s)
        command.kick_speed = kick_speed
        
        # O Ângulo do chute (0 para rasteiro, >0 para cavadinha/cruzamento)
        command.kick_angle = kick_angle
        
        # O Driblador (Rolo giratório que gruda a bola no bico do robô)
        command.dribbler_speed = dribbler_speed

        data = packet.SerializeToString()
        self.sock.sendto(data, (self.ip, self.port))

class VirtualObstacle:
    def __init__(self, x, y):
        # Cria uma estrutura "falsa" imitando o protobuf do robô para o APF conseguir ler
        self.pos = type('Pos', (), {'x': x, 'y': y})()

def create_solid_defense_walls():
    """
    Cria uma barreira física de pontos ao redor das 3 linhas expostas das áreas.
    Adaptado para as coordenadas do campo longo (X de -6.0 a 6.0).
    """
    walls = []
    step = 0.15 # Um ponto a cada 15cm cria um muro impenetrável para o APF
    
    # --- ÁREA ESQUERDA (Nossa: X de -6.0 até -5.0) ---
    # Linha Frontal (X = -5.0, Y de -1.0 a 1.0)
    y = -1.5
    while y <= 1.5:
        walls.append(VirtualObstacle(-5.0, y))
        y += step
        
    # Linhas Laterais (Cima e Baixo)
    x = -6.0
    while x <= -4.5: # Vai do fundo (-6.0) até a linha frontal (-5.0)
        walls.append(VirtualObstacle(x, 1.0))  # Parede do Topo
        walls.append(VirtualObstacle(x, -1.0)) # Parede do Fundo
        x += step

    # --- ÁREA DIREITA (Inimiga: X de 5.0 até 6.0) ---
    # Linha Frontal (X = 5.0, Y de -1.0 a 1.0)
    y = -1.5
    while y <= 1.5:
        walls.append(VirtualObstacle(5.0, y))
        y += step
        
    # Linhas Laterais (Cima e Baixo)
    x = 4.5
    while x <= 6.0: # Vai da linha frontal (5.0) até o fundo (6.0)
        walls.append(VirtualObstacle(x, 1.0))  # Parede do Topo
        walls.append(VirtualObstacle(x, -1.0)) # Parede do Fundo
        x += step
        
    return walls

# ==========================================
# MÓDULO 4: ESTADO GLOBAL (O "Quadro Negro")
# ==========================================
class Blackboard:
    # Agora ele recebe o controller e a action na criação
    def __init__(self, controller, action): 
        self.controller = controller
        self.action = action
        
        self.referee_command = "HALT" 
        self.referee_stage = "NORMAL_FIRST_HALF_PRE"
        self.is_yellow = True
        self.my_id = 1
        self.my_role = "ATACANTE" 
        
        self.my_pos = None    
        self.ball_pos = None  
        self.obstacles = []   
        
        # O Muro Físico das áreas gerado apenas uma vez
        self.defense_walls = create_solid_defense_walls() 


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
            # Esvazia a fila para ter a certeza que ouvimos o apito mais recente
            while True:
                latest_data = self.sock.recv(65535)
        except BlockingIOError:
            pass # A fila secou
            
        if latest_data is not None:
            try:
                # Atualizado aqui também
                msg = ssl_gc_referee_message_pb2.Referee()
                msg.ParseFromString(latest_data)
                
                # Retornamos os nomes em texto puro (Ex: "HALT", "NORMAL_START")
                comando = ssl_gc_referee_message_pb2.Referee.Command.Name(msg.command)
                estagio = ssl_gc_referee_message_pb2.Referee.Stage.Name(msg.stage)
                return comando, estagio
            except Exception as e:
                print(f"Erro ao processar pacote do juiz: {e}")
        return None, None


def maestro_distribui_papeis(team_robots, ball_pos, enemy_goal_x, last_roles=None, id_goleiro=0):
    if last_roles is None: last_roles = {}
    papeis = {}
    if not team_robots: return papeis
    
    papeis[id_goleiro] = "GOLEIRO"
    
    if ball_pos is None:
        for r in team_robots:
            r_id = getattr(r, 'id', 0)
            if r_id != id_goleiro: papeis[r_id] = "ESPERA"
        return papeis

    # 1. Acha o Atacante (O robô mais perto da bola)
    min_dist_bola = float('inf')
    id_atacante = None
    for r in team_robots:
        r_id = getattr(r, 'id', 0)
        if r_id == id_goleiro or (abs(r.pos.x) < 0.001 and abs(r.pos.y) < 0.001): continue
        
        dist_bola = math.hypot(r.pos.x - ball_pos.x, r.pos.y - ball_pos.y)
        
        # CORREÇÃO 1: Histerese do Atacante (Evita que fiquem brigando pela bola)
        if last_roles.get(r_id, "") == "ATACANTE":
            dist_bola -= 0.5 # Bônus: Finge estar 50cm mais perto para segurar a vaga
            
        if dist_bola < min_dist_bola:
            min_dist_bola = dist_bola
            id_atacante = r_id
            
    if id_atacante is not None:
        papeis[id_atacante] = "ATACANTE"

    # 2. Define o Apoio e a Zaga
    sobra = []
    for r in team_robots:
        r_id = getattr(r, 'id', 0)
        if r_id not in papeis and (abs(r.pos.x) >= 0.001 or abs(r.pos.y) >= 0.001):
            dist_gol_inimigo = math.hypot(r.pos.x - enemy_goal_x, r.pos.y - 0.0)
            
            # CORREÇÃO 2: Histerese correta para a defesa não "pular"
            papel_antigo = last_roles.get(r_id, "")
            if papel_antigo == "ATACANTE_APOIO":
                dist_gol_inimigo -= 2.0 
            elif papel_antigo == "ZAGUEIRO_BLOQUEIO":
                dist_gol_inimigo += 2.0 
            elif papel_antigo == "ZAGUEIRO_MARCACAO":
                dist_gol_inimigo += 1.0 
                
            sobra.append((dist_gol_inimigo, r_id))
            
    # Ordena a lista: [Mais Perto do Inimigo ... Mais Perto do Nosso Gol]
    sobra.sort(key=lambda x: x[0])
    
    # CORREÇÃO 3: Distribuição limpa usando pop()
    if len(sobra) > 0:
        # Pega o primeiro da lista (Mais Avançado)
        papeis[sobra.pop(0)[1]] = "ATACANTE_APOIO" 
        
    if len(sobra) > 0:
        # Pega o ÚLTIMO da lista (Mais Recuado)
        papeis[sobra.pop(-1)[1]] = "ZAGUEIRO_BLOQUEIO" 
        
    if len(sobra) > 0:
        # Pega o novo ÚLTIMO da lista
        papeis[sobra.pop(-1)[1]] = "ZAGUEIRO_MARCACAO" 
        
    # Quem restou no meio-campo fica na reserva
    for s in sobra:
        papeis[s[1]] = "ESPERA"
        
    return papeis

def build_attacker_tree():
    """Constrói a Behavior Tree do Atacante e retorna o nó raiz."""
    
    ramo_emergencia = Sequence([ConditionIsHalted(), ActionStopMotors()])
    ramo_kickoff = Sequence([ConditionIsPrepareKickoff(), ActionPrepareKickoff()])
    
    # 3.1. Finalizar (O Radar encontrou uma fresta no gol)
    tentar_finalizar = Sequence([
        ConditionIsNearBall(),
        ConditionIsInShootingZone(),
        ConditionIsPathClear(), 
        ActionAimAndShoot()
    ])
    
    # 3.2. NOVO: O Passe! (Gol bloqueado, mas tem parceiro limpo)
    tentar_passe = Sequence([
        ConditionIsNearBall(),
        ConditionIsPassClear(), # Verifica companheiro e linha limpa
        ActionPassBall()
    ])
    
    # 3.3. Achar Ângulo (Tudo bloqueado. Anda de lado!)
    achar_angulo = Sequence([
        ConditionIsNearBall(),
        ConditionIsInShootingZone(),
        ActionFindShootingAngle()
    ])
    
    # 3.4. Conduzir 
    tentar_conduzir = Sequence([
        ConditionIsNearBall(),
        ActionDribbleToGoal()
    ])
    
    # 3.5. Buscar 
    buscar_bola = ActionGoToBall()
    
    # Agrupa todo o comportamento ofensivo (A Ordem Importa Muito!)
    ramo_ofensivo = Sequence([
        ConditionIsGameRunning(),
        Selector([
            tentar_finalizar,
            tentar_passe,   # Tenta o passe antes de decidir andar de lado!
            achar_angulo,   
            tentar_conduzir,
            buscar_bola
        ])
    ])
    
    root = Selector([ramo_emergencia, ramo_kickoff, ramo_ofensivo])
    return root

def build_atacante_apoio_tree():
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        Sequence([ConditionIsGameRunning(), ActionPositionForPass()]),
        ActionStopMotors() # Fallback: Se o jogo não tá valendo, FREIA O ROBÔ!
    ])

def build_zaga_bloqueio_tree():
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        Sequence([ConditionIsGameRunning(), ActionZagueiroBloqueio()]),
        ActionStopMotors() 
    ])

def build_zaga_marcacao_tree():
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        Sequence([ConditionIsGameRunning(), ActionZagueiroMarcacao()]),
        ActionStopMotors() 
    ])

def build_goleiro_tree():
    """Constrói a Behavior Tree do Goleiro."""
    
    # 1. Ramo de Emergência (Juiz apitou HALT)
    ramo_emergencia = Sequence([
        ConditionIsHalted(),
        ActionStopMotors()
    ])
    
    # 2. Ramo de Limpar a Área
    limpar_area = Sequence([
        ConditionIsBallSafeToClear(),
        ActionClearBall()
    ])
    
    # 3. Ramo de Defesa (Fallback natural)
    defender = ActionDefendGoal()
    
    # Raiz do Goleiro
    root = Selector([
        ramo_emergencia,
        limpar_area,
        defender
    ])
    
    return root

def build_espera_tree():
    """Árvore genérica para robôs ociosos: apenas desliga os motores."""
    return ActionStopMotors()

# ==========================================
# LOOP PRINCIPAL (Integração)
# ==========================================

def main():
    print("Iniciando Motor Tático da Behavior Tree...")
    
    # 1. Lê os argumentos do Docker para saber qual time somos
    parser = argparse.ArgumentParser()
    parser.add_argument('--team', default='yellow', choices=['yellow', 'blue'])
    args = parser.parse_args()
    
    is_yellow = (args.team == 'yellow')
    
    # A porta padrão do grSim: 10302 para Amarelo, 10301 para Azul
    porta_comando = 10302 if is_yellow else 10301
    
    # 2. Inicializa os módulos de infraestrutura
    vision = VisionClient(port=5558) 
    referee = RefereeClient(ip="224.5.23.1", port=10003)
    # Voltando o kp_angular para 2.0 (positivo)
    controller = ProportionalController(kp_linear=2.0, kp_angular=2.0, max_vel=2.5, max_angular_vel=5.0)
    action = ActionClient(port=porta_comando)
    
    # 3. Inicializa o Blackboard
    bb = Blackboard(controller, action)
    bb.is_yellow = is_yellow
    
    # === A MÁGICA DOS LADOS ===
    # Amarelo defende o gol positivo (6.0) e ataca o negativo (-6.0)
    if is_yellow:
        bb.our_goal_x = 6.0
        bb.enemy_goal_x = -6.0
    else:
        bb.our_goal_x = -6.0
        bb.enemy_goal_x = 6.0
        
    
    # 3. Constrói o Cérebro do Atacante
    arvore_atacante = build_attacker_tree()
    arvore_goleiro = build_goleiro_tree()
    arvore_espera = build_espera_tree()
    arvore_zaga_bloqueio = build_zaga_bloqueio_tree()
    arvore_zaga_marcacao = build_zaga_marcacao_tree()
    arvore_atacante_apoio = build_atacante_apoio_tree()

    cycle_time = 1.0 / 60 
    
    arvore_zaga_marcacao = build_zaga_marcacao_tree()
    arvore_atacante_apoio = build_atacante_apoio_tree()

    cycle_time = 1.0 / 60 
    
    # --- NOVO: Timer para o Debug ---
    last_debug_time = time.time()
    
    while True:
        start_time = time.time()
        
        # ==========================================
        # FASE 1: PERCEÇÃO (Atualizar o Blackboard)
        # ==========================================
        cmd, stage = referee.get_latest_command()
        if cmd is not None:
            if bb.referee_command != cmd: 
                print(f"JUIZ APITOU: {cmd} (Fase: {stage})")
            bb.referee_command = cmd
            bb.referee_stage = stage
            
        state = vision.get_latest_state()
        
        # SE TEMOS VISÃO, PODEMOS AGIR!
        if state is not None:
            world = state.last_seen_world
            if world.ball.visible:
                bb.ball_pos = world.ball.pos
                
            team_robots = world.yellow if bb.is_yellow else world.blue
            bb.my_pos = next((r for r in team_robots if getattr(r, 'id', 0) == bb.my_id and (abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001)), None)

            # ==========================================
            # FASE 2: O MAESTRO (Distribuição Tática)
            # ==========================================
            team_robots = world.yellow if bb.is_yellow else world.blue
            
            # Resgata a memória do frame anterior (se existir)
            papeis_anteriores = getattr(bb, 'papeis', {})
            
            # Passa a memória para o Maestro
            papeis_do_time = maestro_distribui_papeis(team_robots, bb.ball_pos, bb.enemy_goal_x, papeis_anteriores, id_goleiro=0)
            
            bb.papeis = papeis_do_time
            bb.team = team_robots

            # ==========================================
            # FASE 3: COMPORTAMENTO E AÇÃO (Multi-Agente)
            # ==========================================
            for robo in team_robots:
                robo_id = getattr(robo, 'id', 0)
                
                # Ignora fantasmas da visão
                if abs(robo.pos.x) < 0.001 and abs(robo.pos.y) < 0.001:
                    continue
                    
                # Verifica se o Maestro deu um papel para este robô
                if robo_id in papeis_do_time:
                    # 1. Troca a "lente" do Blackboard para a perspectiva deste robô
                    bb.my_id = robo_id
                    bb.my_pos = robo
                    bb.my_role = papeis_do_time[robo_id]
                    
                    # CORREÇÃO AQUI: A lista de inimigos para a zaga
                    bb.enemies = world.blue if bb.is_yellow else world.yellow

                    # --- NOVO: CONSTRÓI OS OBSTÁCULOS PARA ESTE ROBÔ ---
                    bb.obstacles = []
                    for r in world.yellow:
                        if getattr(r, 'id', 0) != bb.my_id or not bb.is_yellow:
                            if abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001: bb.obstacles.append(r)
                    for r in world.blue:
                        if getattr(r, 'id', 0) != bb.my_id or bb.is_yellow:
                            if abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001: bb.obstacles.append(r)
                    
                    if bb.my_role != "GOLEIRO":
                        bb.obstacles.extend(bb.defense_walls)
                    # ---------------------------------------------------
                    
                    # 2. Aciona o Cérebro correto dependendo do papel
                    if bb.my_role == "ATACANTE":
                        arvore_atacante.tick(bb)

                    elif bb.my_role == "ATACANTE_APOIO":
                        arvore_atacante_apoio.tick(bb)


                    elif bb.my_role == "ZAGUEIRO_BLOQUEIO":
                        arvore_zaga_bloqueio.tick(bb) # Não esqueça de declarar essa árvore no main()!
                    
                    elif bb.my_role == "ZAGUEIRO_MARCACAO":
                        arvore_zaga_marcacao.tick(bb)
    
                    elif bb.my_role == "ESPERA":
                        arvore_espera.tick(bb)
                        
                    elif bb.my_role == "GOLEIRO":
                        # Por enquanto, deixa parado até fazermos a árvore dele
                        arvore_goleiro.tick(bb)

        # ==========================================
        # PAINEL DE DEBUG (Imprime a cada 1 segundo)
        # ==========================================
        if time.time() - last_debug_time > 1.0:
            time_nome = "AMARELO" if bb.is_yellow else "AZUL"
            print(f"[{time_nome}] Estado do Juiz: {bb.referee_command}")
            for r_id, papel in bb.papeis.items():
                print(f"  ID {r_id}: {papel}")
            print("-" * 30)
            last_debug_time = time.time()
            
        # ==========================================
        # CONTROLE DE FPS (60Hz)
        # ==========================================
        # O Controle de FPS fica FORA do if, para o loop rodar a 60Hz perfeitamente
        elapsed_time = time.time() - start_time
        sleep_time = cycle_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()
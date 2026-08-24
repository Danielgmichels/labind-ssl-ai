import sys
import time
import math
import socket
import zmq
import os
import struct
import argparse

# 1. PRIMEIRO: Descobre onde o main.py está e aponta para a pasta proto_msg interna
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_MSGS_DIR = os.path.join(CURRENT_DIR, "proto_msg")

sys.path.append(CURRENT_DIR)
sys.path.append(PROTO_MSGS_DIR) # Agora o Python já sabe ler os protobufs!

# 2. SEGUNDO: Fazemos os imports dos nossos módulos e protobufs
from behavior_tree import *
from communication.VisionClient import VisionClient
from communication.RefereeClient import RefereeClient
from communication.ActionClient import ActionClient
from navigation.APF import ProportionalController
from world.Blackboard import Blackboard
from strategy.Maestro import maestro_distribui_papeis
try:
    import State_pb2
    import ssl_simulation_robot_control_pb2 
    from state import ssl_gc_referee_message_pb2
    import grSim_Packet_pb2
except ImportError as e:
    print(f"Erro crítico na importação dos Protobufs: {e}")
    sys.exit(1)



def teleporta_bola_simulador(x, y, ip="127.0.0.1", port=20011):
    """Função global para teleportar a bola rapidamente no grSim."""
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.replacement.ball.x = x
    packet.replacement.ball.y = y
    packet.replacement.ball.vx = 0.0
    packet.replacement.ball.vy = 0.0
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(packet.SerializeToString(), (ip, port))

def build_attacker_tree():
    ramo_emergencia = Sequence([ConditionIsHalted(), ActionStopMotors()])
    
    # O Atacante é o único com comportamento diferente
    ramo_kickoff_nosso = Sequence([ConditionIsOurPrepareKickoff(), ActionPrepareKickoff()])
    # Na saída do inimigo, fica a 80cm da bola, perfeitamente fora do círculo central de 50cm!
    ramo_kickoff_deles = Sequence([ConditionIsEnemyPrepareKickoff(), ActionPositionForKickoff(0.8, 0.0)])

    cobrar_falta = Sequence([ # (Seu bloco cobrar_falta antigo continua igualzinho aqui) ...
        ConditionIsOurFreeKick(),
        Selector([
            Sequence([ConditionIsNearBall(), ConditionIsPassClear(), ActionPassBall()]),
            Sequence([ConditionIsNearBall(), ActionCrossToBox()]), 
            ActionGoToBall() 
        ])
    ])
    
    receber_passe = Sequence([ConditionIsPassArriving(), ActionInterceptPass()])
    tentar_finalizar = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ConditionIsPathClear(), ActionAimAndShoot()])
    tentar_passe = Sequence([ConditionIsNearBall(), ConditionIsPassClear(), ActionPassBall()])
    achar_angulo = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ActionFindShootingAngle()])
    tentar_conduzir = Sequence([ConditionIsNearBall(), ActionSmartDribble()])
    buscar_bola = ActionGoToBall()
    
    ramo_ofensivo = Sequence([
        Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()]),
        Selector([receber_passe, tentar_finalizar, tentar_passe, achar_angulo, tentar_conduzir, buscar_bola])
    ])

    return Selector([ramo_emergencia, ramo_kickoff_nosso, ramo_kickoff_deles, Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(6)]), cobrar_falta, ramo_ofensivo])

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

# Agora o Apoio recebe o lado, o índice de marcação (falta inimiga) e o ângulo (nossa falta)
def build_atacante_apoio_tree(lado_y, indice_marcacao, angulo_falta):
    receber_passe = Sequence([ConditionIsPassArriving(), ActionInterceptPass()])
    ramo_ofensivo = Sequence([ConditionIsGameRunning(), Selector([receber_passe, ActionPositionForPass(lado_y)])])
    
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Espaçados 1.5 metros para trás, abertos nas alas
        Sequence([qualquer_kickoff, ActionPositionForKickoff(1.5, lado_y)]), 
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(indice_marcacao)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForShortPass(angulo_falta, 1.5)]),
        ramo_ofensivo,
        ActionStopMotors() 
    ])

def build_zaga_bloqueio_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Zaga se abre na frente da área (Y=1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(5.0, 1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.0)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroBloqueio()]), 
        ActionStopMotors() 
    ])

def build_zaga_marcacao_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Zaga se abre na frente da área (Y=-1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(5.0, -1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.18)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroMarcacao()]),
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

def build_meio_campo_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Fixado no centro a 3.5 metros
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(0)]),
        Sequence([valendo_ou_nossa_falta, ActionMidfieldSupport()]), 
        ActionStopMotors() 
    ])

def build_meia_armador_tree():
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]), 
        # NOVO: Centro-ofensivo recuado 2.5m
        Sequence([qualquer_kickoff, ActionPositionForKickoff(2.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(3)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForShortPass(-45.0, 1.5)]),
        Sequence([ConditionIsGameRunning(), ActionMeiaArmador()]), 
        ActionStopMotors()
    ])

def build_volante_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Cão de guarda fica a 4.5 metros (quase na área)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(4.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=-0.18)]),
        Sequence([valendo_ou_nossa_falta, ActionVolanteDefensivo()]),
        ActionStopMotors() 
    ])

# O Y vai ser 3.5 (esquerda) e -3.5 (direita) para correr perto da borda
def build_lateral_tree(lado_y, indice_marcacao):
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]), 
        # NOVO: Bem abertos nas laterais a 3 metros
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.0, lado_y)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(indice_marcacao)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForCross(lado_y)]),
        Sequence([ConditionIsGameRunning(), ActionLateral(lado_y)]), 
        ActionStopMotors()
    ])

def build_espera_tree():
    """Árvore genérica para robôs ociosos: apenas desliga os motores."""
    return ActionStopMotors()

def build_master_tree():
    """Cria uma Árvore de Comportamento Mestra que engloba todo o time."""
    return Selector([
        Sequence([ConditionCheckRole("GOLEIRO"), build_goleiro_tree()]),
        Sequence([ConditionCheckRole("ATACANTE"), build_attacker_tree()]),
        Sequence([ConditionCheckRole("ATACANTE_APOIO_ESQ"), build_atacante_apoio_tree(2.5, 4, 45.0)]),
        Sequence([ConditionCheckRole("ATACANTE_APOIO_DIR"), build_atacante_apoio_tree(-2.5, 5, -45.0)]),
        Sequence([ConditionCheckRole("MEIA_ARMADOR"), build_meia_armador_tree()]),
        Sequence([ConditionCheckRole("MEIO_CAMPO"), build_meio_campo_tree()]),
        Sequence([ConditionCheckRole("VOLANTE"), build_volante_tree()]),
        Sequence([ConditionCheckRole("LATERAL_ESQUERDO"), build_lateral_tree(3.5, 1)]),
        Sequence([ConditionCheckRole("LATERAL_DIREITO"), build_lateral_tree(-3.5, 2)]),
        Sequence([ConditionCheckRole("ZAGUEIRO_BLOQUEIO"), build_zaga_bloqueio_tree()]),
        Sequence([ConditionCheckRole("ZAGUEIRO_MARCACAO"), build_zaga_marcacao_tree()]),
        Sequence([ConditionCheckRole("ESPERA"), build_espera_tree()])
    ])

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
        
    cycle_time = 1.0 / 60 # Define o tempo de ciclo para 60Hz
    
    # --- NOVO: Timer para o Debug ---
    last_debug_time = time.time()

    # Gera os diagramas visuais!
    arvore_mestra = build_master_tree()
    export_tree_to_xml(arvore_mestra, "diagramas/arvore_mestra.xml")
    
    while True:
        start_time = time.time()
        
        # ==========================================
        # FASE 1: PERCEÇÃO (Atualizar o Blackboard)
        # ==========================================
        cmd, stage, desig_x, desig_y = referee.get_latest_command()
        
        if cmd is not None:
            if bb.referee_command != cmd: 
                print(f"JUIZ APITOU: {cmd} (Fase: {stage})")
                bb.referee_command = cmd
                bb.referee_stage = stage
                
                # --- NOVO: Lógica da Bola em Jogo Forçada ---
                bb.bola_em_jogo_forcada = False
                if getattr(bb, 'ball_pos', None) is not None:
                    # Grava exatamente onde a bola estava quando o juiz apitou
                    bb.posicao_bola_no_apito = (bb.ball_pos.x, bb.ball_pos.y)
                else:
                    bb.posicao_bola_no_apito = None
                # --------------------------------------------
            
            if desig_x is not None and desig_y is not None:
                nova_posicao = (desig_x, desig_y)
                posicao_anterior = getattr(bb, 'designated_position', None)
                if posicao_anterior != nova_posicao:
                    print(f"⚽ Sumatra/AutoRef: Reposicionando a bola para X:{desig_x:.2f}, Y:{desig_y:.2f}")
                    teleporta_bola_simulador(desig_x, desig_y)
                    bb.designated_position = nova_posicao
            
        state = vision.get_latest_state()
        
        # SE TEMOS VISÃO, PODEMOS AGIR!
        if state is not None:
            world = state.last_seen_world
            if world.ball.visible:
                bb.ball_pos = world.ball.pos
                
                # --- Calcula a Velocidade Vetorial da Bola com FILTRO EMA ---
                current_time = time.time()
                dt = current_time - bb.last_ball_time
                if bb.last_ball_pos is not None and dt > 0:
                    vx_raw = (bb.ball_pos.x - bb.last_ball_pos.x) / dt
                    vy_raw = (bb.ball_pos.y - bb.last_ball_pos.y) / dt
                    
                    alpha = 0.3 
                    bb.ball_vel_x = (alpha * vx_raw) + ((1.0 - alpha) * bb.ball_vel_x)
                    bb.ball_vel_y = (alpha * vy_raw) + ((1.0 - alpha) * bb.ball_vel_y)
                
                bb.last_ball_pos = bb.ball_pos
                bb.last_ball_time = current_time
                
                # --- NOVO: DETECTOR DE BOLA EM JOGO (O Hack do Juiz) ---
                is_free_kick = bb.referee_command in [
                    "DIRECT_FREE_YELLOW", "INDIRECT_FREE_YELLOW", 
                    "DIRECT_FREE_BLUE", "INDIRECT_FREE_BLUE"
                ]
                
                if is_free_kick and not getattr(bb, 'bola_em_jogo_forcada', False):
                    speed = math.hypot(bb.ball_vel_x, bb.ball_vel_y)
                    dist_movida = 0.0
                    if getattr(bb, 'posicao_bola_no_apito', None) is not None:
                        dist_movida = math.hypot(bb.ball_pos.x - bb.posicao_bola_no_apito[0], 
                                                 bb.ball_pos.y - bb.posicao_bola_no_apito[1])
                    
                    # A bola voou mais rápido que 0.8 m/s OU andou mais que 15 cm da marca da falta?
                    if speed > 1.0 or dist_movida > 1.5:
                        bb.bola_em_jogo_forcada = True
                        print("⚡ I.A.: A bola andou! Forçando o NORMAL_START e desfazendo a formação!")
                # -------------------------------------------------------
                
            team_robots = world.yellow if bb.is_yellow else world.blue
            bb.my_pos = next((r for r in team_robots if getattr(r, 'id', 0) == bb.my_id and (abs(r.pos.x) > 0.001 or abs(r.pos.y) > 0.001)), None)

            # ==========================================
            # FASE 2: O MAESTRO (Distribuição Tática)
            # ==========================================
            
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
                    arvore_mestra.tick(bb)
                        
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
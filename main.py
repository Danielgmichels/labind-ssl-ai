import sys
import time
import math
import socket
import zmq
import os
import struct
import argparse

# 1. Descobre onde o main.py está e aponta para a pasta proto_msg interna
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_MSGS_DIR = os.path.join(CURRENT_DIR, "proto_msg")

sys.path.append(CURRENT_DIR)
sys.path.append(PROTO_MSGS_DIR) 

# 2. Imports dos módulos e protobufs
from behavior_tree.core import export_tree_to_xml
from behavior_tree.trees.master import build_master_tree
from communication.VisionClient import VisionClient
from communication.RefereeClient import RefereeClient 
from communication.ActionClient import ActionClient
from navigation.APF import ProportionalController
from strategy.Maestro import maestro_distribui_papeis

# Novos imports da arquitetura
from world.Blackboard import Blackboard
from world.world_model import WorldModel

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


# Pequeno mock para garantir retrocompatibilidade com a BT
class PosMock:
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.orientation = yaw
        self.theta = yaw


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
    controller = ProportionalController(kp_linear=2.0, kp_angular=2.0, max_vel=2.5, max_angular_vel=5.0)
    action = ActionClient(port=porta_comando)
    
    # 3. Inicializa o WorldModel e o Blackboard
    world = WorldModel(is_yellow=is_yellow)
    bb = Blackboard(controller, action)
    
    cycle_time = 1.0 / 60 # Define o tempo de ciclo para 60Hz
    last_debug_time = time.time()

    # Gera os diagramas visuais
    arvore_mestra = build_master_tree()
    export_tree_to_xml(arvore_mestra, "diagramas/arvore_mestra.xml")
    
    while True:
        start_time = time.time()
        
        # ==========================================
        # FASE 1: PERCEPÇÃO (Atualizar o WorldModel)
        # ==========================================
        cmd, stage, desig_x, desig_y = referee.get_latest_command()
        
        if cmd is not None:
            # Lógica do árbitro mantida fora do WorldModel por gerar efeitos colaterais visuais/simulador
            if world.referee.command != cmd: 
                print(f"JUIZ APITOU: {cmd} (Fase: {stage})")
                
                bb.bola_em_jogo_forcada = False
                if world.ball.visible:
                    bb.posicao_bola_no_apito = (world.ball.x, world.ball.y)
                else:
                    bb.posicao_bola_no_apito = None
                    
            if desig_x is not None and desig_y is not None:
                nova_posicao = (desig_x, desig_y)
                if getattr(world.referee, 'designated_position', None) != nova_posicao:
                    print(f"⚽ Sumatra/AutoRef: Reposicionando a bola para X:{desig_x:.2f}, Y:{desig_y:.2f}")
                    teleporta_bola_simulador(desig_x, desig_y)
            
            # Atualiza efetivamente a camada de percepção
            designated_tuple = (desig_x, desig_y) if desig_x is not None else None
            world.update_referee(cmd, stage, designated_tuple)
            
        state = vision.get_latest_state()
        if state is not None:
            world.update_vision(state.last_seen_world) 

        # ==========================================
        # FASE 1.5: CAMADA DE COMPATIBILIDADE
        # ==========================================
        # Alimentamos o Blackboard para não quebrar a Behavior Tree e Actions atuais
        
        bb.is_yellow = world.is_yellow
        bb.our_goal_x = world.field.our_goal_x
        bb.enemy_goal_x = world.field.enemy_goal_x
        bb.referee_command = world.referee.command
        bb.referee_stage = world.referee.stage
        
        if world.ball.visible:
            # Duck typing 
            # para imitar o Protobuf
            world.ball.pos = PosMock(world.ball.x, world.ball.y)
            bb.ball_pos = world.ball
            bb.ball_vel_x = world.ball.vx
            bb.ball_vel_y = world.ball.vy
            
            # DETECTOR DE BOLA EM JOGO (O Hack do Juiz)
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
                
                if speed > 1.0 or dist_movida > 1.5:
                    bb.bola_em_jogo_forcada = True
                    print("⚡ I.A.: A bola andou! Forçando o NORMAL_START e desfazendo a formação!")

        # Filtra os robôs e injeta a propriedade 'pos' falsa para manter compatibilidade
        team_robots = []
        for r in world.get_my_team().values():
            if r.visible:
                r.pos = PosMock(r.x, r.y, r.yaw)
                r.orientation = r.yaw
                r.theta = r.yaw
                team_robots.append(r)
                
        enemies_list = []
        for r in world.get_opponent_team().values():
            if r.visible:
                r.pos = PosMock(r.x, r.y, r.yaw)
                r.orientation = r.yaw
                r.theta = r.yaw
                enemies_list.append(r)
                
        bb.enemies = enemies_list

        # ==========================================
        # FASE 2: O MAESTRO (Distribuição Tática)
        # ==========================================
        papeis_anteriores = getattr(bb, 'papeis', {})
        
        # Maestro agora recebe listas limpas (sem fantasmas da visão)
        papeis_do_time = maestro_distribui_papeis(team_robots, getattr(bb, 'ball_pos', None), bb.enemy_goal_x, papeis_anteriores, id_goleiro=0)
        
        bb.papeis = papeis_do_time
        bb.team = team_robots

        # ==========================================
        # FASE 3: COMPORTAMENTO E AÇÃO (Multi-Agente)
        # ==========================================
        for robo in team_robots:
            robo_id = robo.id
            
            # Verifica se o Maestro deu um papel para este robô
            if robo_id in papeis_do_time:
                # 1. Troca a "lente" do Blackboard para a perspectiva deste robô
                bb.my_id = robo_id
                bb.my_pos = robo
                bb.my_role = papeis_do_time[robo_id]
                
                # 2. CONSTRÓI OS OBSTÁCULOS PARA ESTE ROBÔ
                bb.obstacles = []
                # Adiciona companheiros (exceto ele mesmo)
                for r in team_robots:
                    if r.id != bb.my_id:
                        bb.obstacles.append(r)
                # Adiciona os inimigos
                for r in enemies_list:
                    bb.obstacles.append(r)
                
                # Paredes de defesa (se configurado em bb)
                if bb.my_role != "GOLEIRO" and hasattr(bb, 'defense_walls'):
                    bb.obstacles.extend(bb.defense_walls)
                
                # 3. Aciona o Cérebro
                arvore_mestra.tick(bb)
                    
        # ==========================================
        # PAINEL DE DEBUG
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
        elapsed_time = time.time() - start_time
        sleep_time = cycle_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()
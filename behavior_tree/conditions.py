import math
import time
from .core import Node, NodeState

# ==========================================
# NÓS DE CONDIÇÃO (O Juiz)
# ==========================================
class ConditionIsGameRunning(Node):
    """Retorna SUCCESS se o jogo estiver valendo, caso contrário FAILURE."""
    def tick(self, blackboard):
        # Lê o comando atual do juiz no quadro negro
        if blackboard.referee_command in ["NORMAL_START", "FORCE_START"]:
            return NodeState.SUCCESS
        return NodeState.FAILURE

class ConditionIsHalted(Node):
    """Retorna SUCCESS se o juiz mandou parar tudo (HALT)."""
    def tick(self, blackboard):
        if blackboard.referee_command in ["HALT", "STOP"]:
            return NodeState.SUCCESS
        return NodeState.FAILURE

class ConditionIsPrepareKickoff(Node):
    """Retorna SUCCESS se estamos na fase de preparação do chute inicial."""
    def tick(self, blackboard):
        # Verifica se o comando é de preparação e se é para o nosso time (Amarelo)
        if blackboard.referee_command == "PREPARE_KICKOFF_YELLOW" and blackboard.is_yellow:
            return NodeState.SUCCESS
        
        # Lógica análoga caso o time fosse azul
        if blackboard.referee_command == "PREPARE_KICKOFF_BLUE" and not blackboard.is_yellow:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE
    
# ==========================================
# NÓS DE CONDIÇÃO (Ofensivas)
# ==========================================
class ConditionIsNearBall(Node):
    """Retorna SUCCESS se o robô estiver com a posse de bola (muito perto dela)."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE
            
        # Calcula a distância exata até a bola[cite: 1]
        dist = math.hypot(blackboard.ball_pos.x - blackboard.my_pos.pos.x,
                          blackboard.ball_pos.y - blackboard.my_pos.pos.y)
                          
        # Aquele nosso limite de 12cm que já estava funcionando bem[cite: 1]
        if dist < 0.12:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE

class ConditionIsInShootingZone(Node):
    """Retorna SUCCESS se o robô estiver a menos de 4 metros do gol inimigo."""
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE
            
        # Coordenadas do gol inimigo corrigidas para o campo longo (X negativo)
        gol_inimigo_x = blackboard.enemy_goal_x
        gol_inimigo_y = 0.0
        
        dist_gol = math.hypot(gol_inimigo_x - blackboard.my_pos.pos.x, 
                              gol_inimigo_y - blackboard.my_pos.pos.y)
                              
        if dist_gol < 4.0:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE

class ConditionIsPathClear(Node):
    """
    Escaneia o gol (Centro, Direita, Esquerda).
    Retorna SUCCESS se achar uma fresta e salva o alvo no Blackboard.
    """
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        gol_inimigo_x = blackboard.enemy_goal_x
        
        # O gol da SSL tem aprox 1 metro de largura (Y vai de -0.5 a 0.5)
        # Vamos testar o centro (0.0) e os cantos (0.60 e -0.60)
        alvos_y = [0.0, 0.60, -0.60] 
        
        start_x = blackboard.my_pos.pos.x
        start_y = blackboard.my_pos.pos.y
        safe_radius = 0.18 # Raio de segurança
        
        for alvo_y in alvos_y:
            caminho_limpo = True
            dx = gol_inimigo_x - start_x
            dy = alvo_y - start_y
            segment_length_squared = dx*dx + dy*dy
            
            if segment_length_squared == 0:
                continue
                
            # Escaneia os obstáculos contra esta linha de chute
            for obs in blackboard.obstacles:
                if not hasattr(obs, 'id'): continue # Ignora paredes virtuais
                    
                px = obs.pos.x - start_x
                py = obs.pos.y - start_y
                
                t = max(0, min(1, (px * dx + py * dy) / segment_length_squared))
                proj_x = start_x + t * dx
                proj_y = start_y + t * dy
                
                dist_to_line = math.hypot(obs.pos.x - proj_x, obs.pos.y - proj_y)
                if dist_to_line < safe_radius:
                    caminho_limpo = False
                    break # Bateu num obstáculo, essa linha falhou!
                    
            if caminho_limpo:
                # ACHAMOS UMA FRESTA! Salva no quadro negro e autoriza o chute.
                blackboard.best_shot_y = alvo_y
                return NodeState.SUCCESS
                
        # Se o loop terminar, todas as 3 linhas estão bloqueadas
        return NodeState.FAILURE
    
class ConditionIsPassClear(Node):
    """
    Localiza o Atacante de Apoio. Traça uma reta até ele.
    Se não houver inimigos no caminho, salva o alvo e retorna SUCCESS.
    """
    def tick(self, blackboard):
        # Proteção: Verifica se temos as informações do time
        if blackboard.my_pos is None or not hasattr(blackboard, 'papeis') or not hasattr(blackboard, 'team'):
            return NodeState.FAILURE

        # 1. Quem é o Apoio? (Lê os crachás distribuídos pelo Maestro)
        apoio_id = None
        for r_id, papel in blackboard.papeis.items():
            if papel == "ATACANTE_APOIO":
                apoio_id = r_id
                break
                
        if apoio_id is None:
            return NodeState.FAILURE # Ninguém escalado como Apoio no momento
            
        # 2. Onde está o Apoio? (Pega as coordenadas dele)
        apoio_pos = None
        for r in blackboard.team:
            if getattr(r, 'id', -1) == apoio_id:
                apoio_pos = r
                break
                
        if apoio_pos is None:
            return NodeState.FAILURE

        # 3. Matemática do Raycast (Linha de Passe)
        start_x = blackboard.my_pos.pos.x
        start_y = blackboard.my_pos.pos.y
        dx = apoio_pos.pos.x - start_x
        dy = apoio_pos.pos.y - start_y
        segment_length_squared = dx*dx + dy*dy
        
        if segment_length_squared == 0:
            return NodeState.FAILURE
            
        safe_radius = 0.20 # A bola precisa de uma folga de 20cm dos inimigos
        caminho_limpo = True
        
        # 4. Checa apenas contra os INIMIGOS (bb.enemies foi criado antes!)
        if hasattr(blackboard, 'enemies'):
            for obs in blackboard.enemies:
                if abs(obs.pos.x) < 0.001 and abs(obs.pos.y) < 0.001: 
                    continue # Ignora fantasmas
                    
                px = obs.pos.x - start_x
                py = obs.pos.y - start_y
                
                # Projeta o inimigo na linha do passe
                t = max(0, min(1, (px * dx + py * dy) / segment_length_squared))
                proj_x = start_x + t * dx
                proj_y = start_y + t * dy
                
                # Se a distância do inimigo para a linha do passe for menor que a segurança, bloqueou!
                dist_to_line = math.hypot(obs.pos.x - proj_x, obs.pos.y - proj_y)
                if dist_to_line < safe_radius:
                    caminho_limpo = False
                    break
                
        if caminho_limpo:
            # Caminho livre! Salva a posição exata do companheiro para o músculo agir.
            blackboard.pass_target = apoio_pos
            return NodeState.SUCCESS
            
        return NodeState.FAILURE
    
class ConditionIsPassArriving(Node):
    """Retorna SUCCESS se a bola estiver rápida e vindo na direção do robô."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE
            
        speed = math.hypot(blackboard.ball_vel_x, blackboard.ball_vel_y)
        if speed < 0.6: # Bola lenta demais, compensa dominar normal
            return NodeState.FAILURE
            
        # Vetor do robô apontando para a bola
        dx_robo = blackboard.my_pos.pos.x - blackboard.ball_pos.x
        dy_robo = blackboard.my_pos.pos.y - blackboard.ball_pos.y
        dist_robo = math.hypot(dx_robo, dy_robo)
        
        if dist_robo == 0 or dist_robo > 3.0: 
            return NodeState.FAILURE # Longe demais da jogada
            
        # Produto Escalar (Dot Product) para saber se a bola vem na nossa direção
        nx_bola = blackboard.ball_vel_x / speed
        ny_bola = blackboard.ball_vel_y / speed
        
        nx_robo = dx_robo / dist_robo
        ny_robo = dy_robo / dist_robo
        
        dot_product = (nx_bola * nx_robo) + (ny_bola * ny_robo)
        
        # Se for > 0.7, a trajetória da bola aponta diretamente (cone de ~45º) para o robô!
        if dot_product > 0.7: 
            return NodeState.SUCCESS
            
        return NodeState.FAILURE
    
# ==========================================
# NÓS DE CONDIÇÃO (goleiro)
# ==========================================

class ConditionIsBallSafeToClear(Node):
    """
    Retorna SUCCESS se a bola estiver dentro da nossa área E quase parada.
    Impede que o goleiro tente dominar um chute a 80km/h.
    """
    def __init__(self):
        super().__init__()
        self.last_pos = None
        self.last_time = time.time()
        self.speed = 0.0

    def tick(self, blackboard):
        if blackboard.ball_pos is None:
            return NodeState.FAILURE
            
        current_time = time.time()
        
        # 1. Calcula a Velocidade da Bola
        if self.last_pos is not None:
            dt = current_time - self.last_time
            if dt > 0:
                dx = blackboard.ball_pos.x - self.last_pos.x
                dy = blackboard.ball_pos.y - self.last_pos.y
                self.speed = math.hypot(dx, dy) / dt
                
        self.last_pos = blackboard.ball_pos
        self.last_time = current_time
        
        # 2. Verifica se a bola está dentro da nossa área de defesa
        if blackboard.our_goal_x > 0:
            na_area = (4.5 <= blackboard.ball_pos.x <= 6.0) and (-1.5 <= blackboard.ball_pos.y <= 1.5)
        else:
            na_area = (-6.0 <= blackboard.ball_pos.x <= -4.5) and (-1.5 <= blackboard.ball_pos.y <= 1.5)
        
        # 3. A Regra de Ouro: Tá na área E tá lenta (menos de 1.2 m/s)? Limpa!
        if na_area and self.speed < 1.2:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE
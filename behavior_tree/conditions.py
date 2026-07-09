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
        if blackboard.referee_command == "HALT":
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
        gol_inimigo_x = -6.0
        gol_inimigo_y = 0.0
        
        dist_gol = math.hypot(gol_inimigo_x - blackboard.my_pos.pos.x, 
                              gol_inimigo_y - blackboard.my_pos.pos.y)
                              
        if dist_gol < 4.0:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE

class ConditionIsPathClear(Node):
    """
    Retorna SUCCESS se não houver NENHUM robô físico entre o atacante e o gol.
    Ignora as paredes virtuais na verificação.
    """
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        gol_inimigo_x = -6.0
        gol_inimigo_y = 0.0
        
        start_x = blackboard.my_pos.pos.x
        start_y = blackboard.my_pos.pos.y

        dx = gol_inimigo_x - start_x
        dy = gol_inimigo_y - start_y
        segment_length_squared = dx*dx + dy*dy
        
        if segment_length_squared == 0:
            return NodeState.SUCCESS
            
        safe_radius = 0.18 # Raio do robô + Raio da bola + Folga
        
        # Lê os obstáculos diretamente do Blackboard[cite: 1]
        for obs in blackboard.obstacles:
            # Ignora as paredes virtuais (elas não têm o atributo 'id')
            if not hasattr(obs, 'id'):
                continue
                
            px = obs.pos.x - start_x
            py = obs.pos.y - start_y
            
            # Projeta o obstáculo na linha de chute
            t = max(0, min(1, (px * dx + py * dy) / segment_length_squared))
            proj_x = start_x + t * dx
            proj_y = start_y + t * dy
            
            # Se a distância do obstáculo até a linha de chute for menor que a segurança, bloqueia!
            dist_to_line = math.hypot(obs.pos.x - proj_x, obs.pos.y - proj_y)
            if dist_to_line < safe_radius:
                return NodeState.FAILURE # Caminho sujo!
                
        return NodeState.SUCCESS # Caminho limpo!
    
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
        na_area = (-6.0 <= blackboard.ball_pos.x <= -4.5) and (-1.5 <= blackboard.ball_pos.y <= 1.5)
        
        # 3. A Regra de Ouro: Tá na área E tá lenta (menos de 1.2 m/s)? Limpa!
        if na_area and self.speed < 1.2:
            return NodeState.SUCCESS
            
        return NodeState.FAILURE
import time
from navigation.Obstacles import create_solid_defense_walls
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
        self.papeis = {}      # <--- A CORREÇÃO ESTÁ AQUI!

        self.ball_vel_x = 0.0
        self.ball_vel_y = 0.0
        self.last_ball_pos = None
        self.last_ball_time = time.time()
        
        # O Muro Físico das áreas gerado apenas uma vez
        self.defense_walls = create_solid_defense_walls() 
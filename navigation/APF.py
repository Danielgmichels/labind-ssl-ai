import math
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
        self.dist_segura = 0.20  # Raio do robô + folga mínima
        self.kr_repulsao = 0.017  # Repulsão base muito mais suave

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
        dist_segura_dinamica = self.dist_segura + (fator_velocidade * 0.20)
        
        # Aumenta a força repulsiva de forma sutil
        kr_dinamico = self.kr_repulsao + (fator_velocidade * 0.10)
        
        # 2. FORÇA DE REPULSÃO (Com correção GNRON e Paredes Sólidas)
        rep_x = 0.0
        rep_y = 0.0
        
        fator_foco = min(dist_bola / dist_segura_dinamica, 1.0)
        
        # --- A CORREÇÃO DO PÂNICO ---
        # Normaliza o vetor de atração para saber para onde estamos querendo ir.
        # Isso nos permite ignorar inimigos que estão nas nossas costas.
        dir_mov_x, dir_mov_y = 0.0, 0.0
        if mag_attr > 0:
            dir_mov_x = vx_global / mag_attr
            dir_mov_y = vy_global / mag_attr
        
        for obs in obstacles:
            dx = robot_x - obs.pos.x  
            dy = robot_y - obs.pos.y
            dist_obs = math.hypot(dx, dy)
            
            # A MÁGICA: É uma parede virtual ou um robô físico?
            is_wall = not hasattr(obs, 'id')
            
            if is_wall:
                # PAREDE DE CONCRETO: Empurra muito forte e de mais longe
                dist_segura_atual = 0.85  # Sente a parede a 40cm de distância
                kr_atual = 3.0            # Força brutal (ignora a atração da bola)
                fator_foco_atual = 1.0    # Sempre empurra com 100% de prioridade
            else:
                # ROBÔ DE ESPUMA: Repulsão suave configurada anteriormente
                dist_segura_atual = dist_segura_dinamica
                kr_atual = kr_dinamico
                fator_foco_atual = fator_foco
            
            # --- NOVO: Checagem de "Obstáculo Frontal" ---
            # Se o obstáculo não for uma parede, verificamos se ele está na nossa frente.
            if not is_wall:
                # Vetor do robô para o obstáculo
                vetor_para_obs_x = obs.pos.x - robot_x
                vetor_para_obs_y = obs.pos.y - robot_y
                
                # Produto escalar: se for < 0, o obstáculo está atrás. Ignoramos!
                dot_product = (dir_mov_x * vetor_para_obs_x) + (dir_mov_y * vetor_para_obs_y)
                if dot_product < 0:
                    continue

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
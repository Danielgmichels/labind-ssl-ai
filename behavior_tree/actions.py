import math
from .core import Node, NodeState

# ==========================================
# NÓS DE AÇÃO SIMPLES (Segurança)
# ==========================================
class ActionStopMotors(Node):
    """Para completamente os motores do robô em caso de HALT ou STOP."""
    def tick(self, blackboard):
        # A árvore agora envia o comando diretamente usando o cliente salvo no Blackboard
        blackboard.action.send_command(
            robot_id=blackboard.my_id,
            v_forward=0.0,
            v_left=0.0,
            vw=0.0,
            kick_speed=0.0,
            dribbler_speed=0.0
        )
        return NodeState.SUCCESS # Retorna sucesso, pois a ordem de parar foi dada

# ==========================================
# NÓS DE AÇÃO (Posicionamento e Navegação)
# ==========================================
class ActionPrepareKickoff(Node):
    """
    Usa o APF para navegar até um ponto virtual 25cm atrás da bola.
    Mantém o robô rotacionado encarando a bola enquanto espera o apito.
    """
    def tick(self, blackboard):
        # Proteção: Se a câmera piscar e perdermos a bola ou o robô, falha o nó
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. O Alvo Virtual
        # Como o time ataca para a esquerda (X negativo), nós posicionamos 
        # o robô no lado direito da bola (X positivo) para ele chutar para a esquerda.
        # Descobre a direção (1 se positivo, -1 se negativo)
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        alvo_x = blackboard.ball_pos.x - (direcao * 0.25) 
        alvo_y = blackboard.ball_pos.y
        
        # 2. Navegação com APF
        # Agora acessamos o controller e os obstáculos diretamente do Blackboard
        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 3. Rotação (Mira na bola)
        # Ignoramos a rotação do APF e forçamos o bico a olhar para a bola
        import math # Certifique-se de que o math foi importado no topo do arquivo
        
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        if abs(erro_angular) < 0.05:
            vw = 0.0
        else:
            vw = erro_angular * blackboard.controller.kp_angular
            # Limita a velocidade angular máxima
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
            
        # 4. Envio do Comando Físico
        blackboard.action.send_command(
            robot_id=blackboard.my_id,
            v_forward=vf,
            v_left=vl,
            vw=vw,
            kick_speed=0.0,
            dribbler_speed=0.0 # Rolo desligado durante a preparação
        )
        
        # Retorna RUNNING, pois o robô deve continuar rodando esta lógica 
        # para se equilibrar na posição até o juiz apitar o NORMAL_START.
        return NodeState.RUNNING

# ==========================================
# NÓS DE AÇÃO (Condução e Finalização)
# ==========================================
class ActionGoToBall(Node):
    """Navega até a bola usando APF. Se a bola estiver na área, espera na borda."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        alvo_x = blackboard.ball_pos.x
        alvo_y = blackboard.ball_pos.y
        
        # Proteção de Geofencing: Se a bola rolou pra dentro da área
        # (Lembrando que o campo longo vai até -6.0 e 6.0, com a área até -5.0 e 5.0)
        na_area_esq = (-6.0 <= alvo_x <= -5.0) and (-1.5 <= alvo_y <= 1.5)
        na_area_dir = (5.0 <= alvo_x <= 6.0) and (-1.5 <= alvo_y <= 1.5)
        
        if na_area_esq:
            alvo_x = -4.8
            alvo_y = max(-1.2, min(alvo_y, 1.2))
        elif na_area_dir:
            alvo_x = 4.8
            alvo_y = max(-1.2, min(alvo_y, 1.2))


        # APF para calcular o caminho, desviando de robôs e das paredes virtuais[cite: 1]
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=0.0
        )
        return NodeState.RUNNING


class ActionSmartDribble(Node):
    """Conduz a bola para o ataque, mas aplica um drible lateral (Strafing) se houver inimigos na frente."""
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        # Alvo base inicial: O Gol inimigo
        alvo_x = blackboard.enemy_goal_x
        alvo_y = 0.0

        # 1. Escaneamento Frontal (O Radar do Drible)
        inimigo_na_frente = False
        if hasattr(blackboard, 'enemies'):
            for obs in blackboard.enemies:
                # Ignora "fantasmas" do simulador
                if abs(obs.pos.x) < 0.001 and abs(obs.pos.y) < 0.001: 
                    continue
                
                dx = obs.pos.x - blackboard.my_pos.pos.x
                dy = obs.pos.y - blackboard.my_pos.pos.y
                dist_obs = math.hypot(dx, dy)
                
                # Se o inimigo está a menos de 80cm (Zona de Perigo)
                if dist_obs < 0.8:
                    # Verifica se ele está bloqueando a nossa frente (Cone de ~60 graus)
                    angulo_inimigo = math.atan2(dy, dx)
                    erro = (angulo_inimigo - blackboard.my_pos.yaw + math.pi) % (2 * math.pi) - math.pi
                    
                    if abs(erro) < 0.6: # Está diretamente na nossa cara!
                        inimigo_na_frente = True
                        
                        # A MÁGICA DO DRIBLE: Decide pra qual lado puxar a bola
                        if obs.pos.y > blackboard.my_pos.pos.y:
                            # Inimigo fechando pela esquerda, puxa a bola pra direita!
                            alvo_y = blackboard.my_pos.pos.y - 1.0 
                        else:
                            # Inimigo fechando pela direita, puxa a bola pra esquerda!
                            alvo_y = blackboard.my_pos.pos.y + 1.0 
                            
                        # Trava o avanço no eixo X temporariamente para não bater nele
                        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
                        alvo_x = blackboard.my_pos.pos.x + (direcao * 0.5)
                        break # Executa o drible no inimigo mais imediato

        # 2. Navegação com APF usando o novo alvo dinâmico
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 3. O Freio de Evasão
        if inimigo_na_frente:
            # Se forçou o drible, reduz a velocidade pra frente pela metade 
            # para dar tempo da velocidade lateral (vl) fazer a curva
            vf = vf * 0.5 
            
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=1500.0 # Segura a bola na boca firme!
        )
        return NodeState.RUNNING


class ActionAimAndShoot(Node):
    """Gira no próprio eixo mirando no alvo livre (best_shot_y) e atira."""
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        gol_inimigo_x = blackboard.enemy_goal_x
        
        # Lê o alvo salvo pelo Radar! Se não tiver, chuta no meio.
        gol_inimigo_y = getattr(blackboard, 'best_shot_y', 0.0)

        # 1. Mira exata na fresta
        target_angle = math.atan2(gol_inimigo_y - blackboard.my_pos.pos.y, 
                                  gol_inimigo_x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        
        raio_do_robo = 0.09 
        vl = vw * raio_do_robo 
        vf = 0.5 
        velocidade_chute = 0.0
        
        if abs(erro_angular) < 0.1:
            vw = 0.0
            vl = 0.0
            velocidade_chute = 6.0 
            vf = 1.0 
            
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=velocidade_chute, dribbler_speed=1500.0
        )
        return NodeState.RUNNING


class ActionFindShootingAngle(Node):
    """Fica de frente para o gol rodando o driblador e anda de lado (Strafing) até abrir espaço."""
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        gol_inimigo_x = blackboard.enemy_goal_x
        gol_inimigo_y = 0.0
        
        # Mantém a mira no centro do gol enquanto anda
        target_angle = math.atan2(gol_inimigo_y - blackboard.my_pos.pos.y, 
                                  gol_inimigo_x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        
        # A Mágica do Passo Lateral: 
        # Forçamos o v_forward a quase zero (para não bater na zaga)
        # E colocamos uma velocidade em v_left para ele deslizar lateralmente
        vf = 0.1 
        vl = 1.0 # Velocidade lateral constante
        
        # Se estiver muito para baixo no campo (Y negativo), inverte o lado para subir
        if blackboard.my_pos.pos.y < 0:
            vl = -1.0 

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=1500.0 # Segura a bola firme!
        )
        return NodeState.RUNNING
    
class ActionPositionForPass(Node):
    """
    Corre para uma das pontas da área para dar opção de passe.
    Fica cravado no seu lado (Esquerdo ou Direito) para evitar caos no meio-campo.
    """
    def __init__(self, lado_y):
        super().__init__()
        self.lado_y = lado_y

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. O Alvo
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        alvo_x = blackboard.enemy_goal_x - (direcao * 2.5) 
        
        # AQUI ESTÁ A CURA DO CAOS: O Y agora é travado no lado do robô!
        alvo_y = self.lado_y
        
        # 2. Navegação com APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 3. Gira como um radar (Bico apontado para a bola)
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        if abs(erro_angular) > 0.05:
            vw = erro_angular * blackboard.controller.kp_angular
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        else:
            vw = 0.0
            
        # 4. Freio de precisão
        dist_alvo = math.hypot(alvo_x - blackboard.my_pos.pos.x, alvo_y - blackboard.my_pos.pos.y)
        if dist_alvo < 0.3:
            vf = 0.0
            vl = 0.0

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=1500.0 
        )
        return NodeState.RUNNING
    
class ActionPassBall(Node):
    """
    Gira encarando o parceiro de time (salvo no pass_target).
    Calcula a força ideal do chute baseada na distância para não espirrar a bola.
    """
    def tick(self, blackboard):
        if blackboard.my_pos is None or not hasattr(blackboard, 'pass_target'):
            return NodeState.FAILURE
            
        alvo = blackboard.pass_target
        
        target_angle = math.atan2(alvo.pos.y - blackboard.my_pos.pos.y, 
                                  alvo.pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        
        raio_do_robo = 0.09 
        vl = vw * raio_do_robo 
        vf = 0.5 
        velocidade_chute = 0.0
        
        # === A CORREÇÃO DO SNIPER VEM AQUI ===
        # Exige alinhamento quase perfeito (0.03 radianos) antes de soltar a bomba
        if abs(erro_angular) < 0.02:
            vw = 0.0
            vl = 0.0
            
            dist_passe = math.hypot(alvo.pos.x - blackboard.my_pos.pos.x, alvo.pos.y - blackboard.my_pos.pos.y)
            velocidade_chute = min(dist_passe * 1.8, 6.0) 
            
            # FREIA O ROBÔ: Garante que a bola bata limpa no chutador e não nas rodas
            vf = 0.0 
            
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=velocidade_chute, dribbler_speed=1500.0
        )
        return NodeState.RUNNING
    

class ActionInterceptPass(Node):
    """Intercepta a bola em movimento de frente para dominá-la com precisão magnética."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE
            
        # 1. Projeção Vetorial Filtrada (Onde a bola vai passar)
        speed = math.hypot(blackboard.ball_vel_x, blackboard.ball_vel_y)
        if speed > 0:
            nx = blackboard.ball_vel_x / speed
            ny = blackboard.ball_vel_y / speed
        else:
            nx, ny = 0, 0
            
        dx = blackboard.my_pos.pos.x - blackboard.ball_pos.x
        dy = blackboard.my_pos.pos.y - blackboard.ball_pos.y
        
        t = dx * nx + dy * ny 
        
        # O ponto matemático exato
        intercept_x = blackboard.ball_pos.x + nx * t
        intercept_y = blackboard.ball_pos.y + ny * t
        
        # --- A MÁGICA DA RECEPÇÃO VEM AQUI ---
        # "Ataca a bola": Avança o robô 15cm na direção contrária de onde a bola está indo.
        # Isso garante que a bola colida de chapa no driblador e não nas rodas!
        intercept_x -= nx * 0.15
        intercept_y -= ny * 0.15
        
        # 2. Navega IGNORANDO as paredes do APF (lista vazia []) para não ser repelido da jogada
        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            intercept_x, intercept_y, [] 
        )
        
        # 3. Gira para olhar diretamente para a BOLA
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        
        # 4. Motor e Driblador
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=1500.0
        )
        return NodeState.RUNNING
    
# ==========================================
# GOLEIRO
# ==========================================
class ActionDefendGoal(Node):
    """Fica na linha do gol cortando o ângulo, e intercepta chutes rápidos."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        direcao = 1 if blackboard.our_goal_x > 0 else -1
        
        # 1. Linha de atuação cravada embaixo da trave
        alvo_x = blackboard.our_goal_x - (direcao * 0.2) 
        
        # 2. A Mágica da Previsão do Chute
        # Verifica se temos a velocidade filtrada salva no Blackboard
        vx = getattr(blackboard, 'ball_vel_x', 0.0)
        vy = getattr(blackboard, 'ball_vel_y', 0.0)
        speed = math.hypot(vx, vy)
        
        # A bola está vindo na direção do nosso gol?
        bola_vindo_pra_nos = (direcao > 0 and vx > 0) or (direcao < 0 and vx < 0)
        
        if speed > 0.8 and bola_vindo_pra_nos and abs(vx) > 0.001:
            # INTERCEPTAÇÃO: Calcula onde a reta do chute vai cruzar a nossa linha do gol
            m = vy / vx
            alvo_y = blackboard.ball_pos.y + m * (alvo_x - blackboard.ball_pos.x)
        else:
            # FECHAR O ÂNGULO: A bola tá lenta ou no ataque, corta a linha pro centro do gol
            dx = blackboard.our_goal_x - blackboard.ball_pos.x
            dy = 0.0 - blackboard.ball_pos.y
            if abs(dx) > 0.001:
                m = dy / dx
                alvo_y = blackboard.ball_pos.y + m * (alvo_x - blackboard.ball_pos.x)
            else:
                alvo_y = 0.0
            
        # 3. Trava de Segurança: Não deixa o goleiro bater nas traves físicas (largura do gol)
        alvo_y = max(-0.80, min(alvo_y, 0.80))
        
        # 4. Navegação 100% focada, ignorando as barreiras da área para o goleiro poder se mexer solto
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, [] # O Goleiro ignora paredes virtuais!
        )
        
        # 5. Gira encarando a bola
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=0.0
        )
        return NodeState.RUNNING


class ActionClearBall(Node):
    """Vai até a bola na área olhando para ela, domina, gira pro meio campo e cruza por cima."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        alvo_x = blackboard.ball_pos.x
        alvo_y = blackboard.ball_pos.y
        
        # Calcula a distância do goleiro até a bola
        dist_bola = math.hypot(alvo_x - blackboard.my_pos.pos.x, alvo_y - blackboard.my_pos.pos.y)
        
        velocidade_driblador = 1500.0 # Sempre ligado para garantir a posse!
        velocidade_chute = 0.0
        angulo_chute = 0.0

        if dist_bola > 0.15:
            # ==========================================
            # FASE 1: BUSCAR A BOLA OLHANDO PARA ELA
            # ==========================================
            # 1. Mira o bico do robô diretamente para a bola
            target_angle = math.atan2(alvo_y - blackboard.my_pos.pos.y, alvo_x - blackboard.my_pos.pos.x)
            erro_angular = target_angle - blackboard.my_pos.yaw
            erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
            
            vw = erro_angular * blackboard.controller.kp_angular
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
            
            # 2. Navega até a bola com o APF normal
            vf, vl, _ = blackboard.controller.calculate_velocity(
                blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
                alvo_x, alvo_y, blackboard.obstacles
            )
            
        else:
            # ==========================================
            # FASE 2: DOMINOU A BOLA! GIRA (PIVÔ) E CRUZA
            # ==========================================
            # 1. Muda a mira para o meio do campo (0.0, 0.0)
            target_angle = math.atan2(0.0 - blackboard.my_pos.pos.y, 0.0 - blackboard.my_pos.pos.x)
            erro_angular = target_angle - blackboard.my_pos.yaw
            erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
            
            vw = erro_angular * blackboard.controller.kp_angular
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
            
            # 2. Desliga o APF e usa a Matemática do Pivô do Atacante
            # Isso garante que ele rode de lado abraçando a bola com o driblador
            raio_do_robo = 0.09 
            vl = vw * raio_do_robo 
            vf = 0.2
            
            # 3. Se alinhou com o meio campo, dá o chutão de cavadinha!
            if abs(erro_angular) < 0.15:
                vw = 0.0
                vl = 0.0
                vf = 1.0 # Arrancada do impacto
                velocidade_chute = 6.0
                angulo_chute = 45.0 # Mágica do cruzamento!
            
        # Envia os comandos finais
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=velocidade_chute, kick_angle=angulo_chute, dribbler_speed=velocidade_driblador
        )
        
        return NodeState.RUNNING
    

# ==========================================
# ZAQUEIROS
# ==========================================

class ActionZagueiroBloqueio(Node):
    """Zagueiro que forma a parede, mas que toma iniciativa e isola a bola se ela cair no pé dele."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # A que distância a bola está de mim agora?
        dist_bola = math.hypot(blackboard.ball_pos.x - blackboard.my_pos.pos.x, 
                               blackboard.ball_pos.y - blackboard.my_pos.pos.y)
                               
        velocidade_chute = 0.0
        velocidade_driblador = 0.0

        # 1. INSTINTO DE DEFESA (Bola invadiu a zona de 60cm dele)
        if dist_bola < 0.60:
            # Abandona a linha da barreira e ataca a bola de frente!
            alvo_x = blackboard.ball_pos.x
            alvo_y = blackboard.ball_pos.y
            velocidade_driblador = 1500.0 # Liga a boca pra garantir o rebote
            
            # Se já engoliu a bola (chegou a 15cm), dá o chutão de alívio pra frente!
            if dist_bola < 0.15:
                velocidade_chute = 6.0 
        else:
            # 2. A MATEMÁTICA DO ARCO (Comportamento normal da Barreira)
            dx = blackboard.ball_pos.x - blackboard.our_goal_x
            dy = blackboard.ball_pos.y - 0.0
            angulo_bola = math.atan2(dy, dx)
            
            raio_zaga = 2.0 # Fica seguro fora da grande área
            
            alvo_x = blackboard.our_goal_x + raio_zaga * math.cos(angulo_bola)
            alvo_y = 0.0 + raio_zaga * math.sin(angulo_bola)
        
        # 3. Navegação com APF
        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 4. Gira sempre encarando a bola para estar pronto pro bote
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)

        # Envia os comandos (agora injetando o chute e o driblador caso esteja limpando a bola)
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=velocidade_chute, dribbler_speed=velocidade_driblador
        )
        return NodeState.RUNNING
        return NodeState.RUNNING


class ActionZagueiroMarcacao(Node):
    """Zagueiro carrapato. Encontra o inimigo mais perto e se põe na frente dele."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        inimigo_perigoso = None
        menor_dist = float('inf')
        
        # 1. Lê a lista de inimigos (Vamos injetar isso no main.py em breve!)
        if hasattr(blackboard, 'enemies'):
            for inimigo in blackboard.enemies:
                # Ignora os "fantasmas" (falhas de visão)
                if abs(inimigo.pos.x) < 0.001 and abs(inimigo.pos.y) < 0.001:
                    continue
                
                # Ameaça = Quão perto o inimigo está do nosso gol
                dist_pro_gol = math.hypot(inimigo.pos.x - blackboard.our_goal_x, inimigo.pos.y - 0.0)
                if dist_pro_gol < menor_dist:
                    menor_dist = dist_pro_gol
                    inimigo_perigoso = inimigo
                    
        # 2. Posicionamento
        if inimigo_perigoso is not None:
            # Pega o vetor do inimigo apontando para a bola
            dx_bola = blackboard.ball_pos.x - inimigo_perigoso.pos.x
            dy_bola = blackboard.ball_pos.y - inimigo_perigoso.pos.y
            dist_inimigo_bola = math.hypot(dx_bola, dy_bola)
            
            # Fica cravado 40cm na frente do inimigo, cortando a linha de passe dele
            if dist_inimigo_bola > 0:
                alvo_x = inimigo_perigoso.pos.x + (dx_bola / dist_inimigo_bola) * 0.4
                alvo_y = inimigo_perigoso.pos.y + (dy_bola / dist_inimigo_bola) * 0.4
            else:
                alvo_x = inimigo_perigoso.pos.x
                alvo_y = inimigo_perigoso.pos.y
        else:
            # Fallback (Se não tiver inimigos no simulador): Fica do lado da área
            alvo_x = -4.5
            alvo_y = -1.0

        # Navega com APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # Gira encarando a bola
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        if abs(erro_angular) > 0.05:
            vw = erro_angular * blackboard.controller.kp_angular
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        else:
            vw = 0.0

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=0.0
        )
        return NodeState.RUNNING
    
# ==========================================
# MEIO CAMPISTA
# ==========================================

class ActionMidfieldSupport(Node):
    """
    O Meio-Campista (Volante). Fica entre a zaga e o ataque.
    Acompanha a linha da bola de longe para pegar rebotes e cortar passes.
    """
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        direcao = 1 if blackboard.enemy_goal_x > 0 else -1

        # 1. A Matemática do Posicionamento (O Ponto de Rebote)
        # O meio-campista quer ficar 1.5 metros atrás da linha da bola
        alvo_x = blackboard.ball_pos.x - (direcao * 1.5)

        # 2. As "Cercas" Invisíveis (Geofencing)
        # Ele não pode invadir a área da nossa zaga nem a banheira do ataque
        limite_defesa = blackboard.our_goal_x + (direcao * 2.5)
        limite_ataque = blackboard.enemy_goal_x - (direcao * 2.5)

        # Mantém o X estritamente dentro da zona de meio-campo
        if direcao > 0:
            alvo_x = max(limite_defesa, min(alvo_x, limite_ataque))
        else:
            alvo_x = max(limite_ataque, min(alvo_x, limite_defesa))

        # No eixo Y, ele centraliza a jogada. Acompanha a bola, mas de forma suavizada (* 0.5)
        # Isso garante que ele fique no centro interceptando, em vez de correr para as laterais
        alvo_y = blackboard.ball_pos.y * 0.5

        # 3. Navega com o nosso APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 4. Gira encarando a bola a todo momento (Radar ativo)
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        if abs(erro_angular) > 0.05:
            vw = erro_angular * blackboard.controller.kp_angular
            vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        else:
            vw = 0.0

        # Envia o comando (Deixa o driblador levemente ligado para matar a bola se ela espirrar nele)
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=500.0
        )
        return NodeState.RUNNING


# ==========================================
# MEIO ARMADOR
# ==========================================
class ActionMeiaArmador(Node):
    """Joga logo atrás do atacante (camisa 10), ligando o meio ao ataque."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None: return NodeState.FAILURE
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        
        alvo_x = blackboard.ball_pos.x - (direcao * 1.0)
        alvo_y = blackboard.ball_pos.y * 0.8 # Acompanha a bola de perto

        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, alvo_x, alvo_y, blackboard.obstacles)
        vw = blackboard.controller.kp_angular * ((math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, blackboard.ball_pos.x - blackboard.my_pos.pos.x) - blackboard.my_pos.yaw + math.pi) % (2 * math.pi) - math.pi)
        
        blackboard.action.send_command(robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=max(min(vw, 5.0), -5.0), kick_speed=0.0, dribbler_speed=0.0)
        return NodeState.RUNNING

# ==========================================
# VOLANTE DEFENSIVO
# ==========================================
class ActionVolanteDefensivo(Node):
    """Protege a entrada da nossa área (Cão de guarda)."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None: return NodeState.FAILURE
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        
        # Fica sempre 3 metros à frente do nosso gol
        alvo_x = blackboard.our_goal_x + (direcao * 3.0)
        alvo_y = blackboard.ball_pos.y * 0.5 

        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, alvo_x, alvo_y, blackboard.obstacles)
        vw = blackboard.controller.kp_angular * ((math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, blackboard.ball_pos.x - blackboard.my_pos.pos.x) - blackboard.my_pos.yaw + math.pi) % (2 * math.pi) - math.pi)
        
        blackboard.action.send_command(robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=max(min(vw, 5.0), -5.0), kick_speed=0.0, dribbler_speed=0.0)
        return NodeState.RUNNING

# ==========================================
# LATERAIS
# ==========================================
class ActionLateral(Node):
    """Classe base para os laterais correrem pelos trilhos do campo ."""
    def __init__(self, lado_y):
        super().__init__()
        self.lado_y = lado_y # Recebe a coordenada Y do trilho

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None: return NodeState.FAILURE
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        
        alvo_y = self.lado_y
        
        # Acompanha o X da bola, mas não passa do meio do campo de defesa nem da entrada da área inimiga
        limite_ataque = blackboard.enemy_goal_x - (direcao * 2.0)
        limite_defesa = blackboard.our_goal_x + (direcao * 4.0)
        alvo_x = blackboard.ball_pos.x
        
        if direcao > 0: alvo_x = max(limite_defesa, min(alvo_x, limite_ataque))
        else:           alvo_x = max(limite_ataque, min(alvo_x, limite_defesa))

        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, alvo_x, alvo_y, blackboard.obstacles)
        vw = blackboard.controller.kp_angular * ((math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, blackboard.ball_pos.x - blackboard.my_pos.pos.x) - blackboard.my_pos.yaw + math.pi) % (2 * math.pi) - math.pi)
        
        blackboard.action.send_command(robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=max(min(vw, 5.0), -5.0), kick_speed=0.0, dribbler_speed=0.0)
        return NodeState.RUNNING
    

# ==========================================
# Parede bola parada
# ==========================================
class ActionFormDefensiveWall(Node):
    """
    Forma uma barreira perfeitamente alinhada entre a bola e o nosso gol.
    Respeita a regra da SSL de manter no mínimo 500mm (0.5m) de distância.
    """
    def __init__(self, offset_lateral=0.0):
        super().__init__()
        # 0.0 (Centro), positivo (Direita), negativo (Esquerda)
        self.offset_lateral = offset_lateral 

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. Reta da bola em direção ao centro do NOSSO gol
        dx = blackboard.our_goal_x - blackboard.ball_pos.x
        dy = 0.0 - blackboard.ball_pos.y
        dist_pro_gol = math.hypot(dx, dy)

        if dist_pro_gol == 0: dist_pro_gol = 1

        # Vetor unitário apontando da bola pro nosso gol
        nx = dx / dist_pro_gol
        ny = dy / dist_pro_gol

        # 2. O Centro da Barreira (Fica a 60cm da bola para garantir a regra)
        centro_x = blackboard.ball_pos.x + nx * 0.90
        centro_y = blackboard.ball_pos.y + ny * 0.90

        # 3. Espalhamento Lateral (Vetor perpendicular)
        # Se o vetor frontal é (nx, ny), o perpendicular é (-ny, nx)
        perp_x = -ny
        perp_y = nx

        # Aplica o espaçamento para criar o "muro"
        alvo_x = centro_x + (perp_x * self.offset_lateral)
        alvo_y = centro_y + (perp_y * self.offset_lateral)

        # 4. Navegação usando nosso APF
        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 5. Gira cravado encarando a bola para preparar o rebote
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=0.0
        )
        return NodeState.RUNNING
    
class ActionMarkEnemy(Node):
    """Marcação individual: Escolhe o N-ésimo inimigo mais perigoso e gruda nele."""
    def __init__(self, indice_inimigo):
        super().__init__()
        self.indice_inimigo = indice_inimigo

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # Fallback: Fica parado onde está se não achar o inimigo
        alvo_x = blackboard.my_pos.pos.x
        alvo_y = blackboard.my_pos.pos.y

        if hasattr(blackboard, 'enemies'):
            # Filtra inimigos válidos (ignora fantasmas)
            inimigos_validos = [e for e in blackboard.enemies if (abs(e.pos.x) > 0.001 or abs(e.pos.y) > 0.001)]
            
            # Ordena pelo perigo (quem está mais perto do nosso gol)
            inimigos_validos.sort(key=lambda e: math.hypot(e.pos.x - blackboard.our_goal_x, e.pos.y - 0.0))
            
            if len(inimigos_validos) > self.indice_inimigo:
                inimigo_alvo = inimigos_validos[self.indice_inimigo]
                
                # Posiciona-se 45cm à frente do inimigo, cortando a linha direta de passe da bola
                dx_bola = blackboard.ball_pos.x - inimigo_alvo.pos.x
                dy_bola = blackboard.ball_pos.y - inimigo_alvo.pos.y
                dist_inimigo_bola = math.hypot(dx_bola, dy_bola)
                
                if dist_inimigo_bola > 0:
                    alvo_x = inimigo_alvo.pos.x + (dx_bola / dist_inimigo_bola) * 0.45
                    alvo_y = inimigo_alvo.pos.y + (dy_bola / dist_inimigo_bola) * 0.45

        # PROTEÇÃO RIGOROSA DA REGRA DA SSL (Garante no mínimo 70cm da bola)
        dist_bola = math.hypot(blackboard.ball_pos.x - alvo_x, blackboard.ball_pos.y - alvo_y)
        if dist_bola < 0.70:
            dx_fuga = alvo_x - blackboard.ball_pos.x
            dy_fuga = alvo_y - blackboard.ball_pos.y
            angle = math.atan2(dy_fuga, dx_fuga) if dist_bola > 0.001 else 0.0
            alvo_x = blackboard.ball_pos.x + math.cos(angle) * 0.70
            alvo_y = blackboard.ball_pos.y + math.sin(angle) * 0.70

        # Navega com APF
        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # Gira encarando a bola a todo momento (Radar ativo)
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, 
                                  blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)

        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=0.0
        )
        return NodeState.RUNNING
    
class ActionPositionForShortPass(Node):
    """Fica a uma distância fixa da bola, abrindo um ângulo nas costas da barreira."""
    def __init__(self, angulo_graus, distancia=1.5):
        super().__init__()
        self.angulo_rad = math.radians(angulo_graus)
        self.distancia = distancia

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None: return NodeState.FAILURE
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        
        # Cria um arco em volta da bola. 0 graus aponta para o nosso gol de defesa.
        angulo_base = math.atan2(0.0 - blackboard.ball_pos.y, blackboard.our_goal_x - blackboard.ball_pos.x)
        angulo_final = angulo_base + self.angulo_rad
        
        alvo_x = blackboard.ball_pos.x + (math.cos(angulo_final) * self.distancia)
        alvo_y = blackboard.ball_pos.y + (math.sin(angulo_final) * self.distancia)

        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, alvo_x, alvo_y, blackboard.obstacles)
        
        # Gira para olhar para a bola, pronto para o passe (Domínio magnético)
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=max(min(vw, 5.0), -5.0),
            kick_speed=0.0, dribbler_speed=1500.0 # Já deixa o driblador ligado!
        )
        return NodeState.RUNNING

class ActionPositionForCross(Node):
    """Invade a entrada da grande área adversária esperando o cruzamento."""
    def __init__(self, y_offset):
        super().__init__()
        self.y_offset = y_offset

    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None: return NodeState.FAILURE
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        
        # Fica plantado a 2.5 metros da linha do gol inimigo
        alvo_x = blackboard.enemy_goal_x - (direcao * 2.5)
        alvo_y = self.y_offset

        vf, vl, _ = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, alvo_x, alvo_y, blackboard.obstacles)
        
        # Gira acompanhando a bola
        target_angle = math.atan2(blackboard.ball_pos.y - blackboard.my_pos.pos.y, blackboard.ball_pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=max(min(vw, 5.0), -5.0),
            kick_speed=0.0, dribbler_speed=1500.0
        )
        return NodeState.RUNNING
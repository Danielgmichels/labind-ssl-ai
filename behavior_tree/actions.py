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


class ActionDribbleToGoal(Node):
    """Conduz a bola em direção ao ataque com o driblador ligado em potência máxima."""
    def tick(self, blackboard):
        if blackboard.my_pos is None:
            return NodeState.FAILURE

        gol_inimigo_x = blackboard.enemy_goal_x
        gol_inimigo_y = 0.0

        # Usa o APF para guiar a bola até o gol, desviando de bloqueios[cite: 1]
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            gol_inimigo_x, gol_inimigo_y, blackboard.obstacles
        )
        
        # Pode aplicar um limitador aqui se o robô estiver perdendo a bola ao correr demais
        
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=0.0, dribbler_speed=1500.0 # Driblador grudando a bola[cite: 1]
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
    Corre para a trave oposta de onde a bola está, criando opção de passe.
    O robô fica estacionado na beirada da área, sempre encarando a bola.
    """
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. A Matemática do Espaço Vazio (Cross-Field)
        # Se a bola está na parte de cima do campo, vamos para a parte de baixo
        if blackboard.ball_pos.y > 0:
            alvo_y = -2.5 
        else:
            alvo_y = 2.5  
        
        # Descobre a direção (1 se positivo, -1 se negativo)
        direcao = 1 if blackboard.enemy_goal_x > 0 else -1
        # O X será fixado na linha imaginária pouco antes da área inimiga
        alvo_x = blackboard.enemy_goal_x - (direcao * 1.5) 

        
        # 2. Navegação com APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 3. Gira como um radar: Bico SEMPRE apontado para a bola
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
        # Se chegou a 30cm do ponto alvo, zera a velocidade para não ficar tremendo
        dist_alvo = math.hypot(alvo_x - blackboard.my_pos.pos.x, alvo_y - blackboard.my_pos.pos.y)
        if dist_alvo < 0.3:
            vf = 0.0
            vl = 0.0

        # Envia comando com o driblador ligado por segurança (se a bola sobrar, ele gruda!)
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
        
        # 1. Mira cravada no peito do companheiro
        target_angle = math.atan2(alvo.pos.y - blackboard.my_pos.pos.y, 
                                  alvo.pos.x - blackboard.my_pos.pos.x)
        erro_angular = target_angle - blackboard.my_pos.yaw
        erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
        
        vw = erro_angular * blackboard.controller.kp_angular
        vw = max(min(vw, blackboard.controller.max_angular_vel), -blackboard.controller.max_angular_vel)
        
        # Mantém a posse enquanto gira
        raio_do_robo = 0.09 
        vl = vw * raio_do_robo 
        vf = 0.5 
        velocidade_chute = 0.0
        
        # 2. Se alinhou, chuta!
        if abs(erro_angular) < 0.1:
            vw = 0.0
            vl = 0.0
            
            # A Matemática da Força:
            # Calcula a distância exata em metros entre os dois robôs
            dist_passe = math.hypot(alvo.pos.x - blackboard.my_pos.pos.x, alvo.pos.y - blackboard.my_pos.pos.y)
            
            # Multiplicar a distância por 1.8 gera uma velocidade de chute (m/s) 
            # proporcional, permitindo que a bola chegue firme, mas dominável.
            velocidade_chute = min(dist_passe * 1.8, 6.0) # Trava o máximo em 6.0 m/s
            vf = 1.0 # Leve tranco pra frente
            
        blackboard.action.send_command(
            robot_id=blackboard.my_id, v_forward=vf, v_left=vl, vw=vw,
            kick_speed=velocidade_chute, dribbler_speed=1500.0
        )
        return NodeState.RUNNING
    
# ==========================================
# GOLEIRO
# ==========================================
class ActionDefendGoal(Node):
    """Fica na linha do gol cortando o ângulo entre a bola e o centro do gol."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. Linha de atuação (Fica um pouco à frente do gol)
        direcao = 1 if blackboard.our_goal_x > 0 else -1
        alvo_x = blackboard.our_goal_x - (direcao * 0.2) 
        
        # 2. A Mágica Matemática: Reta da Bola até o Centro do Nosso Gol
        dx = blackboard.our_goal_x - blackboard.ball_pos.x
        dy = 0.0 - blackboard.ball_pos.y
        
        if abs(dx) > 0.001:
            m = dy / dx
            alvo_y = blackboard.ball_pos.y + m * (alvo_x - blackboard.ball_pos.x)
        else:
            alvo_y = 0.0
            
        # 2. Trava de Segurança: Não deixa o goleiro sair debaixo das traves
        alvo_y = max(-0.6, min(alvo_y, 0.6))
        
        # 3. Navega com APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 4. Gira encarando a bola
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
    """Zagueiro que forma a parede. Fica em um arco de 1.3 metros ao redor do gol."""
    def tick(self, blackboard):
        if blackboard.my_pos is None or blackboard.ball_pos is None:
            return NodeState.FAILURE

        # 1. A Matemática do Arco: Calcula o ângulo da bola até o nosso gol (-6.0, 0.0)
        dx = blackboard.ball_pos.x - blackboard.our_goal_x
        dy = blackboard.ball_pos.y - 0.0
        angulo_bola = math.atan2(dy, dx)
        
        # O Raio de 1.3 metros mantém o zagueiro de forma segura fora do Geofencing da área
        raio_zaga = 2 
        
        # 2. Ponto alvo no arco
        alvo_x = blackboard.our_goal_x + 2.0 * math.cos(angulo_bola)
        alvo_y = 0.0 + raio_zaga * math.sin(angulo_bola)
        
        # Navega com APF
        vf, vl, vw = blackboard.controller.calculate_velocity(
            blackboard.my_pos.pos.x, blackboard.my_pos.pos.y, blackboard.my_pos.yaw, 
            alvo_x, alvo_y, blackboard.obstacles
        )
        
        # 3. Gira encarando a bola para preparar o rebote
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
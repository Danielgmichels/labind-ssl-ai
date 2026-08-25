import math
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
            
            papel_antigo = last_roles.get(r_id, "")
            if papel_antigo == "ATACANTE_REBOTE":    dist_gol_inimigo -= 2.5 
            elif papel_antigo == "ATACANTE_APOIO_ESQ":   dist_gol_inimigo -= 2.5 
            elif papel_antigo == "ATACANTE_APOIO_DIR":   dist_gol_inimigo -= 2.0 
            elif papel_antigo == "MEIA_ARMADOR":     dist_gol_inimigo -= 1.5
            elif papel_antigo == "LATERAL_ESQUERDO": dist_gol_inimigo -= 1.0 
            elif papel_antigo == "LATERAL_DIREITO":  dist_gol_inimigo -= 1.0 
            elif papel_antigo == "MEIO_CAMPO":       dist_gol_inimigo -= 0.5 
            elif papel_antigo == "VOLANTE":          dist_gol_inimigo += 0.5 
            elif papel_antigo == "ZAGUEIRO_MARCACAO":dist_gol_inimigo += 1.5 
            elif papel_antigo == "ZAGUEIRO_BLOQUEIO":dist_gol_inimigo += 2.5 
                
            sobra.append((dist_gol_inimigo, r_id))

            
    # Ordena: [0] é o mais perto do gol inimigo, [-1] é o mais perto do nosso gol
    sobra.sort(key=lambda x: x[0])
    
    # ---------------------------------------------------------
    # 1. LINHA DE FRENTE (Os 3 mais avançados com Consciência de Y)
    # ---------------------------------------------------------
    frente = []
    for _ in range(min(3, len(sobra))):
        frente.append(sobra.pop(0))
        
    if frente:
        # Ordena os 3 separados pelo eixo Y (do maior/Esquerda para o menor/Direita)
        frente.sort(key=lambda item: next((r.pos.y for r in team_robots if getattr(r, 'id', -1) == item[1]), 0.0), reverse=True)
        
        if len(frente) == 3:
            papeis[frente[0][1]] = "ATACANTE_APOIO_ESQ"
            papeis[frente[1][1]] = "MEIA_ARMADOR"
            papeis[frente[2][1]] = "ATACANTE_APOIO_DIR"
        elif len(frente) == 2:
            papeis[frente[0][1]] = "ATACANTE_APOIO_ESQ"
            papeis[frente[1][1]] = "ATACANTE_APOIO_DIR"
        elif len(frente) == 1:
            papeis[frente[0][1]] = "MEIA_ARMADOR"

    # ---------------------------------------------------------
    # 2. RETRANCA (Os 3 mais recuados)
    # ---------------------------------------------------------
    if len(sobra) > 0: papeis[sobra.pop(-1)[1]] = "ZAGUEIRO_BLOQUEIO" 
    if len(sobra) > 0: papeis[sobra.pop(-1)[1]] = "ZAGUEIRO_MARCACAO" 
    if len(sobra) > 0: papeis[sobra.pop(-1)[1]] = "VOLANTE"
        
    # ---------------------------------------------------------
    # 3. MIOLO DO CAMPO E LATERAIS (Os que restaram no meio)
    # ---------------------------------------------------------
    miolo = []
    for _ in range(min(3, len(sobra))):
         miolo.append(sobra.pop(0))
         
    if miolo:
         # Ordena pelo eixo Y para garantir que ninguém cruze o campo
         miolo.sort(key=lambda item: next((r.pos.y for r in team_robots if getattr(r, 'id', -1) == item[1]), 0.0), reverse=True)
         
         if len(miolo) == 3:
             papeis[miolo[0][1]] = "LATERAL_ESQUERDO"
             papeis[miolo[1][1]] = "MEIO_CAMPO"
             papeis[miolo[2][1]] = "LATERAL_DIREITO"
         elif len(miolo) == 2:
             papeis[miolo[0][1]] = "LATERAL_ESQUERDO"
             papeis[miolo[1][1]] = "LATERAL_DIREITO"
         elif len(miolo) == 1:
             papeis[miolo[0][1]] = "MEIO_CAMPO"
        
    for s in sobra:
        papeis[s[1]] = "ESPERA"
        
    return papeis
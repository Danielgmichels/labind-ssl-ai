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
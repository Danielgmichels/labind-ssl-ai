from behavior_tree import *

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
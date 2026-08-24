from behavior_tree import *

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

def build_meio_campo_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Fixado no centro a 3.5 metros
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(0)]),
        Sequence([valendo_ou_nossa_falta, ActionMidfieldSupport()]), 
        ActionStopMotors() 
    ])

def build_meia_armador_tree():
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]), 
        # NOVO: Centro-ofensivo recuado 2.5m
        Sequence([qualquer_kickoff, ActionPositionForKickoff(2.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(3)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForShortPass(-45.0, 1.5)]),
        Sequence([ConditionIsGameRunning(), ActionMeiaArmador()]), 
        ActionStopMotors()
    ])


# O Y vai ser 3.5 (esquerda) e -3.5 (direita) para correr perto da borda
def build_lateral_tree(lado_y, indice_marcacao):
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]), 
        # NOVO: Bem abertos nas laterais a 3 metros
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.0, lado_y)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(indice_marcacao)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForCross(lado_y)]),
        Sequence([ConditionIsGameRunning(), ActionLateral(lado_y)]), 
        ActionStopMotors()
    ])
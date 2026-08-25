from behavior_tree import *

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

def build_zaga_bloqueio_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Zaga se abre na frente da área (Y=1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(5.0, 1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.0)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroBloqueio()]), 
        ActionStopMotors() 
    ])

def build_zaga_marcacao_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Zaga se abre na frente da área (Y=-1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(5.0, -1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.18)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroMarcacao()]),
        ActionStopMotors() 
    ])

def build_volante_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Cão de guarda fica a 4.5 metros (quase na área)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(4.5, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=-0.18)]),
        Sequence([valendo_ou_nossa_falta, ActionVolanteDefensivo()]),
        ActionStopMotors() 
    ])

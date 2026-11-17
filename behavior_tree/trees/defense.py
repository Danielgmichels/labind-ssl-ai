from behavior_tree import *

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

def build_zaga_bloqueio_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # Zaga se abre com segurança à frente da grande área (X=3.8, Y=1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.8, 1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.0)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroBloqueio()]), 
        ActionStopMotors() 
    ])

def build_zaga_marcacao_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # Zaga se abre com segurança à frente da grande área (X=3.8, Y=-1.5)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.8, -1.5)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=0.18)]),
        Sequence([valendo_ou_nossa_falta, ActionZagueiroMarcacao()]),
        ActionStopMotors() 
    ])

def build_volante_tree():
    valendo_ou_nossa_falta = Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()])
    receber_passe = Sequence([ConditionIsPassArriving(), ActionInterceptPass()])
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # Volante fica posicionado a 3.2m do meio de campo (seguro fora da grande área)
        Sequence([qualquer_kickoff, ActionPositionForKickoff(3.2, 0.0)]),
        Sequence([ConditionIsEnemyFreeKick(), ActionFormDefensiveWall(offset_lateral=-0.18)]),
        Sequence([valendo_ou_nossa_falta, Selector([receber_passe, ActionVolanteDefensivo()])]),
        ActionStopMotors() 
    ])

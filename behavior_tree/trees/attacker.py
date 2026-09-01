from behavior_tree import *

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

def build_attacker_tree():
    ramo_emergencia = Sequence([ConditionIsHalted(), ActionStopMotors()])
    
    # O Atacante é o único com comportamento diferente
    ramo_kickoff_nosso = Sequence([ConditionIsOurPrepareKickoff(), ActionPrepareKickoff()])
    # Na saída do inimigo, fica a 80cm da bola, perfeitamente fora do círculo central de 50cm!
    ramo_kickoff_deles = Sequence([ConditionIsEnemyPrepareKickoff(), ActionPositionForKickoff(0.8, 0.0)])

    cobrar_falta = Sequence([ # (Seu bloco cobrar_falta antigo continua igualzinho aqui) ...
        ConditionIsOurFreeKick(),
        Selector([
            Sequence([ConditionIsNearBall(), ConditionIsPassClear(), ActionPassBall()]),
            Sequence([ConditionIsNearBall(), ActionCrossToBox()]), 
            ActionGoToBall() 
        ])
    ])
    
    receber_passe = Sequence([ConditionIsPassArriving(), ActionInterceptPass()])
    tentar_finalizar = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ConditionEvaluateShot(), ActionAimAndShoot()])
    tentar_passe = Sequence([ConditionIsNearBall(), ConditionIsPassClear(), ActionPassBall()])
    achar_angulo = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ActionFindShootingAngle()])
    tentar_conduzir = Sequence([ConditionIsNearBall(), ActionSmartDribble()])
    buscar_bola = ActionGoToBall()
    
    ramo_ofensivo = Sequence([
        Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()]),
        Selector([receber_passe, tentar_finalizar, tentar_passe, achar_angulo, tentar_conduzir, buscar_bola])
    ])

    return Selector([ramo_emergencia, ramo_kickoff_nosso, ramo_kickoff_deles, Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(6)]), cobrar_falta, ramo_ofensivo])

def build_atacante_apoio_tree(lado_y, indice_marcacao, angulo_falta):
    receber_passe = Sequence([ConditionIsPassArriving(), ActionInterceptPass()])
    ramo_ofensivo = Sequence([ConditionIsGameRunning(), Selector([receber_passe, ActionPositionForPass(lado_y)])])
    
    return Selector([
        Sequence([ConditionIsHalted(), ActionStopMotors()]),
        # NOVO: Espaçados 1.5 metros para trás, abertos nas alas
        Sequence([qualquer_kickoff, ActionPositionForKickoff(1.5, lado_y)]), 
        Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(indice_marcacao)]),
        Sequence([ConditionIsOurFreeKick(), ActionPositionForShortPass(angulo_falta, 1.5)]),
        ramo_ofensivo,
        ActionStopMotors() 
    ])
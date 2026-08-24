from behavior_tree import *

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
    tentar_finalizar = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ConditionIsPathClear(), ActionAimAndShoot()])
    tentar_passe = Sequence([ConditionIsNearBall(), ConditionIsPassClear(), ActionPassBall()])
    achar_angulo = Sequence([ConditionIsNearBall(), ConditionIsInShootingZone(), ActionFindShootingAngle()])
    tentar_conduzir = Sequence([ConditionIsNearBall(), ActionSmartDribble()])
    buscar_bola = ActionGoToBall()
    
    ramo_ofensivo = Sequence([
        Selector([ConditionIsGameRunning(), ConditionIsOurFreeKick()]),
        Selector([receber_passe, tentar_finalizar, tentar_passe, achar_angulo, tentar_conduzir, buscar_bola])
    ])

    return Selector([ramo_emergencia, ramo_kickoff_nosso, ramo_kickoff_deles, Sequence([ConditionIsEnemyFreeKick(), ActionMarkEnemy(6)]), cobrar_falta, ramo_ofensivo])

# --- PARA O RESTO DO TIME, O KICKOFF É UMA POSIÇÃO FIXA E ESPAÇADA ---
qualquer_kickoff = Selector([ConditionIsOurPrepareKickoff(), ConditionIsEnemyPrepareKickoff()])

# Agora o Apoio recebe o lado, o índice de marcação (falta inimiga) e o ângulo (nossa falta)
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

def build_espera_tree():
    """Árvore genérica para robôs ociosos: apenas desliga os motores."""
    return ActionStopMotors()

def build_master_tree():
    """Cria uma Árvore de Comportamento Mestra que engloba todo o time."""
    return Selector([
        Sequence([ConditionCheckRole("GOLEIRO"), build_goleiro_tree()]),
        Sequence([ConditionCheckRole("ATACANTE"), build_attacker_tree()]),
        Sequence([ConditionCheckRole("ATACANTE_APOIO_ESQ"), build_atacante_apoio_tree(2.5, 4, 45.0)]),
        Sequence([ConditionCheckRole("ATACANTE_APOIO_DIR"), build_atacante_apoio_tree(-2.5, 5, -45.0)]),
        Sequence([ConditionCheckRole("MEIA_ARMADOR"), build_meia_armador_tree()]),
        Sequence([ConditionCheckRole("MEIO_CAMPO"), build_meio_campo_tree()]),
        Sequence([ConditionCheckRole("VOLANTE"), build_volante_tree()]),
        Sequence([ConditionCheckRole("LATERAL_ESQUERDO"), build_lateral_tree(3.5, 1)]),
        Sequence([ConditionCheckRole("LATERAL_DIREITO"), build_lateral_tree(-3.5, 2)]),
        Sequence([ConditionCheckRole("ZAGUEIRO_BLOQUEIO"), build_zaga_bloqueio_tree()]),
        Sequence([ConditionCheckRole("ZAGUEIRO_MARCACAO"), build_zaga_marcacao_tree()]),
        Sequence([ConditionCheckRole("ESPERA"), build_espera_tree()])
    ])
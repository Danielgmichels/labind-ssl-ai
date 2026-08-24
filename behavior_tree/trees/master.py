from behavior_tree import *
from .attacker import *
from .defense import *
from .goalkeeper import *
from .support import *

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
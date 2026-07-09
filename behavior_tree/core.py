from enum import Enum

# ==========================================
# ESTADOS DOS NÓS
# ==========================================
class NodeState(Enum):
    SUCCESS = 1   # Ação concluída com sucesso ou Condição verdadeira
    FAILURE = 2   # Ação falhou ou Condição falsa
    RUNNING = 3   # Ação está em andamento (muito importante para robótica!)

# ==========================================
# CLASSE BASE GENÉRICA
# ==========================================
class Node:
    """Classe base para todos os nós da Behavior Tree."""
    def tick(self, blackboard):
        """
        Método que será chamado a cada ciclo de 60Hz.
        Recebe o Blackboard (Quadro Negro) para ler sensores e enviar comandos.
        Deve retornar um NodeState (SUCCESS, FAILURE ou RUNNING).
        """
        raise NotImplementedError("Todo nó filho deve implementar o método tick()")

# ==========================================
# NÓS COMPOSTOS (COMPOSITES)
# ==========================================
class Composite(Node):
    """Classe base para nós que contêm filhos (Selector, Sequence)."""
    def __init__(self, children=None):
        self.children = children if children is not None else []

    def add_child(self, child):
        self.children.append(child)

# ==========================================
# SELECTOR (O "OU" Lógico / Fallback)
# ==========================================
class Selector(Composite):
    """
    Tenta executar os filhos em ordem.
    - Se um filho retornar SUCCESS, o Selector retorna SUCCESS na hora.
    - Se um filho retornar RUNNING, o Selector retorna RUNNING na hora.
    - Se um filho retornar FAILURE, ele tenta o PRÓXIMO filho.
    - Se TODOS falharem, ele retorna FAILURE.
    """
    def tick(self, blackboard):
        for child in self.children:
            status = child.tick(blackboard)
            
            # Se deu Sucesso ou está Rodando, para por aqui e repassa o status pra cima
            if status != NodeState.FAILURE:
                return status
                
        # Se o for terminar, significa que todos os filhos retornaram FAILURE
        return NodeState.FAILURE

# ==========================================
# SEQUENCE (O "E" Lógico)
# ==========================================
class Sequence(Composite):
    """
    Executa os filhos em ordem rigorosa.
    - Se um filho retornar FAILURE, a Sequence quebra e retorna FAILURE na hora.
    - Se um filho retornar RUNNING, a Sequence retorna RUNNING na hora.
    - Se um filho retornar SUCCESS, ele passa para o PRÓXIMO filho.
    - Se TODOS derem sucesso, ele retorna SUCCESS.
    """
    def tick(self, blackboard):
        for child in self.children:
            status = child.tick(blackboard)
            
            # Se Falhou ou está Rodando, para por aqui e repassa o status pra cima
            if status != NodeState.SUCCESS:
                return status
                
        # Se o for terminar, significa que todos os filhos retornaram SUCCESS
        return NodeState.SUCCESS
  
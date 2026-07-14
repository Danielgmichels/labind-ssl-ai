import os
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

def export_tree_to_dot(node, filename="arvore_tatica.dot"):
    """Gera um arquivo Graphviz com um tema escuro e paleta personalizada."""
    def _write_dot_node(current_node, file):
        node_id = id(current_node)
        name = current_node.__class__.__name__

        # Define a cor baseada no tipo do nó (A Mágica das Cores)
        if name.startswith('Action'):
            # Ações (O "músculo" do robô): Fundo Vermelho Vinho
            fillcolor = "#722F37"
            fontcolor = "white"
        elif name.startswith('Condition'):
            # Condições (O "radar"): Preto mais clarinho / Cinza Escuro
            fillcolor = "#2d2d2d"
            fontcolor = "white"
        else:
            # Nós de Controle (Selector / Sequence): Preto puro
            fillcolor = "#121212"
            fontcolor = "white"

        # Desenha a caixinha do nó com bordas arredondadas
        file.write(f'    "{node_id}" [label="{name}", fillcolor="{fillcolor}", fontcolor="{fontcolor}"];\n')

        # Se tiver filhos, desenha as setas conectando
        if hasattr(current_node, 'children'):
            for child in current_node.children:
                child_id = id(child)
                file.write(f'    "{node_id}" -> "{child_id}" [color="#888888"];\n')
                _write_dot_node(child, file)

    diretorio = os.path.dirname(filename)
    if diretorio: # Só tenta criar se houver uma pasta especificada
        os.makedirs(diretorio, exist_ok=True)

    # Abre o arquivo e escreve a estrutura do Graphviz
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("digraph BehaviorTree {\n")
        # Configurações do fundo escuro
        f.write('    bgcolor="#1a1a1a";\n')
        f.write('    node [shape=box, style="filled,rounded", fontname="Arial", color="#000000", penwidth=1];\n')
        _write_dot_node(node, f)
        f.write("}\n")
        
    print(f"Árvore exportada com sucesso para {filename}!")
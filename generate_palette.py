import os
import sys
import inspect
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

# Adiciona o diretório do projeto ao path para permitir a importação dos módulos da BT
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CURRENT_DIR)

from behavior_tree.core import Node, Composite
from behavior_tree import actions, conditions

def generate_groot_palette():
    """
    Inspeciona os nós de ação e condição do projeto e gera uma paleta XML
    customizada para ser usada no editor Groot.
    """
    root = Element('root')
    model = SubElement(root, 'TreeNodesModel')

    # Encontra todas as classes de nós nos módulos de actions e conditions
    all_node_classes = []
    for module in [actions, conditions]:
        for name, obj in inspect.getmembers(module):
            # Garante que é uma classe, herda de Node, mas não é o Node base nem um Composite
            if inspect.isclass(obj) and issubclass(obj, Node) and obj not in [Node, Composite]:
                all_node_classes.append(obj)

    for node_class in sorted(all_node_classes, key=lambda x: x.__name__):
        class_name = node_class.__name__
        
        # Determina o tipo do nó para o Groot (Action, Condition, etc.)
        if class_name.startswith('Action'):
            node_type = 'Action'
        elif class_name.startswith('Condition'):
            node_type = 'Condition'
        else:
            continue # Ignora outros tipos por enquanto

        # Cria o elemento XML principal para o nó
        xml_node = SubElement(model, node_type, {'ID': class_name})

        # Inspeciona o __init__ da classe para encontrar parâmetros e criar "input_port" no XML
        try:
            sig = inspect.signature(node_class.__init__)
            for param in sig.parameters.values():
                # Ignora 'self' e os genéricos *args, **kwargs
                if param.name not in ['self', 'args', 'kwargs']:
                    SubElement(xml_node, 'input_port', {'name': param.name})
        except (ValueError, TypeError):
            # Algumas classes podem não ter um __init__ customizado, o que é normal
            pass

    # Formata o XML para ficar legível
    xml_str = tostring(root, 'utf-8')
    pretty_xml_str = minidom.parseString(xml_str).toprettyxml(indent="  ")

    # Salva o arquivo na pasta de diagramas
    palette_path = os.path.join(CURRENT_DIR, 'diagramas', 'nodes_palette.xml')
    os.makedirs(os.path.dirname(palette_path), exist_ok=True)
    with open(palette_path, 'w', encoding='utf-8') as f:
        f.write(pretty_xml_str)
        
    print(f"Palette XML para o Groot gerada com sucesso em: {palette_path}")

if __name__ == "__main__":
    generate_groot_palette()

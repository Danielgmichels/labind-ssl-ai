from typing import Dict, Type, Any
from navigation.interfaces import IPathPlanner

_PLANNER_REGISTRY: Dict[str, Type[IPathPlanner]] = {}


def register_path_planner(name: str):
    """Decorator para registrar dinamicamente classes de planejadores na fábrica."""
    def decorator(cls: Type[IPathPlanner]):
        _PLANNER_REGISTRY[name.upper()] = cls
        return cls
    return decorator


def get_path_planner(planner_type: str, **kwargs) -> IPathPlanner:
    """
    Retorna uma instância do algoritmo de planejamento solicitado.

    Algoritmos suportados na pesquisa:
    - "C_VG": Grafo de Visibilidade Circular + A* Lazy
    - "VG_ASTAR": Grafo de Visibilidade Poligonal + A*
    - "D_STAR_LITE": Grafo de Visibilidade Poligonal + D* Lite incremental
    - "RRT_STAR": RRT* com viés de meta e passo adaptativo
    - "APF_HYBRID": Campo Potencial Artificial com escape de mínimos locais

    :param planner_type: Nome identificador do algoritmo.
    :param kwargs: Parâmetros opcionais de inicialização do planejador.
    :return: Instância de IPathPlanner.
    """
    normalized = planner_type.strip().upper()

    # Tentativa de importação tardia (lazy load) dos módulos à medida que forem implementados
    if normalized not in _PLANNER_REGISTRY:
        _try_lazy_imports(normalized)

    if normalized not in _PLANNER_REGISTRY:
        supported = list(_PLANNER_REGISTRY.keys())
        raise NotImplementedError(
            f"Planejador '{planner_type}' ainda não implementado ou registrado. "
            f"Planejadores registrados atualmente: {supported if supported else 'Nenhum'}"
        )

    planner_cls = _PLANNER_REGISTRY[normalized]
    return planner_cls(**kwargs)


def _try_lazy_imports(normalized_name: str):
    """Tenta carregar dinamicamente o arquivo do planejador caso exista."""
    try:
        if normalized_name == "C_VG":
            import navigation.planners.c_vg  # noqa: F401
        elif normalized_name == "VG_ASTAR":
            import navigation.planners.vg_astar  # noqa: F401
        elif normalized_name == "D_STAR_LITE":
            import navigation.planners.d_star_lite  # noqa: F401
        elif normalized_name == "RRT_STAR":
            import navigation.planners.rrt_star  # noqa: F401
        elif normalized_name == "APF_HYBRID":
            import navigation.planners.apf_hybrid  # noqa: F401
    except (ImportError, ModuleNotFoundError):
        pass

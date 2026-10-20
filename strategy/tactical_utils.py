from dataclasses import dataclass, field
from typing import List, Optional
from typing import List, Optional, Tuple

# --- Configurações Centralizadas ---
TACTICAL_CONFIG = {
    "SHOT": {
        "candidate_targets_y": [-0.45, -0.38, -0.30, -0.20, -0.10, 0.00, 0.10, 0.20, 0.30, 0.38, 0.45],
        "min_score": 0.58,         # Limiar mínimo aceitável para chutar
        "max_distance": 6.0,       # Distância máxima de finalização (m)
        "collision_radius": 0.16,  # Raio de bloqueio físico na linha (m)
        "safe_margin": 0.35,       # Distância livre para bloqueio nulo (m)
        "weights": {
            "opening": 0.25,
            "angle": 0.20,
            "distance": 0.10,
            "progression": 0.15,
            "blocking": 0.25,
            "risk": 0.05
        }
    },
    "PASS": {
        "min_score": 0.55,         # Limiar mínimo aceitável para autorizar o passe (mais crítico)
        "min_distance": 0.70,      # Distância mínima viável (m)
        "max_distance": 6.00,      # Distância máxima viável (m)
        "collision_radius": 0.20,  # Raio de bloqueio físico na linha de passe (m)
        "safe_margin": 0.50,       # Margem segura livre de interceptação (m)
        "density_radius": 1.20,    # Raio de busca de adversários em volta do receptor (m)
        "weights": {
            "interception": 0.30,  # Prioridade máxima: linha de passe desimpedida
            "space": 0.25,         # Receptor livre de marcação direta
            "progression": 0.20,   # Ganho vertical em direção ao ataque
            "reception_angle": 0.10,# Alinhamento frontal para domínio
            "support": 0.05,       # Preferência tática do papel (peso reduzido para não mascarar marcação)
            "distance": 0.05,      # Faixa operacional confortável
            "density": 0.05        # Densidade de oponentes no entorno
        },
        "role_support_scores": {
            "ATACANTE_APOIO_ESQ": 1.00,
            "ATACANTE_APOIO_DIR": 1.00,
            "MEIA_ARMADOR": 0.90,
            "LATERAL_ESQUERDO": 0.75,
            "LATERAL_DIREITO": 0.75,
            "MEIO_CAMPO": 0.65,
            "VOLANTE": 0.40,
            "ZAGUEIRO_MARCACAO": 0.20,
            "ZAGUEIRO_BLOQUEIO": 0.15,
            "ESPERA": 0.00,
            "GOLEIRO": 0.00
        }
    },
    "FREE_SPACE": {
        "grid_resolution": 0.30,   # Resolução da grade espacial (m)
        "min_score": 0.45,         # Limiar mínimo para aprovação de ponto (mais crítico)
        "stability_bonus": 0.15,   # Bônus para o ponto atual (evita oscilações)
        "switch_margin": 0.05,     # Margem necessária para trocar de ponto
        "safe_margin": 0.45,       # Margem livre de bloqueadores na linha de passe (m)
        "collision_radius": 0.18,  # Raio de bloqueio físico na linha de passe (m)
        "weights": {
            "space": 0.25,
            "pass_line": 0.30,      # Prioridade máxima para linha de passe desimpedida
            "support": 0.15,
            "progression": 0.15,
            "defensive_cover": 0.05, # Não retém excessivamente os apoios de ataque
            "safety": 0.10
        }
    },
    "SHIELD": {
        "danger_radius": 0.75,          # Raio de ativação da proteção por aproximação (m)
        "immediate_threat_radius": 0.50,# Raio de perigo iminente (m)
        "max_shield_duration": 1.50,    # Tempo limite contínuo sob blindagem antes de forçar passe (s)
        "max_shield_distance": 0.80,    # Distância máxima de condução contínua (m, regra de 1m SSL Divisão A)
        "dribbler_speed": 1500.0,       # Rotação do rolete durante a blindagem (RPM)
        "tangent_distance": 0.60,       # Distância projetada para o waypoint de evasão tangencial (m)
        "field_margin_y": 2.60          # Limite de segurança no eixo Y para não sair de campo (m)
    }
}

# --- Estruturas de Dados (Fase 1: Shot Scoring) ---
@dataclass
class ShotCandidate:
    target_y: float
    score: float = 0.0
    opening_score: float = 0.0
    angle_score: float = 0.0
    distance_score: float = 0.0
    progression_score: float = 0.0
    blocking_score: float = 0.0
    risk_score: float = 0.0
    reason: str = ""

@dataclass
class ShotDecision:
    best_target_y: Optional[float] = None
    best_candidate: Optional[ShotCandidate] = None
    all_candidates: List[ShotCandidate] = field(default_factory=list)


# --- Estruturas de Dados (Fase 2: Pass Scoring) ---
@dataclass
class PassCandidate:
    receiver_id: int
    receiver_role: str
    target_point: Tuple[float, float]
    score: float = 0.0
    progression_score: float = 0.0
    space_score: float = 0.0
    reception_angle_score: float = 0.0
    distance_score: float = 0.0
    support_score: float = 0.0
    interception_score: float = 0.0
    density_score: float = 0.0
    reason: str = ""

@dataclass
class PassDecision:
    best_target_robot: Optional[int] = None
    best_target_point: Optional[Tuple[float, float]] = None
    best_candidate: Optional[PassCandidate] = None
    all_candidates: List[PassCandidate] = field(default_factory=list)


# --- Estruturas de Dados (Fase 3: Free-Space Positioning) ---
@dataclass
class PositionCandidate:
    target_point: Tuple[float, float]
    score: float = 0.0
    space_score: float = 0.0
    pass_line_score: float = 0.0
    support_score: float = 0.0
    progression_score: float = 0.0
    defensive_cover_score: float = 0.0
    safety_score: float = 0.0
    reason: str = ""

@dataclass
class PositionDecision:
    best_target_point: Optional[Tuple[float, float]] = None
    best_candidate: Optional[PositionCandidate] = None
    all_candidates: List[PositionCandidate] = field(default_factory=list)
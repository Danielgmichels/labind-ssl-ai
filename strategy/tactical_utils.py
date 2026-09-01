from dataclasses import dataclass, field
from typing import List, Optional

# --- Configurações Centralizadas ---
TACTICAL_CONFIG = {
    "SHOT": {
        "candidate_targets_y": [-0.45, -0.30, -0.15, 0.00, 0.15, 0.30, 0.45],
        "min_score": 0.40,  # Limiar mínimo aceitável para chutar
        "weights": {
            "opening": 0.25,
            "angle": 0.20,
            "distance": 0.15,
            "progression": 0.15,
            "blocking": 0.20,
            "risk": 0.05
        }
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
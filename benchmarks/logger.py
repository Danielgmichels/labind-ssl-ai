import os
import csv
import time
from typing import Optional


class BenchmarkLogger:
    """
    Logger de telemetria a 60 Hz para benchmarks de algoritmos de planejamento de trajetória.
    Grava métricas em formato CSV estruturado para análise estatística e gráficos.
    """

    HEADERS = [
        "timestamp",
        "run_id",
        "algorithm_name",
        "density_level",
        "calc_time_ms",
        "pos_x",
        "pos_y",
        "vel_real_x",
        "vel_real_y",
        "vel_cmd_x",
        "vel_cmd_y",
        "min_obstacle_dist",
        "collision_flag"
    ]

    def __init__(self, output_filepath: str, algorithm_name: str = "UNKNOWN", run_id: int = 0, density_level: int = 0):
        self.output_filepath = output_filepath
        self.algorithm_name = algorithm_name
        self.run_id = run_id
        self.density_level = density_level

        os.makedirs(os.path.dirname(os.path.abspath(output_filepath)), exist_ok=True)
        file_exists = os.path.exists(output_filepath) and os.path.getsize(output_filepath) > 0

        self.file = open(output_filepath, mode="a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)

        if not file_exists:
            self.writer.writerow(self.HEADERS)
            self.file.flush()

    def log_step(
        self,
        calc_time_ms: float,
        pos_x: float,
        pos_y: float,
        vel_real_x: float,
        vel_real_y: float,
        vel_cmd_x: float,
        vel_cmd_y: float,
        min_obstacle_dist: float,
        collision_flag: bool,
        timestamp: Optional[float] = None
    ):
        """Grava uma linha de telemetria do ciclo atual."""
        ts = timestamp if timestamp is not None else time.time()
        self.writer.writerow([
            f"{ts:.4f}",
            self.run_id,
            self.algorithm_name,
            self.density_level,
            f"{calc_time_ms:.4f}",
            f"{pos_x:.4f}",
            f"{pos_y:.4f}",
            f"{vel_real_x:.4f}",
            f"{vel_real_y:.4f}",
            f"{vel_cmd_x:.4f}",
            f"{vel_cmd_y:.4f}",
            f"{min_obstacle_dist:.4f}",
            1 if collision_flag else 0
        ])

    def flush(self):
        """Descarrega os buffers de escrita em disco."""
        if self.file and not self.file.closed:
            self.file.flush()

    def close(self):
        """Fecha o arquivo de log com segurança."""
        if self.file and not self.file.closed:
            self.file.flush()
            self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

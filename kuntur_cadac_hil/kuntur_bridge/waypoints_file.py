"""
waypoints_file.py -- Carga de waypoints estáticos en lat/lon reales (no
NED a mano), convertidos con la MISMA geodesy.SACOLocalFrame ya
calibrada para esta corrida (autoridad única), así que quedan
consistentes con lo que ve FlightGear.

Fork HIL: solo queda `load_waypoints_list()`, el único camino que de
verdad usa cadac_plant/plant_driver.py (--waypoints, para sobrescribir
swel1/swel2/swel3 del *.asc en RealFalcon6Adapter). El avance de
waypoint real lo hace el mecanismo NATIVO de cadac_py (cola de eventos
IF wp_flag=-1 ENDIF, calculado internamente por geometría de radio de
giro) -- esta función solo parsea coordenadas, no implementa ninguna
máquina de avance (ver RealFalcon6Adapter._advance_route_waypoint()).

Formato del archivo (texto plano, una línea por waypoint):
    lat_deg, lon_deg, alt_msl_m[, radio_captura_m]

Líneas vacías y las que empiezan con '#' se ignoran. `capture_radius_m`
queda sin uso en este camino (RealFalcon6Adapter no lo consume), se
acepta por compatibilidad con el formato de scenarios/waypoints_cordoba.txt.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .geodesy import SACOLocalFrame


@dataclass(frozen=True)
class Waypoint:
    north: float
    east: float
    down: float
    capture_radius_m: float = 150.0


def load_waypoints_list(path: str | Path, frame: SACOLocalFrame) -> list[Waypoint]:
    """Parseo puro: archivo -> lista ordenada de Waypoint (NED ya
    convertido)."""
    waypoints: list[Waypoint] = []
    text = Path(path).read_text(encoding="utf-8")
    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) not in (3, 4):
            raise ValueError(
                f"{path}:{line_no}: se esperaban 3 o 4 campos "
                f"(lat_deg, lon_deg, alt_msl_m[, radio_captura_m]), se recibieron {len(parts)}"
            )
        lat_deg, lon_deg, alt_msl_m = (float(p) for p in parts[:3])
        capture_radius_m = float(parts[3]) if len(parts) == 4 else 150.0

        north, east, down = frame.geodetic_to_ned(lat_deg, lon_deg, alt_msl_m)
        waypoints.append(Waypoint(north=north, east=east, down=down, capture_radius_m=capture_radius_m))

    if not waypoints:
        raise ValueError(f"{path}: no se encontró ningún waypoint válido")
    return waypoints

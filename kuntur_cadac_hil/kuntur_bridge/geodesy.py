"""geodesy.py -- Marco local NED -> WGS-84, centrado en el origen SACO.

Única autoridad de esta transformación en todo el sistema (Especificacion_r01.md,
seccion 3: "Autoridad Única -- se preserva geodesy.SACOLocalFrame como
única fuente de verdad para la proyección WGS-84, garantizando que PX4 y
FlightGear utilicen el mismo origen dinámico calibrado"). La S-Function
NO duplica esta proyección -- pos_ned llega crudo (passthrough) y esta
clase es la que lo convierte, para flightgear_worker.py y para
px4_bridge.py por igual, ambos consumiendo la MISMA instancia calibrada.
"""
from __future__ import annotations

import math


class SACOLocalFrame:
    def __init__(self, lat0_deg: float, lon0_deg: float, alt0_m: float):
        self.lat0_rad = math.radians(lat0_deg)
        self.lon0_rad = math.radians(lon0_deg)
        self.alt0_m = alt0_m

        a = 6378137.0
        e2 = 0.00669437999014
        sin_lat = math.sin(self.lat0_rad)
        self.R_M = a * (1 - e2) / ((1 - e2 * sin_lat ** 2) ** 1.5)
        self.R_N = a / math.sqrt(1 - e2 * sin_lat ** 2)

    def ned_to_geodetic(self, north: float, east: float, down: float) -> tuple[float, float, float]:
        """Devuelve (lat_rad, lon_rad, alt_msl_m)."""
        lat_rad = self.lat0_rad + (north / (self.R_M + self.alt0_m))
        lon_rad = self.lon0_rad + (east / ((self.R_N + self.alt0_m) * math.cos(self.lat0_rad)))
        alt_msl = self.alt0_m - down
        return lat_rad, lon_rad, alt_msl

    def geodetic_to_ned(self, lat_deg: float, lon_deg: float, alt_msl_m: float) -> tuple[float, float, float]:
        """Inversa de ned_to_geodetic() -- agregada para el fork CADAC-SIL,
        que necesita expresar waypoints reales (p.ej. puntos alrededor de
        Córdoba) como offsets NED en un input.asc, con el mismo origen y
        la misma aproximación de plano tangente que ya usan
        flightgear_worker.py/px4_bridge.py. No reemplaza ned_to_geodetic()
        como autoridad de proyección -- es su inversa algebraica exacta
        (mismos R_M/R_N calculados una sola vez en el origen), por lo que
        ned_to_geodetic(*geodetic_to_ned(lat, lon, alt)) es identidad
        salvo el error de aproximación de plano tangente ya existente,
        no uno nuevo introducido acá.

        Devuelve (north, east, down) en metros.
        """
        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)
        north = (lat_rad - self.lat0_rad) * (self.R_M + self.alt0_m)
        east = (lon_rad - self.lon0_rad) * (self.R_N + self.alt0_m) * math.cos(self.lat0_rad)
        down = self.alt0_m - alt_msl_m
        return north, east, down

    def matches(self, lat_deg: float, lon_deg: float, tol_deg: float) -> bool:
        """Compara (lat_deg, lon_deg) contra el ORIGEN de este frame (no
        contra un punto arbitrario) -- agregado para el canal KUNTUR_CTL
        (ver icd_schema/kuntur_ic_ctl_v1.json), donde el cliente (GCS) echo
        de vuelta el origen que cree que está usando, y la planta rechaza
        el comando si no coincide con el suyo (config/calibración
        desincronizada entre ambos lados, no un punto objetivo lejano)."""
        return (
            abs(lat_deg - math.degrees(self.lat0_rad)) <= tol_deg
            and abs(lon_deg - math.degrees(self.lon0_rad)) <= tol_deg
        )

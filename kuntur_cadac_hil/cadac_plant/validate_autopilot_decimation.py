#!/usr/bin/env python3
"""
validate_autopilot_decimation.py -- Fase 2 del plan HIL
(plan-hil-stm32-portado-autopiloto.md §5/§11): corre el soak test de
patrullaje (N ciclos sobre scenarios/waypoints_cordoba.txt, default 18 --
el mismo usado para validar el SIL original) dos veces sobre el MISMO
RealFalcon6Adapter -- una vez nativo (autopiloto a 1kHz, un tick de
'guidance'/'control'/'actuator' por paso físico) y otra con el
autopiloto decimado a --rate-hz (default 100Hz, ver
set_autopilot_decimation() en plane_adapter.py) -- y compara dwbh
(distancia horizontal de aproximación más cercana al waypoint,
plane[415]) y psivlx (rumbo en ese instante, flat6[240]) en cada
reenganche de waypoint (wp_flag: 1 -> -1) entre ambas corridas.

Gate (plan §10, Nivel 1): la dispersión de dwbh/psivlx por reenganche
tiene que ser del mismo orden que el soak test original (submilimétrica
a pocos centímetros en dwbh, centésimas de grado en psivlx). Si la
decimación a 100Hz degrada mucho más que eso, hay que revisar la
cadencia ANTES de pasar a Fase 6 (HIL con hardware real) -- ver
firmware/kuntur_autopilot/README.md.

No requiere KSE ni ningún proceso aparte -- llama a RealFalcon6Adapter
directo, sin pasar por plant_driver.py/protocol.py (no hay telemetría
que enviar a ningún lado, solo trayectoria interna que comparar).

Uso: python3 -m cadac_plant.validate_autopilot_decimation [--cycles 18] [--rate-hz 100]
"""
from __future__ import annotations

import argparse
import os
import time

from .plane_adapter import ActuatorCommand, RealFalcon6Adapter
from kuntur_bridge.geodesy import SACOLocalFrame
from kuntur_bridge.waypoints_file import load_waypoints_list

_HERE = os.path.dirname(__file__)
ASC_PATH = os.path.join(_HERE, "..", "cadac_py", "CADAC", "FALCON6", "f16c11_7.asc")
WAYPOINTS_PATH = os.path.join(_HERE, "..", "scenarios", "waypoints_cordoba.txt")

# Mismo origen/condición inicial que plant_driver.py._cli() usa para
# --adapter real (ver ahí) -- consistencia con cualquier corrida futura
# contra el mismo escenario.
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = -31.31, -64.21, 589.0
IC = dict(north0=0.0, east0=0.0, down0=-1000.0, psi0=0.0, ias0=180.0)

# Margen generoso por tramo (perímetro real ~38km/18.3 vueltas a 180m/s
# -- ver sesión de cálculo previa) -- tope de seguridad, no el criterio
# de corte real (eso es n_cycles*4 cruces detectados).
MAX_SECONDS_PER_LEG = 45.0


def run_soak(
    adapter: RealFalcon6Adapter, decimation: int, n_cycles: int, max_steps: int,
) -> list[tuple[int, float, float]]:
    """Corre hasta detectar n_cycles*4 cruces de waypoint (wp_flag
    1->-1) o hasta max_steps, lo que pase primero. Devuelve
    (step, dwbh_m, psivlx_deg) por cruce."""
    adapter.set_autopilot_decimation(decimation)
    adapter.reset(**IC)
    actuator = ActuatorCommand(elevator=0.0, aileron=0.0, rudder=0.0, throttle=0.5)

    plane = adapter.vehicle.plane
    flat6 = adapter.vehicle.flat6
    prev_wp_flag = 1
    crossings: list[tuple[int, float, float]] = []
    step = 0
    target = n_cycles * 4
    while step < max_steps and len(crossings) < target:
        adapter.step(adapter.int_step_s, actuator)
        wp_flag = plane[419].integer()
        if wp_flag == -1 and prev_wp_flag != -1:
            crossings.append((step, plane[415].real(), flat6[240].real()))
        prev_wp_flag = wp_flag
        step += 1
    return crossings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=18)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    args = parser.parse_args()

    frame = SACOLocalFrame(ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT)
    waypoints = load_waypoints_list(WAYPOINTS_PATH, frame)

    adapter = RealFalcon6Adapter(ASC_PATH, waypoints=waypoints)
    adapter.set_frame(frame)

    decimation = round(1.0 / (args.rate_hz * adapter.int_step_s))
    max_steps = int(args.cycles * 4 * MAX_SECONDS_PER_LEG / adapter.int_step_s)

    print(f"int_step={adapter.int_step_s}s -- decimación {args.rate_hz}Hz -> n={decimation}")
    print(f"tope de seguridad: {max_steps} pasos ({max_steps * adapter.int_step_s:.0f}s simulados)")

    print(f"\n[1/2] SIL nativo (1kHz, n=1) -- {args.cycles} ciclos...")
    t0 = time.monotonic()
    native = run_soak(adapter, decimation=1, n_cycles=args.cycles, max_steps=max_steps)
    print(f"  {len(native)} cruces detectados en {time.monotonic() - t0:.1f}s de cómputo")

    print(f"\n[2/2] SIL decimado ({args.rate_hz}Hz, n={decimation}) -- {args.cycles} ciclos...")
    t0 = time.monotonic()
    decimated = run_soak(adapter, decimation=decimation, n_cycles=args.cycles, max_steps=max_steps)
    print(f"  {len(decimated)} cruces detectados en {time.monotonic() - t0:.1f}s de cómputo")

    n = min(len(native), len(decimated))
    if n == 0:
        print("\nFALLO: no se detectó ningún cruce de waypoint en alguna de las corridas "
              "(¿max_steps insuficiente, o el patrullaje en bucle no se activó?)")
        return 1
    if len(native) != len(decimated):
        print(f"\nAVISO: cantidad de cruces distinta entre corridas "
              f"(nativo={len(native)}, decimado={len(decimated)}) -- comparando los primeros {n}")

    print(f"\ncomparando {n} cruces pareados (nativo vs decimado):")
    print(f"{'cruce':>5} {'dwbh_nat(m)':>12} {'dwbh_dec(m)':>12} {'diff(m)':>10} "
          f"{'psivlx_nat(deg)':>16} {'psivlx_dec(deg)':>16} {'diff(deg)':>10}")
    max_d_dwbh = 0.0
    max_d_psivlx = 0.0
    for i in range(n):
        _, dwbh_n, psivlx_n = native[i]
        _, dwbh_d, psivlx_d = decimated[i]
        d_dwbh = abs(dwbh_n - dwbh_d)
        d_psivlx = abs(psivlx_n - psivlx_d)
        max_d_dwbh = max(max_d_dwbh, d_dwbh)
        max_d_psivlx = max(max_d_psivlx, d_psivlx)
        print(f"{i:5d} {dwbh_n:12.4f} {dwbh_d:12.4f} {d_dwbh:10.6f} "
              f"{psivlx_n:16.4f} {psivlx_d:16.4f} {d_psivlx:10.6f}")

    print(f"\ndispersión máxima dwbh:   {max_d_dwbh:.6f} m")
    print(f"dispersión máxima psivlx: {max_d_psivlx:.6f} deg")
    print("\n(gate del plan §10 Nivel 1: 'mismo orden que el soak original' -- "
          "submilimétrica a pocos centímetros en dwbh, centésimas de grado en "
          "psivlx. Evaluar a ojo contra estos números, no hay un umbral numérico "
          "fijo declarado en el plan.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
capture_traces.py -- Nivel 0 del plan (plan-hil-stm32-portado-autopiloto.md
§10): captura traces de FALCON6 Python (RealFalcon6Adapter corriendo
f16c11_7.asc nativo, 1 kHz) para comparar contra el C portado
(falcon6_autopilot_step(), test_bitexact.c).

Timing -- por qué el wire_input(k) mezcla DOS llamadas a step() distintas
----------------------------------------------------------------------
Dentro de UN step() (orden real de f16c11_7.asc: environment, kinematics,
aerodynamics, propulsion, forces, guidance, control, actuator, euler,
newton):

  - environment()/kinematics()/aerodynamics() computan grav/pdynmc/vmach/
    dvbe/derivadas a partir del estado de POSICIÓN/ACTITUD/VELOCIDAD
    vigente ANTES de este step (es decir, lo que devolvió el step()
    ANTERIOR) -- newton()/euler() recién avanzan esos valores al FINAL
    de este mismo step().
  - guidance()/control() usan esas derivadas (recién calculadas, mismo
    step) junto con la actitud/posición/velocidad VIEJA (previa a este
    step, ya que euler()/newton() todavía no corrieron).
  - actuator() produce delax/delex/delrx con todo lo anterior.

Entonces: delax(k)/delex(k)/delrx(k) (leídos de plane[619..621] justo
después de llamar a step() por k-ésima vez) fueron calculados con:
  - derivadas/pdynmc/vmach/dvbe/grav = autopilot_inputs()/aux_telemetry()
    leídos DESPUÉS de esa misma llamada a step() (SÍ alineado, nada lo
    vuelve a tocar hasta el próximo step()).
  - posición/actitud/velocidad = el PlantState que devolvió la llamada
    a step() ANTERIOR (k-1), NO la de esta misma llamada (esa ya
    quedó adelantada por euler()/newton()).

wp_flag/el resto de guidance (swel/psiflx/thtflx/altcom/mguid/maut) se
leen con la MISMA alineación que la cinemática (post step k-1) -- eventos
de avance de waypoint (IF wp_flag=-1 ENDIF) se evalúan al principio de
cada step(), antes de guidance(); elegir una ventana de captura corta
que no cruce ningún waypoint (ver más abajo) evita tener que resolver
esa frontera con precisión quirúrgica acá.

Ventana elegida: 10000 pasos (10 s de vuelo nativo a 1 kHz) arrancando
desde el reposo del *.asc -- a ~180 m/s TAS eso son <1900 m recorridos,
muy por debajo de los 5385 m hasta el primer waypoint (swel=[5000,2000]),
así que mguid/maut/swel/psiflx/thtflx/altcom son CONSTANTES en toda la
ventana (30/35, sin eventos) -- bit-exactitud del port en los lazos que
esta ventana SÍ ejercita (control_lateral_accel, control_altitude ->
control_normal_accel, control_roll, guidance/guidance_line, actuator/
actuator_scnd) sin tener que lidiar con la frontera de eventos todavía.

Estado persistente -- por qué cada registro lo lleva explícito
----------------------------------------------------------------------
zz/zzd/alcomx/ancomx/phicomx/DX/DXD/DDX/DDXD (Autopilot_State_t) se
acumulan desde la primerísima llamada a step() (la numero 0, antes de
que exista ningún wire_input capturable -- kinematics() recién
convierte los cuaterniones iniciales a phi/theta/psi DENTRO de esa
llamada, no se puede leer "desde afuera" sin ya haber corrido al menos
un step()). Reproducir esa llamada 0 en C exigiría reconstruir inputs
que no se pueden extraer limpiamente del lado Python. En vez de eso,
cada registro de este trace captura el estado persistente TAL CUAL
Python lo tiene ANTES de llamar a step() (o sea, el que de verdad usó
esa llamada) y lo escribe en el registro -- test_bitexact.c lo SETEA
explícito en vez de encadenarlo entre registros, así que cada registro
es una unidad de prueba autocontenida e independiente del arranque en
frío de la simulación.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

FORK = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(FORK))

from cadac_plant.plane_adapter import RealFalcon6Adapter, ActuatorCommand  # noqa: E402
from kuntur_bridge.icd_plant_state import pack_plant_state  # noqa: E402

RAD = 0.0174532925199432  # cadac_py/cadac.py -- mismo valor exacto, no math.radians()

N_STEPS = 10000
ASC_PATH = FORK / "cadac_py" / "CADAC" / "FALCON6" / "f16c11_7.asc"
OUT_PATH = Path(__file__).resolve().parent / "fixtures" / "traces_falcon6_f16c11_7.bin"


def main() -> int:
    adapter = RealFalcon6Adapter(str(ASC_PATH))
    adapter.reset(north0=0.0, east0=0.0, down0=-1000.0, psi0=0.0, ias0=180.0)

    actuator = ActuatorCommand(elevator=0.0, aileron=0.0, rudder=0.0, throttle=0.0)
    plane = adapter.vehicle.plane

    prev_state = adapter.step(adapter.int_step_s, actuator)
    records = []

    for k in range(1, N_STEPS + 1):
        # Waypoint/modo -- MISMA alineación que prev_state (post step k-1,
        # antes de que corra el check_events()/dispatch de este step k).
        swel1 = plane[405].real(); swel2 = plane[406].real(); swel3 = plane[407].real()
        psiflx = plane[408].real(); thtflx = plane[409].real()
        altcom = plane[527].real()
        mguid = plane[400].integer(); maut = plane[500].integer()

        # Estado persistente del autopiloto TAL CUAL Python lo tiene antes
        # de llamar a step() -- ver nota de módulo ("Estado persistente").
        # Orden: zz, zzd, alcomx, ancomx, phicomx, DX[3], DXD[3], DDX[3], DDXD[3].
        zz = plane[517].real(); zzd = plane[516].real()
        alcomx_state = plane[522].real(); ancomx_state = plane[523].real()
        phicomx_state = plane[553].real()
        DX = plane[631].vec(); DXD = plane[630].vec()
        DDX = plane[633].vec(); DDXD = plane[632].vec()
        state_in = struct.pack(
            "<ddddd" + "ddd" * 4,
            zz, zzd, alcomx_state, ancomx_state, phicomx_state,
            DX[0], DX[1], DX[2], DXD[0], DXD[1], DXD[2],
            DDX[0], DDX[1], DDX[2], DDXD[0], DDXD[1], DDXD[2],
        )

        state = adapter.step(adapter.int_step_s, actuator)
        ai = adapter.autopilot_inputs()
        aux = adapter.aux_telemetry()

        delax = plane[619].real(); delex = plane[620].real(); delrx = plane[621].real()

        raw = pack_plant_state(
            t_sim_us=k * 1000, seq_num=k,
            # Sin bit INIT: este test inyecta el estado persistente
            # explícito en cada registro (ver más abajo) -- si
            # falcon6_autopilot_step() viera flags.init=1 lo pisaría con
            # autopilot_state_reset() antes de usarlo.
            flags=2,
            sn=prev_state.north, se=prev_state.east, sd=prev_state.down,
            vn=prev_state.vn, ve=prev_state.ve, vd=prev_state.vd,
            phi=prev_state.phi, theta=prev_state.theta, psi=prev_state.psi,
            p=prev_state.p, q=prev_state.q, r=prev_state.r,
            ax=prev_state.ax, ay=prev_state.ay, az=prev_state.az,
            pdynmc=aux.pdynmc, vmach=aux.vmach, dvbe=aux.dvbe, grav=ai.grav,
            alpha=prev_state.alpha, beta=prev_state.beta,
            dla=ai.dla, dlde=ai.dlde, dma=ai.dma, dmq=ai.dmq, dmde=ai.dmde,
            dyb=ai.dyb, dydr=ai.dydr, dnb=ai.dnb, dnr=ai.dnr, dndr=ai.dndr,
            dllp=ai.dllp, dllda=ai.dllda,
            gmax=ai.gmax, gminx=ai.gminx,
            swel1=swel1, swel2=swel2, swel3=swel3,
            psiflx=psiflx, thtflx=thtflx, altcom=altcom,
            mguid=mguid, maut=maut,
            manual_active=0,
        )
        # falcon6_autopilot_step() devuelve delax/delex/delrx en RADIANES
        # (ver icd_schema/kuntur_ic_surface_cmd_v1.json) -- plane[619..621]
        # está en grados (unidad nativa de plane[]), convertir acá con el
        # MISMO RAD que usa el C (cadac_py/cadac.py, no math.radians()).
        expected = struct.pack("<ddd", delax * RAD, delex * RAD, delrx * RAD)
        records.append(raw + state_in + expected)

        prev_state = state

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_bytes(b"".join(records))
    print(f"{len(records)} traces escritas en {OUT_PATH} ({OUT_PATH.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

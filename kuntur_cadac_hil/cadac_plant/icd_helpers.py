"""
icd_helpers.py -- Utilidades del lado planta que NO son parte del contrato
binario en sí (eso vive en kuntur_bridge/icd.py, generado) sino de la
física de conversión que hoy hace kuntur_ic.cpp antes de empaquetar.

Se porta acá UNA sola función, verificada línea por línea contra
kuntur_ic.cpp::specific_force_body() (Especificación_r01.md, sección 3.4):
la transformación de aceleración cinemática cruda a fuerza específica en
ejes cuerpo, que es lo que el ICD espera en accel_body (listo para
HIL_SENSOR.xacc/yacc/zacc sin transformación adicional).

Se porta a Python -- no se reimplementa "a ojo" -- para que el lado
planta del fork (Python/cadac_py) produzca el mismo campo que producía
el lado planta original (C++/Simulink), sin depender de que cadac_py
exponga la fuerza específica ya calculada.
"""
from __future__ import annotations

import math

G_ACCEL = 9.80665  # m/s^2 -- idéntico a kuntur_ic.cpp (Especificación_r01.md, sección 3.4)


def specific_force_body(
    ax_kinematic: float, ay_kinematic: float, az_kinematic: float,
    phi_rad: float, theta_rad: float,
) -> tuple[float, float, float]:
    """f_body = A_bb - g_body, con g_body = R_n^b * [0, 0, g].

    Réplica exacta de specific_force_body() en kuntur_ic.cpp. Verificado
    ahí contra el caso de reposo de la especificación:
    f_body(reposo, nivelado) = [0, 0, -9.80665].

    ax/ay/az de entrada son la aceleración CINEMÁTICA cruda (lo que
    entrega directamente newton.py/euler.py de cadac_py) -- NO la fuerza
    específica. La conversión es responsabilidad de esta función, igual
    que hoy es responsabilidad de la S-Function y no de Simulink.
    """
    sphi, cphi = math.sin(phi_rad), math.cos(phi_rad)
    sth, cth = math.sin(theta_rad), math.cos(theta_rad)

    gx_body = -G_ACCEL * sth
    gy_body = G_ACCEL * sphi * cth
    gz_body = G_ACCEL * cphi * cth

    fx = ax_kinematic - gx_body
    fy = ay_kinematic - gy_body
    fz = az_kinematic - gz_body
    return fx, fy, fz


def _self_test() -> None:
    # Caso de reposo, nivelado (phi=theta=0): A_bb cinemática = 0 en
    # reposo -> f_body debe dar [0, 0, -g] (lo que documenta
    # Especificación_r01.md y lo que kuntur_ic.cpp ya tiene verificado).
    fx, fy, fz = specific_force_body(0.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(fx) < 1e-9 and abs(fy) < 1e-9, (fx, fy)
    assert abs(fz - (-G_ACCEL)) < 1e-9, fz
    print("specific_force_body: caso de reposo nivelado OK ->", (fx, fy, fz))


if __name__ == "__main__":
    _self_test()

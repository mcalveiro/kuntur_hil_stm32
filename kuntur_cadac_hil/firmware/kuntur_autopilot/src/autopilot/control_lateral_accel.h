/*
 * control_lateral_accel.h -- portado de
 * cadac_py/falcon6.py::control_lateral_accel.
 *
 * Hallazgo del port: la función Python lee `grav = f[55].real()` pero
 * NUNCA lo usa en el cálculo (lectura muerta, igual que yy/yyd en
 * autopilot_state.h o `halt` en guidance()) -- no se porta ese
 * parámetro acá, documentado para que no sorprenda si se compara
 * línea por línea contra el original.
 */
#ifndef KUNTUR_CONTROL_LATERAL_ACCEL_H
#define KUNTUR_CONTROL_LATERAL_ACCEL_H

/* alcomx: comando de aceleración lateral, g's. az: fuerza específica
 * eje z cuerpo, m/s^2 (FSPB[2]). gainl: ganancia del *.asc. Devuelve
 * phicomx, deg. */
double control_lateral_accel(double alcomx, double az, double gainl);

#endif /* KUNTUR_CONTROL_LATERAL_ACCEL_H */

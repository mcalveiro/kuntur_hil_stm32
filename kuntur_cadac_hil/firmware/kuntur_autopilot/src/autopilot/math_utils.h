/*
 * math_utils.h -- constantes y utilidades matemáticas usadas por el
 * autopiloto FALCON6 portado (ver cadac_py/cadac.py, matrix.py,
 * integrate.py -- SOLO lo que los lazos de control/guiado realmente
 * usan, no toda la librería Matrix/CADAC original).
 *
 * vec3_t / mat33_t: vectores/matrices 3x1 y 3x3 como arrays planos de
 * double -- sin asignación dinámica, apropiado para el STM32F767 sin
 * RTOS (ver plan-hil-stm32-portado-autopiloto.md §7).
 */
#ifndef KUNTUR_AUTOPILOT_MATH_UTILS_H
#define KUNTUR_AUTOPILOT_MATH_UTILS_H

/* cadac_py/cadac.py -- valores EXACTOS, no redondeados, para preservar
 * bit-exactitud contra la referencia Python. */
#define RAD 0.0174532925199432
#define DEG 57.2957795130823
#define AGRAV 9.80675445
#define SMALL 1.0e-7

typedef struct {
    double v[3];
} vec3_t;

typedef struct {
    double m[3][3];
} mat33_t;

/* matrix.py::sign() -- 1 si >= 0, -1 si no (incluye el caso ==0, a
 * diferencia de math.copysign). */
int fc_sign(double x);

/* integrate.py::integrate_scalar() -- método de Euler-Midpoint
 * modificado (Peter H. Zipfel). */
double fc_integrate(double dydx_new, double dydx, double y, double int_step);

/* ------------------------------------------------------------------ */
/* vec3_t / mat33_t                                                    */
/* ------------------------------------------------------------------ */

vec3_t vec3_make(double v0, double v1, double v2);
vec3_t vec3_sub(vec3_t a, vec3_t b);
double vec3_dot(vec3_t a, vec3_t b);      /* matrix.py::__xor__ */
double vec3_norm(vec3_t a);               /* matrix.py::absolute() */

/* matrix.py::pol_from_cart() -- (magnitud, azimut, elevación), en ese
 * orden, replicando el layout [[mag],[az],[el]] del original. */
vec3_t vec3_pol_from_cart(vec3_t a);

mat33_t mat33_make(
    double m11, double m12, double m13,
    double m21, double m22, double m23,
    double m31, double m32, double m33
);
vec3_t mat33_mulvec3(mat33_t m, vec3_t v);   /* matrix.py::__mul__ (Matrix*Matrix, nx1) */
mat33_t mat33_sub(mat33_t a, mat33_t b);
mat33_t mat33_mulouter(vec3_t a, vec3_t b);  /* BB * (~GAINGAM): columna x fila -> 3x3 */

/* matrix.py::inverse() (np.linalg.inv para 3x3) -- vía cofactores.
 * Sin chequeo de determinante == 0: el autopiloto original tampoco lo
 * hace (confía en que el sistema es siempre invertible para esta
 * planta) -- portado línea por línea, no se agrega robustez que el
 * original no tiene. */
mat33_t mat33_inverse(mat33_t m);

/* matrix.py::mat2tr() -- T.M. de la secuencia psivg -> thtvg (2 ángulos
 * de Euler). Ángulos en RADIANES. */
mat33_t fc_mat2tr(double psivg, double thtvg);

#endif /* KUNTUR_AUTOPILOT_MATH_UTILS_H */

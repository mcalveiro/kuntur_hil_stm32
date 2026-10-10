#include "math_utils.h"
#include <math.h>

int fc_sign(double x) {
    return (x >= 0.0) ? 1 : -1;
}

double fc_integrate(double dydx_new, double dydx, double y, double int_step) {
    return y + (dydx_new + dydx) * int_step / 2.0;
}

vec3_t vec3_make(double v0, double v1, double v2) {
    vec3_t r;
    r.v[0] = v0; r.v[1] = v1; r.v[2] = v2;
    return r;
}

vec3_t vec3_sub(vec3_t a, vec3_t b) {
    return vec3_make(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]);
}

double vec3_dot(vec3_t a, vec3_t b) {
    return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

double vec3_norm(vec3_t a) {
    return sqrt(vec3_dot(a, a));
}

vec3_t vec3_pol_from_cart(vec3_t a) {
    double v1 = a.v[0], v2 = a.v[1], v3 = a.v[2];
    double magnitude = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    double azimuth = atan2(v2, v1);
    double elevation = atan2(-v3, sqrt(v1 * v1 + v2 * v2));
    return vec3_make(magnitude, azimuth, elevation);
}

mat33_t mat33_make(
    double m11, double m12, double m13,
    double m21, double m22, double m23,
    double m31, double m32, double m33
) {
    mat33_t r;
    r.m[0][0] = m11; r.m[0][1] = m12; r.m[0][2] = m13;
    r.m[1][0] = m21; r.m[1][1] = m22; r.m[1][2] = m23;
    r.m[2][0] = m31; r.m[2][1] = m32; r.m[2][2] = m33;
    return r;
}

vec3_t mat33_mulvec3(mat33_t m, vec3_t v) {
    vec3_t r;
    for (int i = 0; i < 3; i++) {
        r.v[i] = m.m[i][0] * v.v[0] + m.m[i][1] * v.v[1] + m.m[i][2] * v.v[2];
    }
    return r;
}

mat33_t mat33_sub(mat33_t a, mat33_t b) {
    mat33_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = a.m[i][j] - b.m[i][j];
        }
    }
    return r;
}

mat33_t mat33_mulouter(vec3_t a, vec3_t b) {
    /* a (3x1) * b^T (1x3) -> 3x3, equivalente a BB * (~GAINGAM) en
     * control_gamma() (BB es 3x1, ~GAINGAM es la transpuesta de un 3x1
     * -> 1x3). */
    mat33_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = a.v[i] * b.v[j];
        }
    }
    return r;
}

mat33_t mat33_inverse(mat33_t a) {
    /* Inversa por cofactores (regla de Cramer) -- matemáticamente
     * equivalente a np.linalg.inv() para una matriz bien condicionada,
     * pero el ORDEN de las operaciones de punto flotante difiere de la
     * descomposición LU que usa LAPACK (numpy). Para las matrices que
     * aparecen en control_gamma() esto es indistinguible en la práctica
     * (ver test de regresión Nivel 0), pero queda documentado: si algún
     * día aparece una divergencia > 1e-10 achacable a esto, es el
     * motivo. f16c11_7.asc NUNCA ejercita este camino (mautp==4 no se
     * usa en ese escenario) -- no hay traces de referencia que lo
     * cubran todavía. */
    double a00 = a.m[0][0], a01 = a.m[0][1], a02 = a.m[0][2];
    double a10 = a.m[1][0], a11 = a.m[1][1], a12 = a.m[1][2];
    double a20 = a.m[2][0], a21 = a.m[2][1], a22 = a.m[2][2];

    double c00 = a11 * a22 - a12 * a21;
    double c01 = -(a10 * a22 - a12 * a20);
    double c02 = a10 * a21 - a11 * a20;
    double c10 = -(a01 * a22 - a02 * a21);
    double c11 = a00 * a22 - a02 * a20;
    double c12 = -(a00 * a21 - a01 * a20);
    double c20 = a01 * a12 - a02 * a11;
    double c21 = -(a00 * a12 - a02 * a10);
    double c22 = a00 * a11 - a01 * a10;

    double det = a00 * c00 + a01 * c01 + a02 * c02;

    mat33_t inv;
    /* adjugate = transpuesta de la matriz de cofactores */
    inv.m[0][0] = c00 / det; inv.m[0][1] = c10 / det; inv.m[0][2] = c20 / det;
    inv.m[1][0] = c01 / det; inv.m[1][1] = c11 / det; inv.m[1][2] = c21 / det;
    inv.m[2][0] = c02 / det; inv.m[2][1] = c12 / det; inv.m[2][2] = c22 / det;
    return inv;
}

mat33_t fc_mat2tr(double psivg, double thtvg) {
    double cpsi = cos(psivg), spsi = sin(psivg);
    double ctht = cos(thtvg), stht = sin(thtvg);
    return mat33_make(
        ctht * cpsi, ctht * spsi, -stht,
        -spsi, cpsi, 0.0,
        stht * cpsi, stht * spsi, ctht
    );
}

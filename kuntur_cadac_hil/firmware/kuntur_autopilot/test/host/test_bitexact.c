/*
 * test_bitexact.c -- Nivel 0 del plan (plan-hil-stm32-portado-autopiloto.md
 * §10): corre falcon6_autopilot_step() (C) sobre las traces capturadas
 * de FALCON6 Python (capture_traces.py) y compara (tolerancia 1e-10,
 * ver gate del plan) cada delax/delex/delrx contra el valor que produjo
 * la referencia Python para ESE MISMO registro.
 *
 * Cada registro es AUTOCONTENIDO: trae el estado persistente del
 * autopiloto (Autopilot_State_t) TAL CUAL Python lo tenía antes de
 * llamar a step() -- este test lo SETEA explícito en cada iteración,
 * no lo encadena entre registros (ver docstring de capture_traces.py,
 * sección "Estado persistente", sobre por qué: la llamada 0 de Python
 * no se puede reproducir limpiamente desde afuera, así que no hay forma
 * de arrancar C "en frío" de forma equivalente -- inyectar el estado ya
 * evita el problema en vez de resolverlo).
 *
 * Formato de cada registro: KUNTUR_PLANT_STATE_SIZE bytes (wire) +
 * 17 double (estado persistente de entrada) + 3 double (delax, delex,
 * delrx esperados) -- todo little-endian.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "kuntur_ic_plant_state.h"
#include "kuntur_ic_surface_cmd.h"
#include "autopilot_state.h"
#include "falcon6_autopilot.h"
#include "scenario_params.h"

#define N_STATE_DOUBLES 17u
#define RECORD_SIZE (KUNTUR_PLANT_STATE_SIZE + N_STATE_DOUBLES * 8u + 24u)
#define GATE_TOLERANCE 1e-10

/* int_step del autopiloto -- 0.001s para igualar la referencia
 * (capturada a 1 kHz nativo, ver capture_traces.py). En HIL real esto
 * sería el período de 100 Hz (0.01s) -- la Fase 2 del plan valida esa
 * decimación por separado; acá se valida el PORT del algoritmo, no la
 * cadencia. */
#define INT_STEP 0.001

static double get_f64_le(const uint8_t *p) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++) {
        bits |= ((uint64_t)p[i]) << (8 * i);
    }
    double v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

/* Orden EXACTO de capture_traces.py: zz, zzd, alcomx, ancomx, phicomx,
 * DX[3], DXD[3], DDX[3], DDXD[3]. */
static void unpack_state(Autopilot_State_t *s, const uint8_t *p) {
    memset(s, 0, sizeof(*s));
    s->zz = get_f64_le(p + 0 * 8);
    s->zzd = get_f64_le(p + 1 * 8);
    s->alcomx = get_f64_le(p + 2 * 8);
    s->ancomx = get_f64_le(p + 3 * 8);
    s->phicomx = get_f64_le(p + 4 * 8);
    for (int i = 0; i < 3; i++) s->DX[i] = get_f64_le(p + (5 + i) * 8);
    for (int i = 0; i < 3; i++) s->DXD[i] = get_f64_le(p + (8 + i) * 8);
    for (int i = 0; i < 3; i++) s->DDX[i] = get_f64_le(p + (11 + i) * 8);
    for (int i = 0; i < 3; i++) s->DDXD[i] = get_f64_le(p + (14 + i) * 8);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "uso: %s <traces.bin>\n", argv[0]);
        return 1;
    }

    FILE *f = fopen(argv[1], "rb");
    if (f == NULL) {
        fprintf(stderr, "no se pudo abrir %s\n", argv[1]);
        return 1;
    }

    Scenario_Params_t params = SCENARIO_PARAMS_F16C11_7_INIT;

    uint8_t record[RECORD_SIZE];
    size_t n_records = 0;
    double max_diff = 0.0;
    size_t n_failures = 0;

    while (fread(record, 1, RECORD_SIZE, f) == RECORD_SIZE) {
        kuntur_plant_state_t in;
        if (kuntur_unpack_plant_state(&in, record, KUNTUR_PLANT_STATE_SIZE) != 0) {
            fprintf(stderr, "registro %zu: KUNTUR_PLANT_STATE invalido\n", n_records);
            return 1;
        }

        Autopilot_State_t state;
        unpack_state(&state, record + KUNTUR_PLANT_STATE_SIZE);

        const uint8_t *exp = record + KUNTUR_PLANT_STATE_SIZE + N_STATE_DOUBLES * 8u;
        double expected_delax = get_f64_le(exp + 0);
        double expected_delex = get_f64_le(exp + 8);
        double expected_delrx = get_f64_le(exp + 16);

        kuntur_surface_cmd_t out;
        falcon6_autopilot_step(&in, &params, &state, INT_STEP, &out);

        double d_a = fabs(out.delax - expected_delax);
        double d_e = fabs(out.delex - expected_delex);
        double d_r = fabs(out.delrx - expected_delrx);
        double worst = d_a;
        if (d_e > worst) worst = d_e;
        if (d_r > worst) worst = d_r;

        if (worst > max_diff) max_diff = worst;
        if (worst > GATE_TOLERANCE) {
            if (n_failures < 10) {
                fprintf(stderr,
                    "registro %zu (seq_num=%u): diff=%e -- delax: c=%.15f py=%.15f, delex: c=%.15f py=%.15f, delrx: c=%.15f py=%.15f\n",
                    n_records, in.seq_num, worst,
                    out.delax, expected_delax, out.delex, expected_delex, out.delrx, expected_delrx);
            }
            n_failures++;
        }

        n_records++;
    }
    fclose(f);

    printf("registros comparados: %zu\n", n_records);
    printf("diferencia maxima (delax/delex/delrx, rad): %e\n", max_diff);
    printf("tolerancia del gate (plan Nivel 0): %e\n", GATE_TOLERANCE);

    if (n_failures > 0) {
        printf("FALLO: %zu registros exceden la tolerancia\n", n_failures);
        return 1;
    }
    printf("BIT-EXACTO (Nivel 0, C vs Python)\n");
    return 0;
}

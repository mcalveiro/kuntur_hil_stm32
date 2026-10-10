/*
 * test_icd_packing.c -- empaqueta payloads de prueba con el C de
 * src/icd/ y los escribe a archivos binarios, para que
 * test_icd_packing.py los compare byte a byte contra lo que produce
 * kuntur_bridge/icd_plant_state.py / icd_surface_cmd.py con LOS MISMOS
 * valores de entrada (ver plan-hil-stm32-portado-autopiloto.md §6.3(b)).
 *
 * Uso: test_icd_packing <plant_state.bin> <surface_cmd.bin>
 *
 * Compilado para host (gcc nativo, x86_64) -- NO es el cross-compile
 * para Cortex-M7 (eso es Fase 4 del plan, con el toolchain de
 * PlatformIO). Acá solo se valida el LAYOUT de bytes, independiente del
 * target: el wire format es little-endian explícito (ver
 * kuntur_ic_wire.c), no depende de la arquitectura que lo ejecuta.
 */
#include <stdio.h>
#include <stdlib.h>

#include "kuntur_ic_plant_state.h"
#include "kuntur_ic_surface_cmd.h"

static void write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        fprintf(stderr, "no se pudo abrir %s para escritura\n", path);
        exit(1);
    }
    if (fwrite(data, 1, size, f) != size) {
        fprintf(stderr, "escritura incompleta en %s\n", path);
        exit(1);
    }
    fclose(f);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "uso: %s <plant_state.bin> <surface_cmd.bin>\n", argv[0]);
        return 1;
    }

    /* Mismos valores que test_icd_packing.py: 1.0, 2.0, 3.0, ... para
     * cada campo f64 en el orden del schema -- fácil de verificar a
     * ojo y suficiente para detectar un campo desplazado/con el tipo
     * equivocado (un valor repetido no lo detectaría). */
    kuntur_plant_state_t ps;
    ps.t_sim_us = 123456789ULL;
    ps.seq_num = 42;
    ps.flags = KUNTUR_PLANT_STATE_FLAG_INIT | KUNTUR_PLANT_STATE_FLAG_ACTIVE;

    double *ps_doubles[] = {
        &ps.sn, &ps.se, &ps.sd, &ps.vn, &ps.ve, &ps.vd,
        &ps.phi, &ps.theta, &ps.psi, &ps.p, &ps.q, &ps.r,
        &ps.ax, &ps.ay, &ps.az,
        &ps.pdynmc, &ps.vmach, &ps.dvbe, &ps.grav, &ps.alpha, &ps.beta,
        &ps.dla, &ps.dlde, &ps.dma, &ps.dmq, &ps.dmde,
        &ps.dyb, &ps.dydr, &ps.dnb, &ps.dnr, &ps.dndr,
        &ps.dllp, &ps.dllda, &ps.gmax, &ps.gminx,
        &ps.swel1, &ps.swel2, &ps.swel3, &ps.psiflx, &ps.thtflx, &ps.altcom,
    };
    size_t n_doubles = sizeof(ps_doubles) / sizeof(ps_doubles[0]);
    for (size_t i = 0; i < n_doubles; i++) {
        *ps_doubles[i] = (double)(i + 1) * 1.5;
    }
    ps.mguid = 30;
    ps.maut = 35;
    ps.manual_active = 0;

    uint8_t ps_buf[KUNTUR_PLANT_STATE_SIZE];
    size_t ps_written = kuntur_pack_plant_state(ps_buf, &ps);
    if (ps_written != KUNTUR_PLANT_STATE_SIZE) {
        fprintf(stderr, "kuntur_pack_plant_state: tamano inesperado %zu\n", ps_written);
        return 1;
    }
    write_file(argv[1], ps_buf, KUNTUR_PLANT_STATE_SIZE);

    kuntur_surface_cmd_t sc;
    sc.t_sim_us = 123456789ULL;
    sc.seq_num = 42;
    sc.status = KUNTUR_SURFACE_CMD_STATUS_OK;
    sc.delax = 0.01;
    sc.delex = -0.02;
    sc.delrx = 0.003;
    sc.throttle = 0.75;
    sc.gainfb1 = 1.1;
    sc.gainfb2 = 2.2;
    sc.gainfb3 = 3.3;
    sc.wp_flag = -1.0;

    uint8_t sc_buf[KUNTUR_SURFACE_CMD_SIZE];
    size_t sc_written = kuntur_pack_surface_cmd(sc_buf, &sc);
    if (sc_written != KUNTUR_SURFACE_CMD_SIZE) {
        fprintf(stderr, "kuntur_pack_surface_cmd: tamano inesperado %zu\n", sc_written);
        return 1;
    }
    write_file(argv[2], sc_buf, KUNTUR_SURFACE_CMD_SIZE);

    return 0;
}

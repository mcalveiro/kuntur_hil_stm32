/*
 * main_bitexact.c -- testbench Nivel 2 del plan
 * (plan-hil-stm32-portado-autopiloto.md §10), corriendo en el target
 * real (Fase 4, ver README.md). Mismo algoritmo que
 * test/host/test_bitexact.c (host, Nivel 0, gate 0.0 exacto sobre las
 * 10000 muestras completas) pero:
 *   - datos embebidos en flash (traces_data.c, subconjunto de 1000
 *     registros -- ver test/target/gen_traces_data.py) en vez de leer
 *     un archivo,
 *   - tolerancia del gate relajada a 1e-6 (Nivel 2: diff FPU/orden-de-
 *     operaciones entre host x86_64 y Cortex-M7, no 0.0 exacto),
 *   - reporte por UART (USART3/VCP) en vez de stdout,
 *   - mide ciclos por iteracion con el DWT cycle counter (performance
 *     budget del plan §7: loop principal <10ms, computo estimado
 *     100-500us).
 *
 * Build/env propio (nucleo_f767zi_bitexact en platformio.ini) que
 * excluye src/main.c (el blink) y en cambio linkea este archivo.
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "stm32f7xx_hal.h"

#include "../bsp/clock_config.h"
#include "../bsp/uart_console.h"
#include "traces_data.h"

#include "../icd/kuntur_ic_plant_state.h"
#include "../icd/kuntur_ic_surface_cmd.h"
#include "../autopilot/autopilot_state.h"
#include "../autopilot/falcon6_autopilot.h"
#include "../config/scenario_params.h"

#define N_STATE_DOUBLES 17u
#define GATE_TOLERANCE 1e-6 /* Nivel 2 (plan §10) -- Nivel 0 (host) ya exigio 0.0 exacto */
#define INT_STEP 0.001      /* igual que test_bitexact.c: iguala la cadencia de captura (1kHz), no la cadencia HIL (100Hz, plan §5) */

static double get_f64_le(const uint8_t *p) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++) {
        bits |= ((uint64_t)p[i]) << (8 * i);
    }
    double v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

/* Orden EXACTO de capture_traces.py / test_bitexact.c: zz, zzd, alcomx,
 * ancomx, phicomx, DX[3], DXD[3], DDX[3], DDXD[3]. */
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

static void init_status_led(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void dwt_cycle_counter_enable(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u; /* unlock (ignorado si el core no tiene lock register; no hace daño) */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Formatea sin depender de printf-con-float (newlib-nano del toolchain
 * de PlatformIO lo compila sin soporte de %e/%f por default -- "-u
 * _printf_float" infla el binario; mas simple evitarlo del todo). Usa
 * solo %lu/%s: el vfprintf de nano.specs tampoco soporta %llu ni ancho
 * dinamico (%0*llu) -- verificado en hardware, ambos se comieron el
 * literal "ll"/"*" en vez de formatear.
 *
 * `decimals` <= 9 (uint32_t div=10^decimals no debe desbordar -- con
 * 10 ya overflow silencioso, visto en hardware: tolerancia 1e-6
 * salia como "0.000000003568" en vez de "0.000001000000"). */
static void format_fixed(double v, unsigned decimals, char *out, size_t out_size) {
    int neg = v < 0.0;
    if (neg) v = -v;

    uint32_t div = 1;
    for (unsigned i = 0; i < decimals; i++) div *= 10u;
    uint32_t scaled = (uint32_t)(v * (double)div + 0.5);
    uint32_t int_part = scaled / div;
    uint32_t frac_part = scaled % div;

    char frac_buf[16];
    snprintf(frac_buf, sizeof(frac_buf), "%lu", (unsigned long)frac_part);
    unsigned frac_len = (unsigned)strlen(frac_buf);
    char padded[16];
    unsigned pad = (decimals > frac_len && decimals < sizeof(padded) - 1) ? decimals - frac_len : 0;
    unsigned j = 0;
    for (; j < pad; j++) padded[j] = '0';
    size_t copy_len = strlen(frac_buf);
    if (copy_len > sizeof(padded) - 1 - pad) copy_len = sizeof(padded) - 1 - pad;
    memcpy(padded + j, frac_buf, copy_len);
    padded[j + copy_len] = '\0';

    snprintf(out, out_size, "%s%lu.%s", neg ? "-" : "", (unsigned long)int_part, padded);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

int main(void) {
    HAL_Init();
    kuntur_clock_config_216mhz();
    SCB_EnableICache();
    SCB_EnableDCache();
    init_status_led();
    kuntur_uart_console_init();
    dwt_cycle_counter_enable();

    Scenario_Params_t params = SCENARIO_PARAMS_F16C11_7_INIT;

    const unsigned record_size = kuntur_target_traces_record_size;
    const unsigned n_records = kuntur_target_traces_n_records;

    double max_diff = 0.0;
    unsigned n_failures = 0;
    uint32_t cycles_min = 0xFFFFFFFFu, cycles_max = 0, cycles_total = 0;

    char line[160];

    kuntur_uart_console_puts("\r\nkuntur_autopilot -- testbench Nivel 2 (target STM32F767)\r\n");
    snprintf(line, sizeof(line), "SystemCoreClock=%lu Hz, registros=%u\r\n",
              (unsigned long)SystemCoreClock, n_records);
    kuntur_uart_console_puts(line);

    for (unsigned i = 0; i < n_records; i++) {
        const uint8_t *record = kuntur_target_traces_data + (size_t)i * record_size;

        kuntur_plant_state_t in;
        if (kuntur_unpack_plant_state(&in, record, KUNTUR_PLANT_STATE_SIZE) != 0) {
            kuntur_uart_console_puts("FALLO: KUNTUR_PLANT_STATE invalido en datos embebidos\r\n");
            while (1) {}
        }

        Autopilot_State_t state;
        unpack_state(&state, record + KUNTUR_PLANT_STATE_SIZE);

        const uint8_t *exp = record + KUNTUR_PLANT_STATE_SIZE + N_STATE_DOUBLES * 8u;
        double expected_delax = get_f64_le(exp + 0);
        double expected_delex = get_f64_le(exp + 8);
        double expected_delrx = get_f64_le(exp + 16);

        kuntur_surface_cmd_t out;
        uint32_t c0 = DWT->CYCCNT;
        falcon6_autopilot_step(&in, &params, &state, INT_STEP, &out);
        uint32_t cycles = DWT->CYCCNT - c0;

        if (cycles < cycles_min) cycles_min = cycles;
        if (cycles > cycles_max) cycles_max = cycles;
        cycles_total += cycles;

        double d_a = fabs(out.delax - expected_delax);
        double d_e = fabs(out.delex - expected_delex);
        double d_r = fabs(out.delrx - expected_delrx);
        double worst = d_a;
        if (d_e > worst) worst = d_e;
        if (d_r > worst) worst = d_r;

        if (worst > max_diff) max_diff = worst;
        if (worst > GATE_TOLERANCE) {
            n_failures++;
            if (n_failures <= 10) {
                char diff_str[32];
                format_fixed(worst, 9, diff_str, sizeof(diff_str));
                snprintf(line, sizeof(line),
                         "registro %u (seq=%lu): diff=%s\r\n",
                         i, (unsigned long)in.seq_num, diff_str);
                kuntur_uart_console_puts(line);
            }
        }
    }

    snprintf(line, sizeof(line), "registros comparados: %u\r\n", n_records);
    kuntur_uart_console_puts(line);

    char max_diff_str[32], tol_str[32];
    format_fixed(max_diff, 9, max_diff_str, sizeof(max_diff_str));
    format_fixed(GATE_TOLERANCE, 9, tol_str, sizeof(tol_str));
    snprintf(line, sizeof(line), "diferencia maxima (delax/delex/delrx, rad): %s\r\n", max_diff_str);
    kuntur_uart_console_puts(line);
    snprintf(line, sizeof(line), "tolerancia del gate (Nivel 2): %s\r\n", tol_str);
    kuntur_uart_console_puts(line);

    uint32_t cycles_avg = cycles_total / n_records;
    double hz = (double)SystemCoreClock;
    char us_min_str[32], us_avg_str[32], us_max_str[32];
    format_fixed((double)cycles_min * 1e6 / hz, 2, us_min_str, sizeof(us_min_str));
    format_fixed((double)cycles_avg * 1e6 / hz, 2, us_avg_str, sizeof(us_avg_str));
    format_fixed((double)cycles_max * 1e6 / hz, 2, us_max_str, sizeof(us_max_str));
    snprintf(line, sizeof(line),
             "ciclos/iteracion: min=%lu avg=%lu max=%lu  ->  us: min=%s avg=%s max=%s\r\n",
             (unsigned long)cycles_min, (unsigned long)cycles_avg, (unsigned long)cycles_max,
             us_min_str, us_avg_str, us_max_str);
    kuntur_uart_console_puts(line);

    if (n_failures > 0) {
        snprintf(line, sizeof(line), "FALLO: %u registros exceden la tolerancia\r\n", n_failures);
        kuntur_uart_console_puts(line);
    } else {
        kuntur_uart_console_puts("BIT-EXACTO dentro de tolerancia (Nivel 2, C cross-compilado vs Python)\r\n");
    }

    /* LED: parpadeo lento = PASS, rapido = FAIL -- indicador visual sin
     * necesidad de abrir el monitor serie. */
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        HAL_Delay(n_failures > 0 ? 100 : 1000);
    }
}

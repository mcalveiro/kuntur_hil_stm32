#!/usr/bin/env python3
"""
test_bitexact.py -- compila test_bitexact.c (gcc nativo, host) y lo
corre contra fixtures/traces_falcon6_f16c11_7.bin, regenerando las
traces primero si falcon6.py o el *.asc cambiaron desde la última
corrida (comparación de mtime simple). Gate Nivel 0 del plan
(plan-hil-stm32-portado-autopiloto.md §10): diff < 1e-10 en las 10000
muestras de la ventana de prueba.

Uso: python3 firmware/kuntur_autopilot/test/host/test_bitexact.py
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
AUTOPILOT_SRC = TEST_DIR.parent.parent / "src" / "autopilot"
ICD_SRC = TEST_DIR.parent.parent / "src" / "icd"
CONFIG_SRC = TEST_DIR.parent.parent / "src" / "config"
FIXTURES_DIR = TEST_DIR / "fixtures"
TRACES_PATH = FIXTURES_DIR / "traces_falcon6_f16c11_7.bin"
CAPTURE_SCRIPT = TEST_DIR / "capture_traces.py"
ASC_PATH = TEST_DIR.parent.parent.parent.parent / "cadac_py" / "CADAC" / "FALCON6" / "f16c11_7.asc"
FALCON6_PY = TEST_DIR.parent.parent.parent.parent / "cadac_py" / "falcon6.py"

SOURCES = [
    "test_bitexact.c",
    *[str(ICD_SRC / n) for n in ("kuntur_ic_wire.c", "kuntur_ic_plant_state.c", "kuntur_ic_surface_cmd.c")],
    *[str(AUTOPILOT_SRC / n) for n in (
        "math_utils.c", "autopilot_state.c", "control_rates.c", "control_normal_accel.c",
        "control_lateral_accel.c", "control_gamma.c", "control_heading.c", "control_altitude.c",
        "control_mix.c", "guidance_line.c", "actuator.c", "falcon6_autopilot.c",
    )],
]


def _traces_stale() -> bool:
    if not TRACES_PATH.exists():
        return True
    traces_mtime = TRACES_PATH.stat().st_mtime
    return any(p.stat().st_mtime > traces_mtime for p in (FALCON6_PY, ASC_PATH, CAPTURE_SCRIPT))


def main() -> int:
    if _traces_stale():
        print("traces desactualizadas o inexistentes -- regenerando con capture_traces.py...")
        subprocess.run([sys.executable, str(CAPTURE_SCRIPT)], check=True)

    with tempfile.TemporaryDirectory(prefix="kuntur_bitexact_") as tmp_dir:
        binary = Path(tmp_dir) / "test_bitexact"
        cmd = ["cc", "-std=c99", "-Wall", "-Wextra", "-O2",
               "-I", str(ICD_SRC), "-I", str(AUTOPILOT_SRC), "-I", str(CONFIG_SRC),
               "-o", str(binary)] + SOURCES + ["-lm"]
        subprocess.run(cmd, check=True, cwd=TEST_DIR)

        result = subprocess.run([str(binary), str(TRACES_PATH)])
        return result.returncode


if __name__ == "__main__":
    sys.exit(main())

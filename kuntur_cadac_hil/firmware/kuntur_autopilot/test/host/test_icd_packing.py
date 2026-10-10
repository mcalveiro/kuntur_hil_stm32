#!/usr/bin/env python3
"""
test_icd_packing.py -- Nivel 0 del canal ICD HIL: compila el C de
src/icd/ para host (gcc nativo), lo corre, y compara byte a byte el
resultado contra kuntur_bridge/icd_plant_state.py / icd_surface_cmd.py
empaquetando LOS MISMOS valores (ver
plan-hil-stm32-portado-autopiloto.md §6.3(b): "test de regresión que
empaquete el mismo payload desde Python y desde C y verifique diff byte
a byte").

Uso: python3 firmware/kuntur_autopilot/test/host/test_icd_packing.py
(desde la raíz del fork, o desde cualquier lado -- se ubica solo por la
ruta de este archivo).

Gate: RuntimeError si cualquiera de los dos payloads difiere en un solo
byte. Sin dependencias de terceros (subprocess + struct, biblioteca
estándar).
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
ICD_DIR = TEST_DIR.parent.parent / "src" / "icd"
REPO_ROOT = TEST_DIR.parent.parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from kuntur_bridge.icd_plant_state import pack_plant_state  # noqa: E402
from kuntur_bridge.icd_surface_cmd import pack_surface_cmd  # noqa: E402

# Mismos valores que test_icd_packing.c -- si se cambian acá, cambiar
# también allá (y viceversa). Deliberadamente NO repetidos entre campos
# (1.5, 3.0, 4.5, ...) para que un campo desplazado o con el tipo
# equivocado se note como diferencia, no se enmascare.
N_DOUBLES_PLANT_STATE = 41


def _build(cc: str) -> Path:
    tmp_dir = Path(tempfile.mkdtemp(prefix="kuntur_icd_test_"))
    binary = tmp_dir / "test_icd_packing"
    sources = [
        TEST_DIR / "test_icd_packing.c",
        ICD_DIR / "kuntur_ic_wire.c",
        ICD_DIR / "kuntur_ic_plant_state.c",
        ICD_DIR / "kuntur_ic_surface_cmd.c",
    ]
    cmd = [cc, "-std=c99", "-Wall", "-Wextra", "-I", str(ICD_DIR), "-o", str(binary)]
    cmd += [str(s) for s in sources]
    subprocess.run(cmd, check=True)
    return binary


def _expected_plant_state_bytes() -> bytes:
    doubles = [(i + 1) * 1.5 for i in range(N_DOUBLES_PLANT_STATE)]
    return pack_plant_state(
        123456789, 42, 0b11,  # t_sim_us, seq_num, flags (INIT|ACTIVE)
        *doubles,
        30, 35,  # mguid, maut
        0,       # manual_active
    )


def _expected_surface_cmd_bytes() -> bytes:
    return pack_surface_cmd(
        t_sim_us=123456789, seq_num=42, status=1,
        delax=0.01, delex=-0.02, delrx=0.003, throttle=0.75,
        gainfb1=1.1, gainfb2=2.2, gainfb3=3.3, wp_flag=-1.0,
    )


def _diff_report(name: str, expected: bytes, actual: bytes) -> str:
    if len(expected) != len(actual):
        return f"{name}: tamaño Python={len(expected)} vs C={len(actual)}"
    diffs = [i for i, (a, b) in enumerate(zip(expected, actual)) if a != b]
    if not diffs:
        return ""
    preview = ", ".join(f"byte {i}: py=0x{expected[i]:02X} c=0x{actual[i]:02X}" for i in diffs[:10])
    return f"{name}: {len(diffs)} bytes distintos -- {preview}"


def main() -> int:
    binary = _build("cc")

    with tempfile.TemporaryDirectory(prefix="kuntur_icd_out_") as out_dir:
        ps_path = Path(out_dir) / "plant_state.bin"
        sc_path = Path(out_dir) / "surface_cmd.bin"
        subprocess.run([str(binary), str(ps_path), str(sc_path)], check=True)

        c_plant_state = ps_path.read_bytes()
        c_surface_cmd = sc_path.read_bytes()

    py_plant_state = _expected_plant_state_bytes()
    py_surface_cmd = _expected_surface_cmd_bytes()

    errors = [
        e for e in (
            _diff_report("KUNTUR_PLANT_STATE", py_plant_state, c_plant_state),
            _diff_report("KUNTUR_SURFACE_CMD", py_surface_cmd, c_surface_cmd),
        ) if e
    ]

    if errors:
        for e in errors:
            print(f"DIVERGENCIA: {e}", file=sys.stderr)
        return 1

    print(f"KUNTUR_PLANT_STATE: {len(py_plant_state)} bytes, Python == C")
    print(f"KUNTUR_SURFACE_CMD: {len(py_surface_cmd)} bytes, Python == C")
    print("BIT-EXACTO (Python <-> C)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

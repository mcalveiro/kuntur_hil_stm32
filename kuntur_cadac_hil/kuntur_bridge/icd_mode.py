"""
icd_mode.py -- Canal auxiliar KUNTUR_MODE (schema v1.0).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3
(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema
de origen de este archivo para transporte/decimación/propósito.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC_MODE = 0x4B4D4F44  # "KMOD"

MODE_FMT = "<IIH"
MODE_SIZE = struct.calcsize(MODE_FMT)

assert MODE_SIZE == 10, f"MODE_FMT error: {MODE_SIZE} bytes (esperados 10)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class ModePacket:
    """Representación tipada de un KUNTUR_MODE ya validado."""
    seq: int
    mode: int


def unpack_mode(data: bytes) -> ModePacket:
    """Valida tamaño + magic y decodifica un KUNTUR_MODE."""
    if len(data) != MODE_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {MODE_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_MODE:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_MODE:08X}")

    (_magic, seq, mode) = struct.unpack(MODE_FMT, data)

    return ModePacket(
        seq=seq,
        mode=mode,
    )


def pack_mode(seq, mode) -> bytes:
    """Empaqueta un KUNTUR_MODE."""
    return struct.pack(MODE_FMT, MAGIC_MODE, seq, mode)

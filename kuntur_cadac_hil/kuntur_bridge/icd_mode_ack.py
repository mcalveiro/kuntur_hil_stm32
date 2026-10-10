"""
icd_mode_ack.py -- Canal auxiliar KUNTUR_MODE_ACK (schema v1.0).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3
(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema
de origen de este archivo para transporte/decimación/propósito.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC_MODE_ACK = 0x4B4D4132  # "KMA2"

MODE_ACK_FMT = "<IIH"
MODE_ACK_SIZE = struct.calcsize(MODE_ACK_FMT)

assert MODE_ACK_SIZE == 10, f"MODE_ACK_FMT error: {MODE_ACK_SIZE} bytes (esperados 10)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class Mode_ackPacket:
    """Representación tipada de un KUNTUR_MODE_ACK ya validado."""
    seq: int
    status: int


def unpack_mode_ack(data: bytes) -> Mode_ackPacket:
    """Valida tamaño + magic y decodifica un KUNTUR_MODE_ACK."""
    if len(data) != MODE_ACK_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {MODE_ACK_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_MODE_ACK:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_MODE_ACK:08X}")

    (_magic, seq, status) = struct.unpack(MODE_ACK_FMT, data)

    return Mode_ackPacket(
        seq=seq,
        status=status,
    )


def pack_mode_ack(seq, status) -> bytes:
    """Empaqueta un KUNTUR_MODE_ACK."""
    return struct.pack(MODE_ACK_FMT, MAGIC_MODE_ACK, seq, status)

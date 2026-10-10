"""
icd_surface_cmd.py -- Canal auxiliar KUNTUR_SURFACE_CMD (schema v1.0).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3
(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema
de origen de este archivo para transporte/decimación/propósito.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC_SURFACE_CMD = 0x4B534331  # "KSC1"

SURFACE_CMD_FMT = "<IQIHdddddddd"
SURFACE_CMD_SIZE = struct.calcsize(SURFACE_CMD_FMT)

assert SURFACE_CMD_SIZE == 82, f"SURFACE_CMD_FMT error: {SURFACE_CMD_SIZE} bytes (esperados 82)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class Surface_cmdPacket:
    """Representación tipada de un KUNTUR_SURFACE_CMD ya validado."""
    t_sim_us: int
    seq_num: int
    status: int
    delax: float
    delex: float
    delrx: float
    throttle: float
    gainfb1: float
    gainfb2: float
    gainfb3: float
    wp_flag: float


def unpack_surface_cmd(data: bytes) -> Surface_cmdPacket:
    """Valida tamaño + magic y decodifica un KUNTUR_SURFACE_CMD."""
    if len(data) != SURFACE_CMD_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {SURFACE_CMD_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_SURFACE_CMD:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_SURFACE_CMD:08X}")

    (_magic, t_sim_us, seq_num, status, delax, delex, delrx, throttle, gainfb1, gainfb2, gainfb3, wp_flag) = struct.unpack(SURFACE_CMD_FMT, data)

    return Surface_cmdPacket(
        t_sim_us=t_sim_us,
        seq_num=seq_num,
        status=status,
        delax=delax,
        delex=delex,
        delrx=delrx,
        throttle=throttle,
        gainfb1=gainfb1,
        gainfb2=gainfb2,
        gainfb3=gainfb3,
        wp_flag=wp_flag,
    )


def pack_surface_cmd(t_sim_us, seq_num, status, delax, delex, delrx, throttle, gainfb1, gainfb2, gainfb3, wp_flag) -> bytes:
    """Empaqueta un KUNTUR_SURFACE_CMD."""
    return struct.pack(SURFACE_CMD_FMT, MAGIC_SURFACE_CMD, t_sim_us, seq_num, status, delax, delex, delrx, throttle, gainfb1, gainfb2, gainfb3, wp_flag)

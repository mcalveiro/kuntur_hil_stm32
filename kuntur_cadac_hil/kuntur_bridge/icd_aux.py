"""
icd_aux.py -- Canal auxiliar KUNTUR_AUX (schema v1.0).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3
(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema
de origen de este archivo para transporte/decimación/propósito.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC_AUX = 0x4B415831  # "KAX1"

AUX_FMT = "<IQIfffffffffff"
AUX_SIZE = struct.calcsize(AUX_FMT)

assert AUX_SIZE == 60, f"AUX_FMT error: {AUX_SIZE} bytes (esperados 60)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class AuxPacket:
    """Representación tipada de un KUNTUR_AUX ya validado."""
    t_sim_us: int
    seq_num: int
    dvbe: float
    vmach: float
    pdynmc: float
    hbe: float
    altrate: float
    alppx: float
    betax: float
    anx: float
    throttle: float
    power: float
    thrust: float


def unpack_aux(data: bytes) -> AuxPacket:
    """Valida tamaño + magic y decodifica un KUNTUR_AUX."""
    if len(data) != AUX_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {AUX_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_AUX:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_AUX:08X}")

    (_magic, t_sim_us, seq_num, dvbe, vmach, pdynmc, hbe, altrate, alppx, betax, anx, throttle, power, thrust) = struct.unpack(AUX_FMT, data)

    return AuxPacket(
        t_sim_us=t_sim_us,
        seq_num=seq_num,
        dvbe=dvbe,
        vmach=vmach,
        pdynmc=pdynmc,
        hbe=hbe,
        altrate=altrate,
        alppx=alppx,
        betax=betax,
        anx=anx,
        throttle=throttle,
        power=power,
        thrust=thrust,
    )


def pack_aux(t_sim_us, seq_num, dvbe, vmach, pdynmc, hbe, altrate, alppx, betax, anx, throttle, power, thrust) -> bytes:
    """Empaqueta un KUNTUR_AUX."""
    return struct.pack(AUX_FMT, MAGIC_AUX, t_sim_us, seq_num, dvbe, vmach, pdynmc, hbe, altrate, alppx, betax, anx, throttle, power, thrust)

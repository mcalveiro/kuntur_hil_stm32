"""
icd_plant_state.py -- Canal auxiliar KUNTUR_PLANT_STATE (schema v1.0).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3
(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema
de origen de este archivo para transporte/decimación/propósito.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC_PLANT_STATE = 0x4B505331  # "KPS1"

PLANT_STATE_FMT = "<IQIHdddddddddddddddddddddddddddddddddddddddddHHH"
PLANT_STATE_SIZE = struct.calcsize(PLANT_STATE_FMT)

assert PLANT_STATE_SIZE == 352, f"PLANT_STATE_FMT error: {PLANT_STATE_SIZE} bytes (esperados 352)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class Plant_statePacket:
    """Representación tipada de un KUNTUR_PLANT_STATE ya validado."""
    t_sim_us: int
    seq_num: int
    flags: int
    sn: float
    se: float
    sd: float
    vn: float
    ve: float
    vd: float
    phi: float
    theta: float
    psi: float
    p: float
    q: float
    r: float
    ax: float
    ay: float
    az: float
    pdynmc: float
    vmach: float
    dvbe: float
    grav: float
    alpha: float
    beta: float
    dla: float
    dlde: float
    dma: float
    dmq: float
    dmde: float
    dyb: float
    dydr: float
    dnb: float
    dnr: float
    dndr: float
    dllp: float
    dllda: float
    gmax: float
    gminx: float
    swel1: float
    swel2: float
    swel3: float
    psiflx: float
    thtflx: float
    altcom: float
    mguid: int
    maut: int
    manual_active: int


def unpack_plant_state(data: bytes) -> Plant_statePacket:
    """Valida tamaño + magic y decodifica un KUNTUR_PLANT_STATE."""
    if len(data) != PLANT_STATE_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {PLANT_STATE_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_PLANT_STATE:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_PLANT_STATE:08X}")

    (_magic, t_sim_us, seq_num, flags, sn, se, sd, vn, ve, vd, phi, theta, psi, p, q, r, ax, ay, az, pdynmc, vmach, dvbe, grav, alpha, beta, dla, dlde, dma, dmq, dmde, dyb, dydr, dnb, dnr, dndr, dllp, dllda, gmax, gminx, swel1, swel2, swel3, psiflx, thtflx, altcom, mguid, maut, manual_active) = struct.unpack(PLANT_STATE_FMT, data)

    return Plant_statePacket(
        t_sim_us=t_sim_us,
        seq_num=seq_num,
        flags=flags,
        sn=sn,
        se=se,
        sd=sd,
        vn=vn,
        ve=ve,
        vd=vd,
        phi=phi,
        theta=theta,
        psi=psi,
        p=p,
        q=q,
        r=r,
        ax=ax,
        ay=ay,
        az=az,
        pdynmc=pdynmc,
        vmach=vmach,
        dvbe=dvbe,
        grav=grav,
        alpha=alpha,
        beta=beta,
        dla=dla,
        dlde=dlde,
        dma=dma,
        dmq=dmq,
        dmde=dmde,
        dyb=dyb,
        dydr=dydr,
        dnb=dnb,
        dnr=dnr,
        dndr=dndr,
        dllp=dllp,
        dllda=dllda,
        gmax=gmax,
        gminx=gminx,
        swel1=swel1,
        swel2=swel2,
        swel3=swel3,
        psiflx=psiflx,
        thtflx=thtflx,
        altcom=altcom,
        mguid=mguid,
        maut=maut,
        manual_active=manual_active,
    )


def pack_plant_state(t_sim_us, seq_num, flags, sn, se, sd, vn, ve, vd, phi, theta, psi, p, q, r, ax, ay, az, pdynmc, vmach, dvbe, grav, alpha, beta, dla, dlde, dma, dmq, dmde, dyb, dydr, dnb, dnr, dndr, dllp, dllda, gmax, gminx, swel1, swel2, swel3, psiflx, thtflx, altcom, mguid, maut, manual_active) -> bytes:
    """Empaqueta un KUNTUR_PLANT_STATE."""
    return struct.pack(PLANT_STATE_FMT, MAGIC_PLANT_STATE, t_sim_us, seq_num, flags, sn, se, sd, vn, ve, vd, phi, theta, psi, p, q, r, ax, ay, az, pdynmc, vmach, dvbe, grav, alpha, beta, dla, dlde, dma, dmq, dmde, dyb, dydr, dnb, dnr, dndr, dllp, dllda, gmax, gminx, swel1, swel2, swel3, psiflx, thtflx, altcom, mguid, maut, manual_active)

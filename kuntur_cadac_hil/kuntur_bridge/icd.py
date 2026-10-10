"""
icd.py -- Contrato Kuntur-IC v1.3 (KUNTUR_STATES / KUNTUR_COMMAND).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py desde icd_schema/kuntur_ic_v1_3.json.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from .config import ExecMode

MAGIC_STATES = 0x4B535433   # "KST3"
MAGIC_COMMAND = 0x434D4432  # "CMD2"

STATES_FMT = "<IQIffffffffffffffffffHH"
STATES_SIZE = struct.calcsize(STATES_FMT)

COMMAND_FMT = "<IQfffffH"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

assert STATES_SIZE == 92, f"STATES_FMT error: {STATES_SIZE} bytes (esperados 92)"
assert COMMAND_SIZE == 34, f"COMMAND_FMT error: {COMMAND_SIZE} bytes (esperados 34)"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class StatesPacket:
    """Representación tipada de un KUNTUR_STATES ya validado."""
    t_sim_us: int
    seq_num: int
    north: float
    east: float
    down: float
    phi: float
    theta: float
    psi: float
    p: float
    q: float
    r: float
    ax: float
    ay: float
    az: float
    vn: float
    ve: float
    vd: float
    ias: float
    alpha: float
    beta: float
    status: int


def unpack_states(data: bytes) -> StatesPacket:
    """Valida tamaño + magic y decodifica un KUNTUR_STATES."""
    if len(data) != STATES_SIZE:
        raise ICDFrameError(f"tamaño {len(data)} != {STATES_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_STATES:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_STATES:08X}")

    (_magic, t_sim_us, seq_num, north, east, down, phi, theta, psi, p, q, r, ax, ay, az, vn, ve, vd, ias, alpha, beta, status, _pad) = struct.unpack(STATES_FMT, data)

    return StatesPacket(
        t_sim_us=t_sim_us,
        seq_num=seq_num,
        north=north,
        east=east,
        down=down,
        phi=phi,
        theta=theta,
        psi=psi,
        p=p,
        q=q,
        r=r,
        ax=ax,
        ay=ay,
        az=az,
        vn=vn,
        ve=ve,
        vd=vd,
        ias=ias,
        alpha=alpha,
        beta=beta,
        status=status,
    )


def pack_command(t_sim_us_grant, elevator, aileron, rudder, throttle, ground_elev_m, exec_mode) -> bytes:
    """Empaqueta un KUNTUR_COMMAND (Grant + actuadores + terreno + modo)."""
    return struct.pack(COMMAND_FMT, MAGIC_COMMAND, t_sim_us_grant, elevator, aileron, rudder, throttle, ground_elev_m, int(exec_mode))


@dataclass(frozen=True)
class CommandPacket:
    """Representacion tipada de un KUNTUR_COMMAND ya validado (lado planta)."""
    t_sim_us_grant: int
    elevator: float
    aileron: float
    rudder: float
    throttle: float
    ground_elev_m: float
    exec_mode: int


def unpack_command(data: bytes) -> CommandPacket:
    """Valida tamano + magic y decodifica un KUNTUR_COMMAND (lado planta,
    simetrico de unpack_states() -- lo que hoy hace kuntur_ic.cpp con los
    OUT_* al recibir desde KSE)."""
    if len(data) != COMMAND_SIZE:
        raise ICDFrameError(f"tamano {len(data)} != {COMMAND_SIZE} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_COMMAND:
        raise ICDFrameError(f"magic 0x{magic:08X} != 0x{MAGIC_COMMAND:08X}")

    (_magic, t_sim_us_grant, elevator, aileron, rudder, throttle, ground_elev_m, exec_mode) = struct.unpack(COMMAND_FMT, data)

    return CommandPacket(
        t_sim_us_grant=t_sim_us_grant,
        elevator=elevator,
        aileron=aileron,
        rudder=rudder,
        throttle=throttle,
        ground_elev_m=ground_elev_m,
        exec_mode=exec_mode,
    )


def pack_states(t_sim_us, seq_num, north, east, down, phi, theta, psi, p, q, r, ax, ay, az, vn, ve, vd, ias, alpha, beta, status) -> bytes:
    """Empaqueta un KUNTUR_STATES (lado planta -> KSE). pos_ned viaja
    CRUDO (passthrough) -- la proyeccion WGS-84 es responsabilidad
    exclusiva de geodesy.SACOLocalFrame del lado Python de Kuntur-Bridge,
    nunca de quien genera este paquete (misma regla que hoy respeta
    kuntur_ic.cpp). accel_body debe venir ya convertido a fuerza
    especifica (f_body = A_bb - g_body) por quien llama a esta funcion --
    ver icd_helpers.specific_force_body() en el paquete cadac_plant para
    la misma transformacion que hace kuntur_ic.cpp."""
    return struct.pack(STATES_FMT, MAGIC_STATES, t_sim_us, seq_num, north, east, down, phi, theta, psi, p, q, r, ax, ay, az, vn, ve, vd, ias, alpha, beta, status, 0)

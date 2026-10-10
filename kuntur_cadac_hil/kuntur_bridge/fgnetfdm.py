"""fgnetfdm.py -- Empaquetado FGNetFDM v24 (408 bytes, big-endian).

Formato verificado contra la definición autoritativa de flightgear-python
(fdm_v24.fdm_struct, que espeja net_fdm.hxx). El struct real tiene 8
arrays de 4 floats para parámetros de motor (fuel_flow, fuel_px, egt,
cht, mp_osi, tit, oil_temp, oil_px), no 9 -- bug histórico ya corregido.
"""
from __future__ import annotations

import struct

_FMT = (
    "!"
    "I4s"
    "ddd"
    + "f" * 6
    + "f" * 16
    + "I"
    + "I" * 4
    + "f" * 4
    + "f" * 32   # 8 arrays de 4 floats (corregido; no 9)
    + "I"
    + "f" * 4
    + "I"
    + "I" * 3
    + "f" * 9
    + "Ii"
    + "f"
    + "f" * 10
)
_SIZE = struct.calcsize(_FMT)
assert _SIZE == 408, f"FGNetFDM: {_SIZE} bytes (esperados 408)"


def pack_fg_net_fdm(
    lon_rad: float, lat_rad: float, alt_m: float,
    phi_rad: float, theta_rad: float, psi_rad: float,
    elev: float, ail: float, rud: float, throt: float,
) -> bytes:
    data = struct.pack(
        _FMT,
        24, b"\x00\x00\x00\x00",
        lon_rad, lat_rad, alt_m,
        0.0, phi_rad, theta_rad, psi_rad, 0.0, 0.0,
        *([0.0] * 16),
        1, 2, 0, 0, 0,
        throt * 2700, 0, 0, 0,
        *([0.0] * 32),
        1, 100.0, 0.0, 0.0, 0.0,
        3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0, 0, 0.0,
        elev, 0.0, 0.0, 0.0, ail, ail, rud, 0.0, 0.0, 0.0,
    )
    assert len(data) == 408, f"FDM packet: {len(data)} bytes (esperados 408)"
    return data

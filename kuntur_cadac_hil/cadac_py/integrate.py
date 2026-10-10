"""
Port of the integration and US76-atmosphere free functions in
'utility_header.hpp' / 'utility_functions.cpp'.

050203 integrate() - Modified Euler-Midpoint method (Peter H Zipfel)
030318 atmosphere76() - US Standard Atmosphere 1976 (Peter H Zipfel)
"""
from __future__ import annotations

from typing import overload

from .matrix import Matrix


def integrate_scalar(dydx_new: float, dydx: float, y: float, int_step: float) -> float:
    """Scalar overload of integrate(): Modified Euler-Midpoint method.

    Example (first order lag):
        phid_new = (phic - phi) / tphi
        phi = integrate_scalar(phid_new, phid, phi, int_step)
        phid = phid_new
    """
    return y + (dydx_new + dydx) * int_step / 2.0


def integrate_matrix(dydx_new: Matrix, dydx: Matrix, y: Matrix, int_step: float) -> Matrix:
    """Matrix overload of integrate(): element-wise Euler-Midpoint method."""
    nrow, ncol = y.get_rows(), y.get_cols()
    if (dydx_new.get_rows() != nrow or dydx.get_rows() != nrow
            or dydx_new.get_cols() != ncol or dydx.get_cols() != ncol):
        raise ValueError("incompatible dimensions in integrate()")

    result = Matrix(nrow, ncol)
    for r in range(nrow):
        for c in range(ncol):
            result.assign_loc(
                r, c,
                integrate_scalar(dydx_new.get_loc(r, c), dydx.get_loc(r, c),
                                  y.get_loc(r, c), int_step),
            )
    return result


def integrate(dydx_new, dydx, y, int_step):
    """Dispatches to the scalar or Matrix overload, mirroring the C++
    overloaded free function 'integrate(...)' used throughout the modules.
    """
    if isinstance(y, Matrix):
        return integrate_matrix(dydx_new, dydx, y, int_step)
    return integrate_scalar(dydx_new, dydx, y, int_step)


# --------------------------------------------------------------------- #
# US Standard Atmosphere 1976
# --------------------------------------------------------------------- #
import math

_REARTH_KM = 6369.0
_GMR = 34.163195
_RHOSL = 1.22500
_PRESSL = 101325.0
_TEMPKSL = 288.15

_HTAB = (0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852)
_TTAB = (288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946)
_PTAB = (1.0, 2.233611e-1, 5.403295e-2, 8.5666784e-3, 1.0945601e-3,
         6.6063531e-4, 3.9046834e-5, 3.68501e-6)
_GTAB = (-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0)


def atmosphere76(balt: float) -> tuple[float, float, float]:
    """US Standard Atmosphere 1976, up to 85 km (extrapolated beyond 71 km).

    Args:
        balt: geometric altitude above sea level - m

    Returns:
        (rho, press, tempk): air density [kg/m^3], static pressure [Pa],
        temperature [degK]
    """
    alt = balt / 1000.0
    h = alt * _REARTH_KM / (alt + _REARTH_KM)

    # binary search: table entry i below actual altitude
    i, j = 0, 7
    while True:
        k = (i + j) // 2
        if h < _HTAB[k]:
            j = k
        else:
            i = k
        if j <= i + 1:
            break

    tgrad = _GTAB[i]
    tbase = _TTAB[i]
    deltah = h - _HTAB[i]
    tlocal = tbase + tgrad * deltah
    theta = tlocal / _TTAB[0]

    if tgrad == 0:
        delta = _PTAB[i] * math.exp(-_GMR * deltah / tbase)
    else:
        delta = _PTAB[i] * (tbase / tlocal) ** (_GMR / tgrad)

    sigma = delta / theta

    rho = _RHOSL * sigma
    press = _PRESSL * delta
    tempk = _TEMPKSL * theta
    return rho, press, tempk

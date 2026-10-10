"""
Port of 'class_hierarchy.hpp' + 'class_functions.cpp' -> base classes
'Cadac', 'Flat6', 'Flat3'.

Original: Peter H Zipfel, CADAC++

Fidelity notes:
- The C++ code stores module-variables in fixed-size arrays (flat6[NFLAT6],
  missile[NMISSILE], flat3[NFLAT3], target[NTARGET]) addressed by literal
  integer offsets assigned by convention in each module's def_xxx()
  (e.g. 'newton' owns flat6[200-249], 'kinematics' owns flat6[100-149]).
  This port keeps that exact indexing scheme -- every module method below
  reads/writes the same numeric offsets as the .cpp source -- because the
  offsets are load-bearing: cross-module communication, the input-file
  parser and the documentation generator all key off them.
- Python has no header/implementation split, so def_xxx()/xxx() pairs that
  were 'virtual' in C++ are declared here as ordinary methods on Flat6 and
  overridden by the vehicle-specific subclass (Missile) exactly like the
  C++ vtable would dispatch them.
"""
from __future__ import annotations

import math
from typing import List

from .core import Variable, Event
from .matrix import Matrix, mat2tr, mat3tr
from .integrate import integrate, atmosphere76

NFLAT6 = 250
NMISSILE = 750
NFLAT3 = 40
NTARGET = 50
NEVENT = 20

REARTH = 6370987.308
WEII3 = 7.292115e-5
RAD = 0.0174532925199432
DEG = 57.2957795130823
PI = 3.1415927
AGRAV = 9.80675445
G = 6.673e-11
EARTH_MASS = 5.973e24
R = 287.053
EPS = 1.0e-10
SMALL = 1.0e-7


def _new_variable_array(size: int) -> List[Variable]:
    arr = [Variable() for _ in range(size)]
    for v in arr:
        v.init("empty", 0.0, " ", " ", " ", " ")
    return arr


class Cadac:
    """Abstract base class 'Cadac' (class_hierarchy.hpp)."""

    def __init__(self):
        self.name: str = ""
        self.event_epoch: bool = False
        # module-variable arrays -- present only on the branch that uses them
        self.flat6: List[Variable] = []
        self.flat3: List[Variable] = []
        self.missile: List[Variable] = []
        self.target: List[Variable] = []

    def set_name(self, vehicle_name: str) -> None:
        self.name = vehicle_name

    def get_vname(self) -> str:
        return self.name


class Flat6(Cadac):
    """Derived class 'Flat6' (class_hierarchy.hpp / environment.cpp /
    kinematics.cpp / newton.cpp / euler.cpp).

    Models atmosphere, gravitational acceleration and the 6-DoF equations
    of motion over a flat, non-rotating Earth. Shared by every 6-DoF
    CADAC vehicle (not just SRAAM6) -- this is the generic part of the
    architecture.
    """

    def __init__(self):
        super().__init__()
        self.flat6 = _new_variable_array(NFLAT6)

    # ------------------------------------------------------------------ #
    # environment.cpp
    # ------------------------------------------------------------------ #
    def def_environment(self) -> None:
        f = self.flat6
        f[52].init("press", 0.0, "Atmospheric pressure - Pa", "environment", "out", "")
        f[53].init("rho", 0.0, "Atmospheric density - kg/m^3", "environment", "out", "")
        f[54].init("vsound", 0.0, "Sonic speed - m/sec", "environment", "diag", "")
        f[55].init("grav", 0.0, "Gravity acceleration - m/s^2", "environment", "out", "")
        f[56].init("vmach", 0.0, "Mach number of missile", "environment", "out", "scrn,plot,com")
        f[57].init("pdynmc", 0.0, "Dynamic pressure - Pa", "environment", "out", "plot")
        f[58].init("tempk", 0.0, "Atmospheric temperature - K", "environment", "out", "")
        f[59].init("mfreeze_environ", "int", 0, "Saving 'mfreeze' value", "environment", "save", "")
        f[60].init("pdynmcf", 0.0, "Saved dyn.press. when mfreeze=1 - Pa", "environment", "save", "")
        f[61].init("vmachf", 0.0, "Saved Mach# when mfreeze=1 - Pa", "environment", "save", "")

    def environment(self) -> None:
        f, m = self.flat6, self.missile
        # localizing module-variables
        mfreeze_environ = f[59].integer()
        pdynmcf = f[60].real()
        vmachf = f[61].real()
        dvbe = f[236].real()
        hbe = f[239].real()
        trcode = m[180].real()
        trmach = m[183].real()
        trdynm = m[184].real()
        mguid = m[400].integer()
        mfreeze = m[501].integer()
        # ---------------------------------------------------------------
        rad = REARTH + hbe
        grav = G * EARTH_MASS / rad ** 2

        rho, press, tempk = atmosphere76(hbe)
        vsound = (1.4 * R * tempk) ** 0.5
        vmach = abs(dvbe / vsound)
        pdynmc = 0.5 * rho * dvbe * dvbe

        if mguid == 6:
            if vmach <= trmach:
                trcode = 2.0
            if pdynmc <= trdynm:
                trcode = 3.0

        if mfreeze == 0:
            mfreeze_environ = 0
        else:
            if mfreeze != mfreeze_environ:
                mfreeze_environ = mfreeze
                vmachf = vmach
                pdynmcf = pdynmc
            vmach = vmachf
            pdynmc = pdynmcf
        # ---------------------------------------------------------------
        f[59].gets(mfreeze_environ)
        f[60].gets(pdynmcf)
        f[61].gets(vmachf)
        f[52].gets(press)
        f[53].gets(rho)
        f[55].gets(grav)
        f[56].gets(vmach)
        f[57].gets(pdynmc)
        f[58].gets(tempk)
        f[54].gets(vsound)
        m[180].gets(trcode)

    # ------------------------------------------------------------------ #
    # kinematics.cpp
    # ------------------------------------------------------------------ #
    def def_kinematics(self) -> None:
        f = self.flat6
        f[101].init("ck", 50.0, "Quaternion orthogonalizing factor ", "kinematics", "data", "")
        f[110].init("q0d", 0.0, "Quaternion derivative, 0-th component", "kinematics", "state", "")
        f[111].init("q0", 0.0, "Quaternion, 0-th component", "kinematics", "state", "")
        f[112].init("q1d", 0.0, "Quaternion derivative, 1-th component", "kinematics", "state", "")
        f[113].init("q1", 0.0, "Quaternion, 1-th component", "kinematics", "state", "")
        f[114].init("q2d", 0.0, "Quaternion derivative, 2-th component", "kinematics", "state", "")
        f[115].init("q2", 0.0, "Quaternion, 2-th component", "kinematics", "state", "")
        f[116].init("q3d", 0.0, "Quaternion derivative, 3-th component", "kinematics", "state", "")
        f[117].init("q3", 0.0, "Quaternion, 3-th component", "kinematics", "state", "")
        f[120].init("TBL", 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    "T.M. of body wrt local level coord", "kinematics", "out", "")
        f[134].init("psibl", 0.0, "Yawing angle of vehicle - rad", "kinematics", "diag", "")
        f[135].init("thtbl", 0.0, "Pitching angle of vehicle - rad", "kinematics", "diag", "")
        f[136].init("phibl", 0.0, "Rolling angle of vehicle - rad", "kinematics", "diag", "")
        f[137].init("psiblx", 0.0, "G Yawing angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[138].init("thtblx", 0.0, "G Pitching angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[139].init("phiblx", 0.0, "G Rolling angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[140].init("alppx", 0.0, "Total angle of attack - deg", "kinematics", "out", "scrn,plot")
        f[141].init("phipx", 0.0, "Aerodynamic roll angle - deg", "kinematics", "out", "scrn,plot")
        f[142].init("alpp", 0.0, "Total angle of attack - rad", "kinematics", "out", "")
        f[143].init("phip", 0.0, "Aerodynamic roll angle - rad", "kinematics", "out", "")
        f[144].init("alphax", 0.0, "Angle of attack - deg", "kinematics", "diag", "plot")
        f[145].init("betax", 0.0, "Side slip angle - deg", "kinematics", "diag", "plot")
        f[146].init("erq", 0.0, "Error of quaternion nonorthogonality", "kinematics", "diag", "plot")
        f[147].init("etbl", 0.0, "Error of D.C.M. nonorthogonality", "kinematics", "diag", "plot")
        f[148].init("TLB", 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    "T.M. of local level wrt body coord", "kinematics", "diag", "")

    def init_kinematics(self) -> None:
        f = self.flat6
        psiblx = f[137].real()
        thtblx = f[138].real()
        phiblx = f[139].real()
        # -----------------------------------------------------------
        spsi = math_sin(psiblx / (2.0 * DEG))
        cpsi = math_cos(psiblx / (2.0 * DEG))
        stht = math_sin(thtblx / (2.0 * DEG))
        ctht = math_cos(thtblx / (2.0 * DEG))
        sphi = math_sin(phiblx / (2.0 * DEG))
        cphi = math_cos(phiblx / (2.0 * DEG))

        q0 = cpsi * ctht * cphi + spsi * stht * sphi
        q1 = cpsi * ctht * sphi - spsi * stht * cphi
        q2 = cpsi * stht * cphi + spsi * ctht * sphi
        q3 = -cpsi * stht * sphi + spsi * ctht * cphi

        TBL = mat3tr(psiblx / DEG, thtblx / DEG, phiblx / DEG)
        # -----------------------------------------------------------
        f[111].gets(q0)
        f[113].gets(q1)
        f[115].gets(q2)
        f[117].gets(q3)
        f[120].gets_mat(TBL)

    def kinematics(self, int_step: float) -> None:
        f, m = self.flat6, self.missile
        ck = f[101].real()
        time = f[0].real()
        trcode = m[180].real()
        tralp = m[186].real()
        VBEB = f[213].vec()
        pp = f[155].real()
        qq = f[157].real()
        rr = f[159].real()
        q0d = f[110].real(); q0 = f[111].real()
        q1d = f[112].real(); q1 = f[113].real()
        q2d = f[114].real(); q2 = f[115].real()
        q3d = f[116].real(); q3 = f[117].real()
        # -----------------------------------------------------------
        quat_metric = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3
        erq = 1.0 - quat_metric

        new_q0d = 0.5 * (-pp * q1 - qq * q2 - rr * q3) + ck * erq * q0
        new_q1d = 0.5 * (pp * q0 + rr * q2 - qq * q3) + ck * erq * q1
        new_q2d = 0.5 * (qq * q0 - rr * q1 + pp * q3) + ck * erq * q2
        new_q3d = 0.5 * (rr * q0 + qq * q1 - pp * q2) + ck * erq * q3

        q0 = integrate(new_q0d, q0d, q0, int_step)
        q1 = integrate(new_q1d, q1d, q1, int_step)
        q2 = integrate(new_q2d, q2d, q2, int_step)
        q3 = integrate(new_q3d, q3d, q3, int_step)
        q0d, q1d, q2d, q3d = new_q0d, new_q1d, new_q2d, new_q3d

        TBL = Matrix(3, 3)
        TBL.assign_loc(0, 0, q0*q0 + q1*q1 - q2*q2 - q3*q3)
        TBL.assign_loc(0, 1, 2.0 * (q1*q2 + q0*q3))
        TBL.assign_loc(0, 2, 2.0 * (q1*q3 - q0*q2))
        TBL.assign_loc(1, 0, 2.0 * (q1*q2 - q0*q3))
        TBL.assign_loc(1, 1, q0*q0 - q1*q1 + q2*q2 - q3*q3)
        TBL.assign_loc(1, 2, 2.0 * (q2*q3 + q0*q1))
        TBL.assign_loc(2, 0, 2.0 * (q1*q3 + q0*q2))
        TBL.assign_loc(2, 1, 2.0 * (q2*q3 - q0*q1))
        TBL.assign_loc(2, 2, q0*q0 - q1*q1 - q2*q2 + q3*q3)

        TLB = TBL.trans()
        UBL = TLB * TBL
        e1 = UBL.get_loc(0, 0) - 1.0
        e2 = UBL.get_loc(1, 1) - 1.0
        e3 = UBL.get_loc(2, 2) - 1.0
        etbl = (e1*e1 + e2*e2 + e3*e3) ** 0.5

        tbl13 = TBL.get_loc(0, 2)
        tbl11 = TBL.get_loc(0, 0)
        tbl33 = TBL.get_loc(2, 2)
        tbl12 = TBL.get_loc(0, 1)
        tbl23 = TBL.get_loc(1, 2)

        if abs(tbl13) < 1.0:
            thtbl = math_asin(-tbl13)
            cthtbl = math_cos(thtbl)
        else:
            thtbl = PI / 2.0 * (1 if -tbl13 >= 0 else -1)
            cthtbl = EPS

        cpsi_ = tbl11 / cthtbl
        if abs(cpsi_) >= 1:
            cpsi_ = (1 - EPS) * (1 if cpsi_ >= 0 else -1)
        cphi_ = tbl33 / cthtbl
        if abs(cphi_) >= 1:
            cphi_ = (1 - EPS) * (1 if cphi_ >= 0 else -1)

        psibl = math_acos(cpsi_) * (1 if tbl12 >= 0 else -1)
        phibl = math_acos(cphi_) * (1 if tbl23 >= 0 else -1)

        psiblx = DEG * psibl
        thtblx = DEG * thtbl
        phiblx = DEG * phibl

        vbeb1 = VBEB.get_loc(0, 0)
        vbeb2 = VBEB.get_loc(1, 0)
        vbeb3 = VBEB.get_loc(2, 0)

        alpha = math_atan2(vbeb3, vbeb1)
        dvbe = VBEB.absolute()
        beta = math_asin(vbeb2 / dvbe) if dvbe != 0 else 0.0

        dum = vbeb1 / dvbe if dvbe != 0 else 0.0
        if abs(dum) > 1:
            dum = 1 * (1 if dum >= 0 else -1)
        alpp = math_acos(dum)
        if vbeb2 == 0 and vbeb3 == 0:
            phip = 0.0
        elif abs(vbeb2) < EPS:
            phip = 0.0 if vbeb3 > 0 else PI
        else:
            phip = math_atan2(vbeb2, vbeb3)

        alphax = alpha * DEG
        betax = beta * DEG
        alppx = alpp * DEG
        phipx = phip * DEG

        if alpp > tralp:
            trcode = 5.0
        # -----------------------------------------------------------
        f[110].gets(q0d); f[111].gets(q0)
        f[112].gets(q1d); f[113].gets(q1)
        f[114].gets(q2d); f[115].gets(q2)
        f[116].gets(q3d); f[117].gets(q3)
        f[120].gets_mat(TBL)
        f[140].gets(alppx)
        f[141].gets(phipx)
        f[142].gets(alpp)
        f[143].gets(phip)
        f[134].gets(psibl)
        f[135].gets(thtbl)
        f[136].gets(phibl)
        f[137].gets(psiblx)
        f[138].gets(thtblx)
        f[139].gets(phiblx)
        f[144].gets(alphax)
        f[145].gets(betax)
        f[146].gets(erq)
        f[147].gets(etbl)
        f[148].gets_mat(TLB)
        m[180].gets(trcode)

    # ------------------------------------------------------------------ #
    # newton.cpp
    # ------------------------------------------------------------------ #
    def def_newton(self) -> None:
        f = self.flat6
        f[0].init("time", 0.0, "Simulation time - sec", "newton", "exec", "scrn,plot,com")
        f[1].init("halt", "int", 0, "=1: vehicle object is halted - ND", "newton", "exec", "")
        f[210].init("VBEBD", 0, 0, 0, " Derivative of missile velocity - m/s^2", "newton", "state", "")
        f[213].init("VBEB", 0, 0, 0, "Missile velocity in body axes - m/s", "newton", "state", "")
        f[216].init("SBELD", 0, 0, 0, "Derivative of missile position - m/s", "newton", "state", "")
        f[219].init("SBEL", 0, 0, 0, "Missile pos. wrt point E in local level axes - m", "newton", "state", "plot,com")
        f[220].init("sbel1", 0.0, "Initial north comp of SBEL - m", "newton", "data", "")
        f[221].init("sbel2", 0.0, "Initial east comp of SBEL - m", "newton", "data", "")
        f[222].init("sbel3", 0.0, "Initial down comp of SBEL - m", "newton", "data", "")
        f[223].init("trackd", 0.0, "Ground track derivative - m", "newton", "state", "")
        f[224].init("track", 0.0, "Ground track - m", "newton", "state", "plot")
        f[230].init("FSPB", 0, 0, 0, "Specific force in body axes - m/s^2", "newton", "out", "")
        f[233].init("VBEL", 0, 0, 0, "Missile velocity in local level axes - m/s", "newton", "out", "com,scrn,plot")
        f[236].init("dvbe", 0.0, "Missile speed - m/s", "newton", "in/out", "plot")
        f[237].init("alpha0x", 0.0, "Initial angle-of-attack - deg", "newton", "data", "")
        f[238].init("beta0x", 0.0, "Initial side slip angle - deg", "newton", "data", "")
        f[239].init("hbe", 0.0, "Height above ground - m", "newton", "out", "scrn,plot")
        f[240].init("psivlx", 0.0, "Heading angle - deg", "newton", "diag", "scrn,plot")
        f[241].init("thtvlx", 0.0, "Vertical flight path angle - deg", "newton", "diag", "scrn,plot")
        f[244].init("anx", 0.0, "Normal specific force component - g's", "newton", "diag", "scrn,plot,com")
        f[245].init("ayx", 0.0, "Side specific force component - g's", "newton", "diag", "scrn,plot,com")
        f[246].init("ATB", 0, 0, 0, "Tangential accel, solving Newton's Law - m/s^2", "newton", "diag", "")
        f[247].init("mfreeze_newt", "int", 0, "Saving mfreze value - ND", "newton", "save", "")
        f[248].init("dvbef", 0.0, "Saved speed under mfreeze=1 - m/s", "newton", "save", "")

    def init_newton(self) -> None:
        f = self.flat6
        sbel1 = f[220].real()
        sbel2 = f[221].real()
        sbel3 = f[222].real()
        dvbe = f[236].real()
        alpha0x = f[237].real()
        beta0x = f[238].real()
        TBL = f[120].mat()
        # -----------------------------------------------------------
        salp = math_sin(alpha0x * RAD)
        calp = math_cos(alpha0x * RAD)
        sbet = math_sin(beta0x * RAD)
        cbet = math_cos(beta0x * RAD)
        VBEB = Matrix().build_vec3(calp * cbet * dvbe, sbet * dvbe, salp * cbet * dvbe)

        VBEL = (~TBL) * VBEB

        vbel1 = VBEL.get_loc(0, 0)
        vbel2 = VBEL.get_loc(1, 0)
        vbel3 = VBEL.get_loc(2, 0)
        if vbel1 == 0.0 and vbel2 == 0.0:
            psivl = 0.0
        else:
            psivl = math_atan2(vbel2, vbel1)
        thtvl = math_atan2(-vbel3, (vbel1*vbel1 + vbel2*vbel2) ** 0.5)
        psivlx = psivl * DEG
        thtvlx = thtvl * DEG

        SBEL = Matrix().build_vec3(sbel1, sbel2, sbel3)
        hbe = -SBEL.get_loc(2, 0)
        # -----------------------------------------------------------
        f[213].gets_vec(VBEB)
        f[219].gets_vec(SBEL)
        f[233].gets_vec(VBEL)
        f[239].gets(hbe)
        f[240].gets(psivlx)
        f[241].gets(thtvlx)

    def newton(self, sim_time: float, int_step: float) -> None:
        f, m = self.flat6, self.missile
        dvbe = f[236].real()
        mfreeze_newt = f[247].integer()
        dvbef = f[248].real()
        grav = f[55].real()
        TBL = f[120].mat()
        FAPB = f[200].vec()
        WBEB = f[163].vec()
        vmass = m[59].real()
        mfreeze = m[501].integer()
        VBEBD = f[210].vec()
        VBEB = f[213].vec()
        SBELD = f[216].vec()
        SBEL = f[219].vec()
        trackd = f[223].real()
        track = f[224].real()
        # -----------------------------------------------------------
        time = sim_time

        ATB = WBEB.skew_sym() * VBEB
        GRAVL = Matrix().build_vec3(0.0, 0.0, grav)

        FSPB = FAPB * (1.0 / vmass)
        VBEBD_NEW = FSPB - ATB + TBL * GRAVL
        VBEB = integrate(VBEBD_NEW, VBEBD, VBEB, int_step)
        VBEBD = VBEBD_NEW

        VBEL = (~TBL) * VBEB
        SBELD_NEW = VBEL
        SBEL = integrate(SBELD_NEW, SBELD, SBEL, int_step)
        SBELD = SBELD_NEW

        VGROUND = VBEL.copy()
        VGROUND[2] = 0.0
        vground = VGROUND.absolute()
        trackd_new = vground
        track = integrate(trackd_new, trackd, track, int_step)
        trackd = trackd_new

        vbel1 = VBEL.get_loc(0, 0)
        vbel2 = VBEL.get_loc(1, 0)
        vbel3 = VBEL.get_loc(2, 0)
        if vbel1 == 0.0 and vbel2 == 0.0:
            psivl = 0.0
        else:
            psivl = math_atan2(vbel2, vbel1)
        thtvl = math_atan2(-vbel3, (vbel1*vbel1 + vbel2*vbel2) ** 0.5)
        psivlx = psivl * DEG
        thtvlx = thtvl * DEG

        dvbe = VBEL.absolute()
        hbe = -SBEL.get_loc(2, 0)

        anx = -FSPB.get_loc(2, 0) / grav
        ayx = FSPB.get_loc(1, 0) / grav

        if mfreeze == 0:
            mfreeze_newt = 0
        else:
            if mfreeze != mfreeze_newt:
                mfreeze_newt = mfreeze
                dvbef = dvbe
            dvbe = dvbef
        # -----------------------------------------------------------
        f[210].gets_vec(VBEBD)
        f[213].gets_vec(VBEB)
        f[216].gets_vec(SBELD)
        f[219].gets_vec(SBEL)
        f[223].gets(trackd)
        f[224].gets(track)
        f[247].gets(mfreeze_newt)
        f[248].gets(dvbef)
        f[0].gets(time)
        f[230].gets_vec(FSPB)
        f[233].gets_vec(VBEL)
        f[236].gets(dvbe)
        f[239].gets(hbe)
        f[240].gets(psivlx)
        f[241].gets(thtvlx)
        f[244].gets(anx)
        f[245].gets(ayx)
        f[246].gets_vec(ATB)

    # ------------------------------------------------------------------ #
    # euler.cpp
    # ------------------------------------------------------------------ #
    def def_euler(self) -> None:
        f = self.flat6
        f[154].init("ppd", 0.0, "Body roll angular velocity derivative - rad/s^2", "euler", "state", "")
        f[155].init("pp", 0.0, "Body roll angular velocity - rad/s", "euler", "state", "")
        f[156].init("qqd", 0.0, "Body pitch angular velocity derivative - rad/s^2", "euler", "state", "")
        f[157].init("qq", 0.0, "Body pitch angular velocity - rad/s", "euler", "state", "")
        f[158].init("rrd", 0.0, "Body yaw angular velocity derivative - rad/s^2", "euler", "state", "")
        f[159].init("rr", 0.0, "Body yaw angular velocity - rad/s", "euler", "state", "")
        f[160].init("ppx", 0.0, "Body roll angular velocity in body axes - deg/s", "euler", "out", "plot")
        f[161].init("qqx", 0.0, "Body pitch angular velocity in body axes - deg/s", "euler", "out", "plot")
        f[162].init("rrx", 0.0, "Body yaw angular velocity in body axes - deg/s", "euler", "out", "plot")
        f[163].init("WBEB", 0, 0, 0, "Ang vel of veh wrt earth, body axes - rad/s", "euler", "diag", "")

    def euler(self, int_step: float) -> None:
        f, m = self.flat6, self.missile
        FMB = f[201].vec()
        ai11 = m[71].real()
        ai33 = m[72].real()
        ppd = f[154].real(); pp = f[155].real()
        qqd = f[156].real(); qq = f[157].real()
        rrd = f[158].real(); rr = f[159].real()
        # -----------------------------------------------------------
        fmb1 = FMB.get_loc(0, 0)
        fmb2 = FMB.get_loc(1, 0)
        fmb3 = FMB.get_loc(2, 0)

        ppd_new = fmb1 / ai11
        pp = integrate(ppd_new, ppd, pp, int_step)
        ppd = ppd_new

        qqd_new = ((ai33 - ai11) * pp * rr + fmb2) / ai33
        qq = integrate(qqd_new, qqd, qq, int_step)
        qqd = qqd_new

        rrd_new = (-(ai33 - ai11) * pp * qq + fmb3) / ai33
        rr = integrate(rrd_new, rrd, rr, int_step)
        rrd = rrd_new

        WBEB = Matrix().build_vec3(pp, qq, rr)

        ppx = pp * DEG
        qqx = qq * DEG
        rrx = rr * DEG
        # -----------------------------------------------------------
        f[154].gets(ppd); f[155].gets(pp)
        f[156].gets(qqd); f[157].gets(qq)
        f[158].gets(rrd); f[159].gets(rr)
        f[160].gets(ppx)
        f[161].gets(qqx)
        f[162].gets(rrx)
        f[163].gets_vec(WBEB)


class Flat3(Cadac):
    """Derived class 'Flat3' (class_hierarchy.hpp) -- 3-DoF point-mass
    equations of motion over a flat, non-rotating Earth. Used by the
    'Target' branch. def_environment/environment/def_newton/init_newton/
    newton ported 1:1 from flat3_modules.cpp; def_kinematics/kinematics/
    def_euler/euler are dummy no-ops in the original (Target doesn't
    model body rotation, only a banked-turn approximation in Target's
    own control() module).
    """

    def __init__(self):
        super().__init__()
        self.flat3 = _new_variable_array(NFLAT3)

    # ------------------------------------------------------------------ #
    # dummy returns of unused modules (class_hierarchy.hpp: Flat3)
    # ------------------------------------------------------------------ #
    def def_kinematics(self) -> None:
        pass

    def init_kinematics(self) -> None:
        pass

    def kinematics(self, int_step: float) -> None:
        pass

    def def_euler(self) -> None:
        pass

    def euler(self, int_step: float) -> None:
        pass

    # ------------------------------------------------------------------ #
    # flat3_modules.cpp  (Flat3::def_environment / environment)
    #   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_environment(self) -> None:
        f = self.flat3
        f[11].init("grav", 0.0, "Gravitational acceleration - m/s^2", "environment", "out", "")
        f[12].init("rho", 0.0, "Air density - kg/m^3", "environment", "out", "")
        f[13].init("pdynmc", 0.0, "Dynamic pressure - Pa", "environment", "out", "scrn,plot")
        f[14].init("mach", 0.0, "Mach number - ND", "environment", "out", "scrn,plot")
        f[15].init("vsound", 0.0, "Speed of sound - m/s", "environment", "diag", "")

    def environment(self) -> None:
        f = self.flat3
        dvae = f[25].real()
        SAEL = f[26].vec()
        # -----------------------------------------------------------
        alt = -SAEL.get_loc(2, 0)

        grav = G * EARTH_MASS / (REARTH + alt) ** 2

        rho, press, tempk = atmosphere76(alt)

        vsound = math.sqrt(1.4 * R * tempk)
        mach = abs(dvae / vsound)
        pdynmc = 0.5 * rho * dvae ** 2
        # -----------------------------------------------------------
        f[11].gets(grav)
        f[12].gets(rho)
        f[13].gets(pdynmc)
        f[14].gets(mach)
        f[15].gets(vsound)

    # ------------------------------------------------------------------ #
    # flat3_modules.cpp  (Flat3::def_newton / init_newton / newton)
    #   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_newton(self) -> None:
        f = self.flat3
        f[22].init("TAL", 0, 0, 0, 0, 0, 0, 0, 0, 0, "TM of aircraft wrt loc lev coor", "newton", "out", "")
        f[23].init("TAV", 0, 0, 0, 0, 0, 0, 0, 0, 0, "TM of aircraft wrt velocity coor", "newton", "diag", "")
        f[24].init("TVL", 0, 0, 0, 0, 0, 0, 0, 0, 0, "TM of vel coor wrt loc lev coor ", "newton", "diag", "")
        f[25].init("dvae", 0.0, "Aircraft speed - m/s", "newton", "init/out", "com")
        f[26].init("SAEL", 0, 0, 0, "Aircraft position wrt point E - m ", "newton", "state", "com,plot")
        f[27].init("VAEL", 0, 0, 0, "Aircraft velocity wrt Earth - m/s ", "newton", "state", "com")
        f[28].init("AAEL", 0, 0, 0, "Aircraft acceleration  wrt Earth - m/s^2 ", "newton", "state", "")
        f[29].init("psialx", 0.0, "Aircraft heading angle - deg", "newton", "init/diag", "com")
        f[30].init("thtalx", 0.0, "Aircraft flight path angle - deg", "newton", "init/diag", "com")
        f[31].init("sael1", 0.0, "Aircraft initial north position - m", "newton", "init", "")
        f[32].init("sael2", 0.0, "Aircraft initial east position - m", "newton", "init", "")
        f[33].init("sael3", 0.0, "Aircraft initial down position - m", "newton", "init", "")
        f[34].init("psial", 0.0, "Aircraft heading angle - rad", "newton", "out", "")
        f[35].init("thtal", 0.0, "Aircraft flight path angle - rad", "newton", "out", "")

    def init_newton(self) -> None:
        f = self.flat3
        phiavout = f[21].real()
        dvae = f[25].real()
        SAEL = f[26].vec()
        VAEL = f[27].vec()
        psialx = f[29].real()
        thtalx = f[30].real()
        sael1 = f[31].real()
        sael2 = f[32].real()
        sael3 = f[33].real()
        # -----------------------------------------------------------
        psial = psialx * RAD
        thtal = thtalx * RAD

        VAEL = VAEL.cart_from_pol(dvae, psial, thtal)

        TVL = mat2tr(psial, thtal)

        TAV = Matrix(3, 3).identity()
        cphi = math.cos(phiavout)
        sphi = math.sin(phiavout)
        TAV.assign_loc(1, 1, cphi)
        TAV.assign_loc(2, 2, cphi)
        TAV.assign_loc(1, 2, sphi)
        TAV.assign_loc(2, 1, -sphi)

        TAL = TAV * TVL

        SAEL = Matrix().build_vec3(sael1, sael2, sael3)
        # -----------------------------------------------------------
        f[26].gets_vec(SAEL)
        f[27].gets_vec(VAEL)
        f[22].gets_mat(TAL)
        f[23].gets_mat(TAV)
        f[24].gets_mat(TVL)

    def newton(self, sim_time: float, int_step: float) -> None:
        f = self.flat3
        FSPA = f[20].vec()
        grav = f[11].real()
        phiavout = f[21].real()
        TAL = f[22].mat()
        SAEL = f[26].vec()
        VAEL = f[27].vec()
        AAEL = f[28].vec()
        # -----------------------------------------------------------
        GRAVL = Matrix().build_vec3(0.0, 0.0, grav)

        TLA = TAL.trans()
        NEXT_ACC = TLA * FSPA + GRAVL
        NEXT_VEL = integrate(NEXT_ACC, AAEL, VAEL, int_step)
        SAEL = integrate(NEXT_VEL, VAEL, SAEL, int_step)
        AAEL = NEXT_ACC
        VAEL = NEXT_VEL

        POLAR = VAEL.pol_from_cart()
        dvae = POLAR.get_loc(0, 0)
        psial = POLAR.get_loc(1, 0)
        thtal = POLAR.get_loc(2, 0)
        TVL = mat2tr(psial, thtal)

        TAV = Matrix(3, 3).identity()
        cphi = math.cos(phiavout)
        sphi = math.sin(phiavout)
        TAV.assign_loc(1, 1, cphi)
        TAV.assign_loc(2, 2, cphi)
        TAV.assign_loc(1, 2, sphi)
        TAV.assign_loc(2, 1, -sphi)

        TAL = TAV * TVL

        psialx = psial * DEG
        thtalx = thtal * DEG
        # -----------------------------------------------------------
        f[26].gets_vec(SAEL)
        f[27].gets_vec(VAEL)
        f[28].gets_vec(AAEL)
        f[22].gets_mat(TAL)
        f[24].gets_mat(TVL)
        f[25].gets(dvae)
        f[34].gets(psial)
        f[35].gets(thtal)
        f[29].gets(psialx)
        f[30].gets(thtalx)


# --------------------------------------------------------------------- #
# thin math wrappers (keeps the module code above visually close to the
# C++ 'sin(...)'/'cos(...)' call sites instead of 'math.sin(...)')
# --------------------------------------------------------------------- #
import math as _math

math_sin = _math.sin
math_cos = _math.cos
math_asin = _math.asin
math_acos = _math.acos
math_atan2 = _math.atan2

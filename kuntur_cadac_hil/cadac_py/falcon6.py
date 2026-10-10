"""
Port of CADAC++ FALCON6 -- an F16-like fixed-wing aircraft, 6-DoF, flat
Earth (source: CADAC/FALCON6).

Fidelity notes:
- FALCON6 shares the general Cadac architecture (module-variable arrays
  addressed by literal integer offsets, same Matrix/Table/Datadeck
  framework, same def_xxx()/init_xxx()/xxx() module pattern) with
  SRAAM6 -- but its own 'environment'/'kinematics'/'newton'/'euler'
  module *implementations* are different C++ source files with a
  different physical model (its own simple wind filter, full 3x3
  inertia tensor + engine angular momentum in 'euler', no missile/
  target-specific handling). In the original C++, each CADAC project
  directory compiles its OWN standalone 'Flat6'/'Plane' (or 'Missile')
  classes -- the class *names* are shared by convention across
  projects, but the code is not. This module therefore defines its
  own 'Flat6'/'Plane' pair (distinct from cadac.py's Flat6, which is
  SRAAM6-specific) rather than inheriting/reusing it, mirroring that
  reality; only the framework primitives (Variable, Matrix, Table,
  integrate(), atmosphere76(), the numeric constants) are shared, since
  'utility_header.hpp'/'global_constants.hpp' ARE byte-identical
  between the two CADAC projects (verified by diff).
- FALCON6 is a single-vehicle simulation (an autonomous F16 flying
  line/waypoint guidance) -- there is no seeker.cpp/tvc.cpp/
  intercept.cpp/Target counterpart for it; 'combus' packets exist in
  the framework (some flat6/plane variables are tagged 'com') but are
  unused with only one vehicle in the scenario.
- Every method below was ported line-by-line, same module-variable
  indices (flat6[N]/plane[N]), same calculation order as the .cpp
  source, exactly like missile.py/target.py.
"""
from __future__ import annotations

import math
from typing import List

from .core import Variable
from .matrix import Matrix, mat2tr, mat3tr, sign
from .integrate import integrate, atmosphere76
from .cadac import (
    Cadac,
    _new_variable_array,
    RAD, DEG, PI, AGRAV, G, EARTH_MASS, R, EPS, SMALL, REARTH,
)

NFLAT6 = 250
NPLANE = 750

# global_constants.hpp (FALCON6) -- not previously needed by SRAAM6
FOOT = 3.280834   # conversion factor m -> ft
NT = 4.448        # conversion factor lb -> N


class Flat6(Cadac):
    """Derived class 'Flat6' (class_hierarchy.hpp / environment.cpp /
    kinematics.cpp / newton.cpp / euler.cpp of CADAC/FALCON6) --
    atmosphere+wind, quaternion kinematics, translational and
    rotational (full inertia tensor) equations of motion."""

    def __init__(self):
        super().__init__()
        self.flat6: List[Variable] = _new_variable_array(NFLAT6)

    # ------------------------------------------------------------------ #
    # environment.cpp -- flat6[50-99]
    # ------------------------------------------------------------------ #
    def def_environment(self) -> None:
        f = self.flat6
        f[50].init("mwind", "int", 0, "=0: no wind; =1:const; =2:shear", "environment", "data", "")
        f[52].init("press", 0.0, "Atmospheric pressure - Pa", "environment", "out", "")
        f[53].init("rho", 0.0, "Atmospheric density - kg/m^3", "environment", "out", "")
        f[54].init("vsound", 0.0, "Sonic speed - m/sec", "environment", "diag", "")
        f[55].init("grav", 0.0, "Gravity acceleration - m/s^2", "environment", "out", "")
        f[56].init("vmach", 0.0, "Mach number of plane", "environment", "out", "scrn,plot,com")
        f[57].init("pdynmc", 0.0, "Dynamic pressure - Pa", "environment", "out", "scrn,plot")
        f[58].init("tempk", 0.0, "Atmospheric temperature - K", "environment", "out", "")
        f[59].init("mfreeze_environ", "int", 0, "Saving 'mfreeze' value", "environment", "save", "")
        f[60].init("pdynmcf", 0.0, "Saved dyn.press. when mfreeze=1 - Pa", "environment", "save", "")
        f[61].init("vmachf", 0.0, "Saved Mach# when mfreeze=1 - Pa", "environment", "save", "")
        f[64].init("dvae", 0.0, "Magnitude of constant air speed - m/s", "environment", "data", "")
        f[65].init("dvael", 0.0, "Air speed at low altitude - m/s", "environment", "data", "")
        f[66].init("waltl", 0.0, "Low altitude of air speed - m", "environment", "data", "")
        f[67].init("dvaeh", 0.0, "Air speed at high altitude - m/s", "environment", "data", "")
        f[68].init("walth", 0.0, "High altitude of air speed - m", "environment", "data", "")
        f[69].init("vaed3", 0.0, "Vertical air speed (pos.down) - m/s", "environment", "data", "")
        f[70].init("psiwdx", 0.0, "Wind direction from north - m/s", "environment", "data", "")
        f[71].init("twind", 0.1, "Wind smoothing time constant - sec", "environment", "data", "")
        f[72].init("VAELS", 0, 0, 0, "Smoothed wind vel in local level coord - m/s", "environment", "state", "")
        f[73].init("VAELSD", 0, 0, 0, "Smoothed wind velocity derivative - m/s", "environment", "state", "")
        f[74].init("VAEL", 0, 0, 0, "Wind vel in local level coordinates - m/s", "environment", "out", "")
        f[75].init("dvba", 0.0, "Vehicle speed wrt air - m/s", "environment", "out", "plot")
        f[76].init("VBAL", 0, 0, 0, "Vehicle vel wrt air in local level coord - m/s", "environment", "out", "")

    def environment(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        mwind = f[50].integer()
        dvae = f[64].real()
        dvael = f[65].real()
        waltl = f[66].real()
        dvaeh = f[67].real()
        walth = f[68].real()
        vaed3 = f[69].real()
        psiwdx = f[70].real()
        twind = f[71].real()
        mfreeze_environ = f[59].integer()
        pdynmcf = f[60].real()
        vmachf = f[61].real()
        VAELS = f[72].vec()
        VAELSD = f[73].vec()
        VBEL = f[233].vec()
        hbe = f[239].real()
        trcode = p[180].real()
        trmach = p[183].real()
        trdynm = p[184].real()
        mguid = p[400].integer()
        mfreeze = p[503].integer()

        rad = REARTH + hbe
        grav = G * EARTH_MASS / pow(rad, 2)

        rho, press, tempk = atmosphere76(hbe)
        vsound = math.sqrt(1.4 * R * tempk)

        dvw = 0.0
        VAEL = Matrix(3, 1)
        if mwind > 0:
            if mwind == 1:
                dvw = dvae
            if mwind == 2:
                dvw = dvael + (dvaeh - dvael) * (hbe - waltl) / (walth - waltl)
                if hbe < waltl:
                    dvw = 0
                if hbe > walth:
                    dvw = 0
            VAEL_RAW = Matrix().build_vec3(-dvw * math.cos(psiwdx * RAD),
                                            -dvw * math.sin(psiwdx * RAD), vaed3)
            VAELSD_NEW = (VAEL_RAW - VAELS) * (1.0 / twind)
            VAELS = integrate(VAELSD_NEW, VAELSD, VAELS, int_step)
            VAELSD = VAELSD_NEW
            VAEL = VAELS

        VBAL = VBEL - VAEL
        dvba = VBAL.absolute()

        vmach = abs(dvba / vsound)
        pdynmc = 0.5 * rho * dvba * dvba

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

        f[59].gets(mfreeze_environ)
        f[60].gets(pdynmcf)
        f[61].gets(vmachf)
        f[72].gets_vec(VAELS)
        f[73].gets_vec(VAELSD)
        f[52].gets(press)
        f[53].gets(rho)
        f[55].gets(grav)
        f[56].gets(vmach)
        f[57].gets(pdynmc)
        f[58].gets(tempk)
        f[75].gets(dvba)
        f[54].gets(vsound)
        f[74].gets_vec(VAEL)
        f[76].gets_vec(VBAL)
        p[180].gets(trcode)

    # ------------------------------------------------------------------ #
    # kinematics.cpp -- flat6[100-149]
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
        f[120].init("TBL", 0, 0, 0, 0, 0, 0, 0, 0, 0, "T.M. of body wrt local level coord", "kinematics", "out", "")
        f[134].init("psibl", 0.0, "Yawing angle of vehicle - rad", "kinematics", "diag", "")
        f[135].init("thtbl", 0.0, "Pitching angle of vehicle - rad", "kinematics", "diag", "")
        f[136].init("phibl", 0.0, "Rolling angle of vehicle - rad", "kinematics", "diag", "")
        f[137].init("psiblx", 0.0, "Yawing angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[138].init("thtblx", 0.0, "Pitching angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[139].init("phiblx", 0.0, "Rolling angle of vehicle - deg", "kinematics", "in/di", "scrn,plot")
        f[140].init("alppx", 0.0, "Total angle of attack - deg", "kinematics", "out", "plot")
        f[141].init("phipx", 0.0, "Aerodynamic roll angle - deg", "kinematics", "out", "plot")
        f[142].init("alpp", 0.0, "Total angle of attack - rad", "kinematics", "out", "")
        f[143].init("phip", 0.0, "Aerodynamic roll angle - rad", "kinematics", "out", "")
        f[144].init("alphax", 0.0, "Angle of attack - deg", "kinematics", "diag", "scrn,plot")
        f[145].init("betax", 0.0, "Side slip angle - deg", "kinematics", "diag", "scrn,plot")
        f[146].init("erq", 0.0, "Error of quaternion orthogonality", "kinematics", "diag", "plot")
        f[147].init("etbl", 0.0, "Error of D.C.M. nonorthogonality", "kinematics", "diag", "plot")
        f[148].init("TLB", 0, 0, 0, 0, 0, 0, 0, 0, 0, "T.M. of local level wrt body coord", "kinematics", "diag", "")

    def init_kinematics(self) -> None:
        f = self.flat6
        psiblx = f[137].real()
        thtblx = f[138].real()
        phiblx = f[139].real()

        spsi = math.sin(psiblx / (2.0 * DEG)); cpsi = math.cos(psiblx / (2.0 * DEG))
        stht = math.sin(thtblx / (2.0 * DEG)); ctht = math.cos(thtblx / (2.0 * DEG))
        sphi = math.sin(phiblx / (2.0 * DEG)); cphi = math.cos(phiblx / (2.0 * DEG))

        q0 = cpsi * ctht * cphi + spsi * stht * sphi
        q1 = cpsi * ctht * sphi - spsi * stht * cphi
        q2 = cpsi * stht * cphi + spsi * ctht * sphi
        q3 = -cpsi * stht * sphi + spsi * ctht * cphi

        TBL = mat3tr(psiblx / DEG, thtblx / DEG, phiblx / DEG)

        f[111].gets(q0)
        f[113].gets(q1)
        f[115].gets(q2)
        f[117].gets(q3)
        f[120].gets_mat(TBL)

    def kinematics(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        ck = f[101].real()
        time = f[0].real()
        dvba = f[75].real()
        VBAL = f[76].vec()
        WBEB = f[164].vec()
        trcode = p[180].real()
        tralppx = p[186].real()
        tralpnx = p[187].real()
        trbetx = p[188].real()

        q0d = f[110].real(); q0 = f[111].real()
        q1d = f[112].real(); q1 = f[113].real()
        q2d = f[114].real(); q2 = f[115].real()
        q3d = f[116].real(); q3 = f[117].real()

        quat_metric = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3
        erq = 1.0 - quat_metric

        pp = WBEB[0]; qq = WBEB[1]; rr = WBEB[2]

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
        TBL.assign_loc(0, 0, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
        TBL.assign_loc(0, 1, 2.0 * (q1 * q2 + q0 * q3))
        TBL.assign_loc(0, 2, 2.0 * (q1 * q3 - q0 * q2))
        TBL.assign_loc(1, 0, 2.0 * (q1 * q2 - q0 * q3))
        TBL.assign_loc(1, 1, q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)
        TBL.assign_loc(1, 2, 2.0 * (q2 * q3 + q0 * q1))
        TBL.assign_loc(2, 0, 2.0 * (q1 * q3 + q0 * q2))
        TBL.assign_loc(2, 1, 2.0 * (q2 * q3 - q0 * q1))
        TBL.assign_loc(2, 2, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)

        TLB = TBL.trans()
        UBL = TLB * TBL
        e1 = UBL.get_loc(0, 0) - 1.0
        e2 = UBL.get_loc(1, 1) - 1.0
        e3 = UBL.get_loc(2, 2) - 1.0
        etbl = math.sqrt(e1 * e1 + e2 * e2 + e3 * e3)

        tbl13 = TBL.get_loc(0, 2)
        tbl11 = TBL.get_loc(0, 0)
        tbl33 = TBL.get_loc(2, 2)
        tbl12 = TBL.get_loc(0, 1)
        tbl23 = TBL.get_loc(1, 2)

        if abs(tbl13) < 1:
            thtbl = math.asin(-tbl13)
            cthtbl = math.cos(thtbl)
        else:
            thtbl = PI / 2 * sign(-tbl13)
            cthtbl = EPS

        cpsi = tbl11 / cthtbl
        if abs(cpsi) >= 1:
            cpsi = (1 - EPS) * sign(cpsi)
        cphi = tbl33 / cthtbl
        if abs(cphi) >= 1:
            cphi = (1 - EPS) * sign(cphi)

        psibl = math.acos(cpsi) * sign(tbl12)
        phibl = math.acos(cphi) * sign(tbl23)

        psiblx = DEG * psibl
        thtblx = DEG * thtbl
        phiblx = DEG * phibl

        VBAB = TBL * VBAL
        vbab1 = VBAB[0]; vbab2 = VBAB[1]; vbab3 = VBAB[2]

        alpha = math.atan2(vbab3, vbab1)
        beta = math.asin(vbab2 / dvba)

        dum = vbab1 / dvba
        if abs(dum) > 1:
            dum = 1 * sign(dum)
        alpp = math.acos(dum)
        if vbab2 == 0 and vbab3 == 0:
            phip = 0.0
        elif abs(vbab2) < EPS:
            phip = 0.0 if vbab3 > 0 else PI
        else:
            phip = math.atan2(vbab2, vbab3)

        alphax = alpha * DEG
        betax = beta * DEG
        alppx = alpp * DEG
        phipx = phip * DEG

        if alphax > tralppx:
            trcode = 5
        if alphax < tralpnx:
            trcode = 6
        if abs(betax) > trbetx:
            trcode = 7

        f[110].gets(q0d); f[111].gets(q0)
        f[112].gets(q1d); f[113].gets(q1)
        f[114].gets(q2d); f[115].gets(q2)
        f[116].gets(q3d); f[117].gets(q3)
        f[120].gets_mat(TBL)
        f[144].gets(alphax)
        f[145].gets(betax)
        f[134].gets(psibl)
        f[135].gets(thtbl)
        f[136].gets(phibl)
        f[137].gets(psiblx)
        f[138].gets(thtblx)
        f[139].gets(phiblx)
        f[140].gets(alppx)
        f[141].gets(phipx)
        f[146].gets(erq)
        f[147].gets(etbl)
        f[148].gets_mat(TLB)
        p[180].gets(trcode)

    # ------------------------------------------------------------------ #
    # newton.cpp -- flat6[0-1,210-249]
    # ------------------------------------------------------------------ #
    def def_newton(self) -> None:
        f = self.flat6
        f[0].init("time", 0.0, "Simulation time - sec", "newton", "exec", "scrn,plot,com")
        f[1].init("halt", "int", 0, "=1: vehicle object is halted - ND", "newton", "exec", "")
        f[210].init("VBEBD", 0, 0, 0, " Derivative of plane velocity - m/s^2", "newton", "state", "")
        f[213].init("VBEB", 0, 0, 0, "Plane velocity in body axes - m/s", "newton", "state", "plot")
        f[216].init("SBELD", 0, 0, 0, "Derivative of plane position - m/s", "newton", "state", "")
        f[219].init("SBEL", 0, 0, 0, "Plane pos. wrt point E in local level axes - m", "newton", "state", "plot,com")
        f[220].init("sbel1", 0.0, "Initial north comp of SBEL - m", "newton", "data", "")
        f[221].init("sbel2", 0.0, "Initial east comp of SBEL - m", "newton", "data", "")
        f[222].init("sbel3", 0.0, "Initial down comp of SBEL - m", "newton", "data", "")
        f[223].init("SBELM", 0, 0, 0, "Saved aircraft position SBEL - m", "newton", "save", "")
        f[224].init("groundrange", 0.0, "ground range - m", "newton", "diag", "")
        f[230].init("FSPB", 0, 0, 0, "Specific force in body axes - m/s^2", "newton", "out", "")
        f[233].init("VBEL", 0, 0, 0, "Plane velocity in local level axes - m/s", "newton", "out", "com,scrn,plot")
        f[236].init("dvbe", 0.0, "Plane speed - m/s", "newton", "in/out", "plot")
        f[237].init("alpha0x", 0.0, "Initial angle-of-attack - deg", "newton", "data", "")
        f[238].init("beta0x", 0.0, "Initial side slip angle - deg", "newton", "data", "")
        f[239].init("hbe", 0.0, "Height above ground - m", "newton", "out", "scrn,plot")
        f[240].init("psivlx", 0.0, "Heading angle - deg", "newton", "diag", "scrn,plot")
        f[241].init("thtvlx", 0.0, "Vertical flight path angle - deg", "newton", "diag", "scrn,plot")
        f[243].init("alx", 0.0, "Lateral (horizontal) specific force component - g's", "newton", "diag", "plot")
        f[244].init("anx", 0.0, "Normal specific force component - g's", "newton", "diag", "scrn,plot")
        f[245].init("ayx", 0.0, "Side specific force component - g's", "newton", "diag", "plot")
        f[246].init("ATB", 0, 0, 0, "Tangential accel, solving Newton's Law - m/s^2", "newton", "diag", "")
        f[247].init("mfreeze_newt", "int", 0, "Saving mfreze value - ND", "newton", "save", "")
        f[248].init("dvbef", 0.0, "Saved speed under mfreeze=1 - m/s", "newton", "save", "")

    def init_newton(self) -> None:
        f = self.flat6
        sbel1 = f[220].real(); sbel2 = f[221].real(); sbel3 = f[222].real()
        dvbe = f[236].real()
        alpha0x = f[237].real(); beta0x = f[238].real()
        TBL = f[120].mat()

        salp = math.sin(alpha0x * RAD); calp = math.cos(alpha0x * RAD)
        sbet = math.sin(beta0x * RAD); cbet = math.cos(beta0x * RAD)
        VBEB = Matrix().build_vec3(calp * cbet * dvbe, sbet * dvbe, salp * cbet * dvbe)

        VBEL = (~TBL) * VBEB

        vbel1 = VBEL.get_loc(0, 0); vbel2 = VBEL.get_loc(1, 0); vbel3 = VBEL.get_loc(2, 0)
        if vbel1 == 0.0 and vbel2 == 0.0:
            psivl = 0.0
        else:
            psivl = math.atan2(vbel2, vbel1)
        thtvl = math.atan2(-vbel3, math.sqrt(vbel1 * vbel1 + vbel2 * vbel2))
        psivlx = psivl * DEG
        thtvlx = thtvl * DEG

        SBEL = Matrix().build_vec3(sbel1, sbel2, sbel3)
        hbe = -SBEL.get_loc(2, 0)
        SBELM = SBEL

        f[213].gets_vec(VBEB)
        f[219].gets_vec(SBEL)
        f[223].gets_vec(SBELM)
        f[233].gets_vec(VBEL)
        f[239].gets(hbe)
        f[240].gets(psivlx)
        f[241].gets(thtvlx)

    def newton(self, sim_time: float, int_step: float) -> None:
        f, p = self.flat6, self.plane

        dvbe = f[236].real()
        mfreeze_newt = f[247].integer()
        dvbef = f[248].real()
        SBELM = f[223].vec()
        groundrange = f[224].real()
        grav = f[55].real()
        TBL = f[120].mat()
        WBEB = f[164].vec()
        FAPB = f[200].vec()
        vmass = p[190].real()
        mfreeze = p[503].integer()
        VBEBD = f[210].vec()
        VBEB = f[213].vec()
        SBELD = f[216].vec()
        SBEL = f[219].vec()

        time = sim_time

        ATB = WBEB.skew_sym() * VBEB
        GRAVL = Matrix().build_vec3(0, 0, grav)

        FSPB = FAPB * (1.0 / vmass)
        VBEBD_NEW = FSPB - ATB + TBL * GRAVL
        VBEB = integrate(VBEBD_NEW, VBEBD, VBEB, int_step)
        VBEBD = VBEBD_NEW

        VBEL = (~TBL) * VBEB
        SBELD_NEW = VBEL
        SBEL = integrate(SBELD_NEW, SBELD, SBEL, int_step)
        SBELD = SBELD_NEW

        vbel1 = VBEL.get_loc(0, 0); vbel2 = VBEL.get_loc(1, 0); vbel3 = VBEL.get_loc(2, 0)
        if vbel1 == 0.0 and vbel2 == 0.0:
            psivl = 0.0
        else:
            psivl = math.atan2(vbel2, vbel1)
        thtvl = math.atan2(-vbel3, math.sqrt(vbel1 * vbel1 + vbel2 * vbel2))
        psivlx = psivl * DEG
        thtvlx = thtvl * DEG

        dvbe = VBEL.absolute()
        hbe = -SBEL.get_loc(2, 0)

        anx = -FSPB[2] / grav
        ayx = FSPB[1] / grav

        TVL = mat2tr(psivl, thtvl)
        TVB = TVL * (~TBL)
        FSPV = TVB * FSPB
        alx = FSPV[1] / grav

        if mfreeze == 0:
            mfreeze_newt = 0
        else:
            if mfreeze != mfreeze_newt:
                mfreeze_newt = mfreeze
                dvbef = dvbe
            dvbe = dvbef

        DELSBEL = SBEL - SBELM
        DELSBEL[2] = 0
        delgrd = DELSBEL.absolute()
        groundrange += delgrd
        SBELM = SBEL

        f[210].gets_vec(VBEBD)
        f[213].gets_vec(VBEB)
        f[216].gets_vec(SBELD)
        f[219].gets_vec(SBEL)
        f[223].gets_vec(SBELM)
        f[224].gets(groundrange)
        f[247].gets(mfreeze_newt)
        f[248].gets(dvbef)
        f[0].gets(time)
        f[230].gets_vec(FSPB)
        f[233].gets_vec(VBEL)
        f[236].gets(dvbe)
        f[239].gets(hbe)
        f[240].gets(psivlx)
        f[241].gets(thtvlx)
        f[243].gets(alx)
        f[244].gets(anx)
        f[245].gets(ayx)
        f[246].gets_vec(ATB)

    # ------------------------------------------------------------------ #
    # euler.cpp -- flat6[150-199]
    # ------------------------------------------------------------------ #
    def def_euler(self) -> None:
        f = self.flat6
        f[160].init("ppx", 0.0, "Body roll angular vel wrt Earth in body axes - deg/s", "euler", "init/out", "plot")
        f[161].init("qqx", 0.0, "Body pitch angular vel wrt Earth in body axes - deg/s", "euler", "init/out", "plot")
        f[162].init("rrx", 0.0, "Body yaw angular vel wrt Earth in body axes - deg/s", "euler", "init/out", "plot")
        f[164].init("WBEB", 0, 0, 0, "Ang vel of veh wrt inertial frame, body axes - rad/s", "euler", "state", "")
        f[165].init("WBEBD", 0, 0, 0, "Ang vel of veh wrt inertl frame, deriv - rad/s^2", "euler", "state", "")

    def init_euler(self) -> None:
        f = self.flat6
        ppx = f[160].real(); qqx = f[161].real(); rrx = f[162].real()
        WBEB = Matrix().build_vec3(ppx * RAD, qqx * RAD, rrx * RAD)
        f[164].gets_vec(WBEB)

    def euler(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        FMB = f[201].vec()
        IBBB = p[191].mat()
        eng_ang_mom = p[192].real()
        WBEB = f[164].vec()
        WBEBD = f[165].vec()

        L_ENGINE = Matrix().build_vec3(eng_ang_mom, 0, 0)

        WACC_NEXT = IBBB.inverse() * (FMB - WBEB.skew_sym() * (IBBB * WBEB + L_ENGINE))
        WBEB = integrate(WACC_NEXT, WBEBD, WBEB, int_step)
        WBEBD = WACC_NEXT

        ppx = WBEB.get_loc(0, 0) * DEG
        qqx = WBEB.get_loc(1, 0) * DEG
        rrx = WBEB.get_loc(2, 0) * DEG

        f[164].gets_vec(WBEB)
        f[165].gets_vec(WBEBD)
        f[160].gets(ppx)
        f[161].gets(qqx)
        f[162].gets(rrx)


class Plane(Flat6):
    """Derived class 'Plane' (class_hierarchy.hpp / aerodynamics.cpp /
    propulsion.cpp / forces.cpp / actuator.cpp / control.cpp /
    guidance.cpp of CADAC/FALCON6) -- the F16-like fixed-wing aircraft:
    aero tables, single turbojet, rate/acceleration/altitude/heading
    autopilot, line/waypoint guidance."""

    def __init__(self):
        super().__init__()
        self.plane: List[Variable] = _new_variable_array(NPLANE)
        self.aerotable = None   # Datadeck, set by AERO_DECK in input.asc
        self.proptable = None   # Datadeck, set by PROP_DECK in input.asc

    # ------------------------------------------------------------------ #
    # aerodynamics.cpp -- plane[100-199]
    # ------------------------------------------------------------------ #
    def def_aerodynamics(self) -> None:
        p = self.plane
        p[104].init("refa", 0.0, "Reference area for aero coefficients - m^2", "aerodynamics", "init", "")
        p[105].init("refb", 0.0, "Reference span for aero coefficients - m", "aerodynamics", "init", "")
        p[106].init("refc", 0.0, "Reference cord for aero coefficients - m", "aerodynamics", "init", "")
        p[109].init("cd", 0.0, "Drag coefficient - ND", "aerodynamics", "diag", "")
        p[110].init("cl", 0.0, "Lift coefficient - ND", "aerodynamics", "diag", "")
        p[111].init("cxt", 0.0, "X-force coefficient - ND", "aerodynamics", "out", "")
        p[112].init("cyt", 0.0, "Side force coefficient - ND", "aerodynamics", "out", "")
        p[113].init("czt", 0.0, "Z-force coefficient - ND", "aerodynamics", "out", "")
        p[114].init("clt", 0.0, "Rolling moment coefficient - ND", "aerodynamics", "out", "")
        p[115].init("cmt", 0.0, "Pitching moment coefficient - ND", "aerodynamics", "out", "")
        p[116].init("cnt", 0.0, "Yawing moment coefficient - ND", "aerodynamics", "out", "")
        p[120].init("cd0", 0.0, "Reference drag coeff(alpha,mach) - ND", "aerodynamics", "diag", "")
        p[121].init("cda", 0.0, "Delta drag force due to alpha(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[122].init("cl0", 0.0, "Reference lift coeff(alpha,mach) - ND", "aerodynamics", "diag", "")
        p[123].init("cla", 0.0, "Lift slope derivative(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[124].init("clde", 0.0, "Lift force due to elevator (alpha.mach), - 1/deg", "aerodynamics", "diag", "")
        p[125].init("cyb", 0.0, "Weather vane der wrt beta(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[126].init("cyda", 0.0, "Side force due to aileron deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[127].init("cydr", 0.0, "Side force due to rudder deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[128].init("cllb", 0.0, "Dutch-roll deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[129].init("cllda", 0.0, "Roll control effectiveness(alpha,mach), - 1/deg", "aerodynamics", "diag", "")
        p[130].init("cllp", 0.0, "Roll damping deriv(alpha,mach) - 1/rad", "aerodynamics", "diag", "")
        p[131].init("cllr", 0.0, "Roll moment due to yaw rate deriv(alpha,mach) - 1/rad", "aerodynamics", "diag", "")
        p[132].init("cm0", 0.0, "Pitch moment coeff(alpha,mach) - ND", "aerodynamics", "diag", "")
        p[133].init("cma", 0.0, "Pitch moment due to alpha deriv(alpha,mach) -1/deg", "aerodynamics", "diag", "")
        p[134].init("cmde", 0.0, "Pitch control effectiveness(alpha,mach), - 1/deg", "aerodynamics", "diag", "")
        p[135].init("cmq", 0.0, "Pitch dampning deriv(alpha,mach) - 1/rad", "aerodynamics", "diag", "")
        p[136].init("clnb", 0.0, "Yaw moment deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[137].init("clnda", 0.0, "Yaw moment due to aileron deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[138].init("clndr", 0.0, "Yaw moment due to rudder deriv(alpha,mach) - 1/deg", "aerodynamics", "diag", "")
        p[139].init("clnp", 0.0, "Yaw moment due to roll rate deriv(alpha,mach) - 1/rad", "aerodynamics", "diag", "")
        p[140].init("clnr", 0.0, "Yaw damping deriv(alpha,mach) - 1/rad", "aerodynamics", "diag", "")
        p[141].init("clldr", 0.0, "Roll moment due to rudder - 1/deg", "aerodynamics", "diag", "")
        p[142].init("clovercd", 0.0, "Lift over drag ratio - ND", "aerodynamics", "diag", "")
        p[144].init("stmarg", 0.0, "Static margin (+stable, -unstable) - caliber", "aerodynamics", "diag", "plot")
        p[145].init("dla", 0.0, "Lift slope derivative - m/s^2", "aerodynamics", "out", "")
        p[146].init("dlde", 0.0, "Lift elevator control derivative - m/s^2", "aerodynamics", "out", "")
        p[147].init("dma", 0.0, "Pitch moment derivative - 1/s^2", "aerodynamics", "out", "plot")
        p[148].init("dmq", 0.0, "Pitch damping derivative - 1/s", "aerodynamics", "out", "")
        p[149].init("dmde", 0.0, "Pitch control derivative - 1/s^2", "aerodynamics", "out", "plot")
        p[150].init("dyb", 0.0, "Side force derivative - m/s^2", "aerodynamics", "out", "")
        p[151].init("dydr", 0.0, "Side force control derivative - m/s^2", "aerodynamics", "out", "")
        p[152].init("dnb", 0.0, "Yawing moment derivative - 1/s^2", "aerodynamics", "out", "")
        p[153].init("dnr", 0.0, "Yaw dampnig derivative - 1/s", "aerodynamics", "out", "")
        p[154].init("dndr", 0.0, "Yaw control derivative - 1/s^2", "aerodynamics", "out", "")
        p[155].init("dllp", 0.0, "Roll damping derivative - 1/s", "aerodynamics", "out", "")
        p[156].init("dllda", 0.0, "Roll control derivative - 1/s^2", "aerodynamics", "out", "")
        p[157].init("cdrag", 0.0, "Drag coefficient - ND", "aerodynamics", "out", "")
        p[158].init("clift", 0.0, "Drag coefficient - ND", "aerodynamics", "diag", "")
        p[160].init("alplimpx", 0.0, "Maximum positive alpha permissible - deg", "aerodynamics", "data", "")
        p[163].init("gmax", 0.0, "Max maneuverability limited by strct_pos_limitx- g's", "aerodynamics", "out", "plot")
        p[164].init("alplimnx", 0.0, "Minimum neg alpha permissible (with neg sign) - deg", "aerodynamics", "data", "")
        p[167].init("gminx", 0.0, "Min maneuverability limited by strct_neg_limitx - g's", "aerodynamics", "out", "plot")
        p[170].init("realp1", 0.0, "First real root of airframe pitch dyn  - rad/s", "aerodynamics", "diag", "plot")
        p[171].init("realp2", 0.0, "Second real root of airframe pitch dyn - rad/s", "aerodynamics", "diag", "plot")
        p[172].init("wnp", 0.0, "Natural frequency of airframe pitch dynamics - rad/s", "aerodynamics", "diag", "plot")
        p[173].init("zetp", 0.0, "Damping of airframe pitch dynamics - NA", "aerodynamics", "diag", "plot")
        p[174].init("rpreal", 0.0, "Real part or mean value (real roots) of pitch  - rad/s", "aerodynamics", "diag", "")
        p[175].init("realy1", 0.0, "First real root of airframe yaw dynamics - rad/s", "aerodynamics", "diag", "plot")
        p[176].init("realy2", 0.0, "Second real root of airframe yaw dynamics - rad/s", "aerodynamics", "diag", "plot")
        p[177].init("wny", 0.0, "Natural frequency of airframe yaw dynamics - rad/s", "aerodynamics", "diag", "plot")
        p[178].init("zety", 0.0, "Damping of airframe yaw dynamics - NA", "aerodynamics", "diag", "plot")
        p[179].init("ryreal", 0.0, "Real part or mean value (real roots) of yaw - rad/s", "aerodynamics", "diag", "")
        p[180].init("trcode", 0.0, "Termination code number", "aerodynamics", "init", "")
        p[181].init("tmcode", 0.0, "Dummy variable initialized to zero", "aerodynamics", "data", "")
        p[183].init("trmach", 0.0, "Minimum Mach number", "aerodynamics", "data", "")
        p[184].init("trdynm", 0.0, "Minimum dynamic pressure - Pa", "aerodynamics", "data", "")
        p[185].init("trload", 0.0, "Minimum load capacity - g's", "aerodynamics", "data", "")
        p[186].init("tralppx", 0.0, "Maximum positive angle of attack - deg", "aerodynamics", "data", "")
        p[187].init("tralpnx", 0.0, "Minimum negative angle of attack - deg", "aerodynamics", "data", "")
        p[188].init("trbetx", 0.0, "Maximum absolute value of sideslip angle - deg", "aerodynamics", "data", "")
        p[190].init("vmass", 0.0, "Aircraft mass - kg", "aerodynamics", "init", "")
        p[191].init("IBBB", 0, 0, 0, 0, 0, 0, 0, 0, 0, "Aircraft moment of inertia - kg*m^2", "aerodynamics", "init", "")
        p[192].init("eng_ang_mom", 0.0, "Engine angular momentum - kg*m^2/sec", "aerodynamics", "init", "")
        p[193].init("xcg", 0.0, "Actual c.g location - m", "aerodynamics", "data", "")
        p[194].init("xcgr", 0.0, "Reference c.g location - m", "aerodynamics", "data", "")

    def init_aerodynamics(self) -> None:
        p = self.plane
        refa, refb, refc = 27.87, 9.14, 3.45

        vmass = 9496.0
        IBBB = Matrix(3, 3)
        IBBB.assign_loc(0, 0, 12875)
        IBBB.assign_loc(2, 0, -1331.4)
        IBBB.assign_loc(1, 1, 75673)
        IBBB.assign_loc(0, 2, -1331.4)
        IBBB.assign_loc(2, 2, 85551)

        eng_ang_mom = 70000.0

        trmach, trdynm, trload = 0.8, 10.0e3, 3.0
        tralppx, tralpnx, trbetx = 21.0, -6.0, 5.0
        trcode, tmcode = 0.0, 0.0

        p[104].gets(refa); p[105].gets(refb); p[106].gets(refc)
        p[180].gets(trcode); p[181].gets(tmcode)
        p[183].gets(trmach); p[184].gets(trdynm); p[185].gets(trload)
        p[186].gets(tralppx); p[187].gets(tralpnx); p[188].gets(trbetx)
        p[190].gets(vmass)
        p[191].gets_mat(IBBB)
        p[192].gets(eng_ang_mom)

    def aerodynamics(self) -> None:
        f, p, tbl = self.flat6, self.plane, self.aerotable

        alplimpx = p[160].real(); alplimnx = p[164].real()
        xcg = p[193].real(); xcgr = p[194].real()
        refa = p[104].real(); refb = p[105].real(); refc = p[106].real()
        trcode = p[180].real()
        trload = p[185].real()
        vmass = p[190].real()

        alphax = f[144].real(); betax = f[145].real()
        vmach = f[56].real(); pdynmc = f[57].real(); dvba = f[75].real()
        ppx = f[160].real(); qqx = f[161].real(); rrx = f[162].real()
        delax = p[619].real(); delex = p[620].real(); delrx = p[621].real()

        c2v = refc / (2 * dvba)
        b2v = refb / (2 * dvba)

        cx = tbl.look_up("cx_vs_elev_alpha", delex, alphax)
        cxq = tbl.look_up("cxq_vs_alpha", alphax)
        cxt = cx + c2v * cxq * qqx * RAD

        cyr = tbl.look_up("cyr_vs_alpha", alphax)
        cyp = tbl.look_up("cyp_vs_alpha", alphax)
        cyt = -0.02 * betax + 0.021 * delax / 20 + 0.086 * delrx / 30 + b2v * (cyr * rrx * RAD + cyp * ppx * RAD)

        cz = tbl.look_up("cz_vs_alpha", alphax)
        czq = tbl.look_up("czq_vs_alpha", alphax)
        czt = cz * (1 - pow(betax * RAD, 2)) - 0.19 * delex / 25 + c2v * czq * qqx * RAD

        cl = tbl.look_up("cl_vs_beta_alpha", betax, alphax)
        cldr = tbl.look_up("cldr_vs_beta_alpha", betax, alphax)
        clda = tbl.look_up("clda_vs_beta_alpha", betax, alphax)
        clda = -clda
        clr = tbl.look_up("clr_vs_alpha", alphax)
        clp = tbl.look_up("clp_vs_alpha", alphax)
        cllr = p[131].real()
        clt = cl + clda * delax / 20 + cldr * delrx / 30 + b2v * (cllr * rrx * RAD + clp * ppx * RAD)

        cm = tbl.look_up("cm_vs_elev_alpha", delex, alphax)
        cmq = tbl.look_up("cmq_vs_alpha", alphax)
        czt_for_cmt = czt
        cmt = cm + c2v * cmq * qqx * RAD + czt_for_cmt * (xcgr - xcg) / refc

        cn = tbl.look_up("cn_vs_beta_alpha", betax, alphax)
        cnda = tbl.look_up("cnda_vs_beta_alpha", betax, alphax)
        cndr = tbl.look_up("cndr_vs_beta_alpha", betax, alphax)
        cnr = tbl.look_up("cnr_vs_alpha", alphax)
        cnp = tbl.look_up("cnp_vs_alpha", alphax)
        cnt = (cn + cnda * delax / 20 + cndr * delrx / 30 - cyt * (xcgr - xcg) / refb
               + b2v * (cnr * rrx * RAD + cnp * ppx * RAD))

        czp = tbl.look_up("cz_vs_alpha", alplimpx)
        czn = tbl.look_up("cz_vs_alpha", alplimnx)
        alpx = -czp * pdynmc * refa
        alnx = -czn * pdynmc * refa
        weight = vmass * AGRAV
        gmax = alpx / weight
        gminx = alnx / weight

        cosa = math.cos(alphax * RAD); sina = math.sin(alphax * RAD)
        cdrag = -cxt * cosa - czt * sina
        clift = cxt * sina - czt * cosa
        clovercd = clift / cdrag

        clde = 0.19 / 25
        cyb = -0.02 * RAD
        cydr = -0.086 / 30
        clnr = b2v * cnr
        clndr = cndr / 30
        cllp = b2v * clp
        cllda = clda / 20

        if gmax < trload:
            trcode = 4

        p[111].gets(cxt); p[112].gets(cyt); p[113].gets(czt)
        p[114].gets(clt); p[115].gets(cmt); p[116].gets(cnt)
        p[157].gets(cdrag); p[163].gets(gmax); p[167].gets(gminx)
        p[109].gets(0.0); p[110].gets(0.0)
        p[120].gets(0.0); p[121].gets(0.0); p[122].gets(0.0)
        p[123].gets(0.0); p[124].gets(clde)
        p[125].gets(cyb); p[126].gets(0.0); p[127].gets(cydr)
        p[128].gets(0.0); p[129].gets(cllda); p[130].gets(cllp); p[131].gets(cllr)
        p[132].gets(0.0); p[134].gets(0.0); p[135].gets(cmq)
        p[137].gets(0.0); p[138].gets(clndr); p[139].gets(0.0); p[140].gets(clnr)
        p[141].gets(0.0); p[142].gets(clovercd)
        p[158].gets(clift)
        p[180].gets(trcode)

        self.aerodynamics_der()

    def aerodynamics_der(self) -> None:
        f, p, tbl = self.flat6, self.plane, self.aerotable

        refa = p[104].real(); refb = p[105].real(); refc = p[106].real()
        xcg = p[193].real(); xcgr = p[194].real()
        vmach = f[56].real(); pdynmc = f[57].real(); dvba = f[75].real()
        alphax = f[144].real(); betax = f[145].real()
        vmass = p[190].real()
        IBBB = p[191].mat()
        delex = p[620].real()

        clde = p[124].real()
        cyb = p[125].real()
        cydr = p[127].real()
        cllda = p[129].real()
        cllp = p[130].real()
        cmq = p[135].real()
        clndr = p[138].real()
        clnr = p[140].real()

        czp = tbl.look_up("cz_vs_alpha", alphax + 1.5)
        czn = tbl.look_up("cz_vs_alpha", alphax - 1.5)
        cza = (czp - czn) / 3
        cla = -cza

        cmp_ = tbl.look_up("cm_vs_elev_alpha", delex, alphax + 1.5)
        cmn = tbl.look_up("cm_vs_elev_alpha", delex, alphax - 1.5)
        dum = (cmp_ - cmn) / 3
        cma = dum + cza * (xcgr - xcg) / refc

        cmp_ = tbl.look_up("cm_vs_elev_alpha", delex + 1.5, alphax)
        cmn = tbl.look_up("cm_vs_elev_alpha", delex - 1.5, alphax)
        cmde = (cmp_ - cmn) / 3

        cnp = tbl.look_up("cn_vs_beta_alpha", betax + 1.5, alphax)
        cnn = tbl.look_up("cn_vs_beta_alpha", betax - 1.5, alphax)
        dum = (cnp - cnn) / 3
        clnb = dum - cyb * (xcgr - xcg) / refb

        ibbb11 = IBBB.get_loc(0, 0); ibbb22 = IBBB.get_loc(1, 1); ibbb33 = IBBB.get_loc(2, 2)

        duml = (pdynmc * refa / vmass) / RAD
        dla = duml * cla
        dlde = duml * clde
        dumm = pdynmc * refa * refc / ibbb22
        dma = dumm * cma / RAD
        dmq = dumm * (refc / (2.0 * dvba)) * cmq
        dmde = dumm * cmde / RAD

        dumy = pdynmc * refa / vmass
        dyb = dumy * cyb / RAD
        dydr = dumy * cydr / RAD
        dumn = pdynmc * refa * refb / ibbb33
        dnb = dumn * clnb / RAD
        dnr = dumn * (refb / (2.0 * dvba)) * clnr
        dndr = dumn * clndr / RAD

        dumll = pdynmc * refa * refb / ibbb11
        dllp = dumll * (refb / (2.0 * dvba)) * cllp
        dllda = dumll * cllda / RAD

        stmarg = -cma / cla if cla else 0.0

        # note: pitch-plane roots use 'dvba' (same as 'dvbe' elsewhere --
        # this scope only localizes 'dvba', matching the original's
        # 'aerodynamics_der()', which never localizes 'dvbe')
        a11 = dmq
        a12 = dma / dla
        a21 = dla
        a22 = -dla / dvba

        arg = pow((a11 + a22), 2) - 4.0 * (a11 * a22 - a12 * a21)
        if arg >= 0.0:
            wnp = 0.0; zetp = 0.0
            dum2 = a11 + a22
            realp1 = (dum2 + math.sqrt(arg)) / 2.0
            realp2 = (dum2 - math.sqrt(arg)) / 2.0
            rpreal = (realp1 + realp2) / 2.0
        else:
            realp1 = 0.0; realp2 = 0.0
            wnp = math.sqrt(a11 * a22 - a12 * a21)
            zetp = -(a11 + a22) / (2.0 * wnp)
            rpreal = -zetp * wnp

        a11 = dnr; a12 = dnb / dyb; a21 = -dyb; a22 = dyb / dvba
        arg = pow((a11 + a22), 2) - 4.0 * (a11 * a22 - a12 * a21)
        if arg >= 0.0:
            wny = 0.0; zety = 0.0
            dum2 = a11 + a22
            realy1 = (dum2 + math.sqrt(arg)) / 2.0
            realy2 = (dum2 - math.sqrt(arg)) / 2.0
            ryreal = (realy1 + realy2) / 2.0
        else:
            realy1 = 0.0; realy2 = 0.0
            wny = math.sqrt(a11 * a22 - a12 * a21)
            zety = -(a11 + a22) / (2.0 * wny)
            ryreal = -zety * wny

        p[145].gets(dla); p[146].gets(dlde); p[147].gets(dma)
        p[148].gets(dmq); p[149].gets(dmde)
        p[150].gets(dyb); p[151].gets(dydr)
        p[152].gets(dnb); p[153].gets(dnr); p[154].gets(dndr)
        p[155].gets(dllp); p[156].gets(dllda)
        p[133].gets(cma); p[136].gets(clnb); p[144].gets(stmarg)
        p[170].gets(realp1); p[171].gets(realp2); p[172].gets(wnp); p[173].gets(zetp); p[174].gets(rpreal)
        p[175].gets(realy1); p[176].gets(realy2); p[177].gets(wny); p[178].gets(zety); p[179].gets(ryreal)

    # ------------------------------------------------------------------ #
    # propulsion.cpp -- plane[50-99]
    # ------------------------------------------------------------------ #
    def def_propulsion(self) -> None:
        p = self.plane
        p[50].init("mprop", "int", 0, "=0: off,=1: manual throttle,=2: Mach hold", "propulsion", "data", "")
        p[51].init("vmachcom", 0.0, "Commanded Mach # - ND", "propulsion", "data", "")
        p[52].init("throttle", 0.0, "Throttle setting (0->1) ND", "propulsion", "data", "scrn,plot")
        p[53].init("gmach", 0.0, "Gain conversion from Mach to throttle - ND", "propulsion", "data", "")
        p[54].init("thrustf", 0.0, "Saved thrust when 'mfeeze'=1 - N", "propulsion", "save", "")
        p[55].init("thrust", 0.0, "Turbojet thrust - N", "propulsion", "out", "scrn,plot")
        p[56].init("thrust_req", 0.0, "Required thrust - N", "propulsion", "diag", "")
        p[57].init("mfreeze_prop", "int", 0, "Saving m'mfreeze' value - ND", "propulsion", "save", "")
        p[58].init("powerd", 0.0, "Derivative of achieved power setting - %/sec", "propulsion", "state", "")
        p[59].init("power", 0.0, "Achieved power setting - %", "propulsion", "state", "")
        p[60].init("power_com", 0.0, "Commanded power setting - %", "propulsion", "diag", "")
        p[61].init("tpower", 0.0, "Spool-up time constant - sec", "propulsion", "diag", "")
        p[62].init("idle", 0.0, "Idle thrust from tables - N", "propulsion", "diag", "plot")
        p[63].init("mil", 0.0, "Military thrust from tables - N", "propulsion", "diag", "plot")
        p[64].init("max", 0.0, "Maximum thrust from tables - N", "propulsion", "diag", "plot")

    def propulsion(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        mprop = p[50].integer()
        vmachcom = p[51].real()
        throttle = p[52].real()
        gmach = p[53].real()
        thrustf = p[54].real()
        mfreeze_prop = p[57].integer()
        vmach = f[56].real()
        pdynmc = f[57].real()
        alphax = f[144].real()
        refa = p[104].real()
        cdrag = p[157].real()
        mfreeze = p[503].integer()

        thrust_req = 0.0
        if mprop == 1:
            thrust = self.propulsion_thrust(throttle, int_step)
        elif mprop == 2:
            emach = vmachcom - vmach
            throttle = gmach * emach
            if throttle < 0:
                throttle = 0
            if throttle > 0.77:
                throttle = 0.77
            thrust = self.propulsion_thrust(throttle, int_step)
            thrust_req = cdrag * pdynmc * refa / math.cos(alphax * RAD)
        else:
            thrust = 0.0

        if mfreeze == 0:
            mfreeze_prop = 0
        else:
            if mfreeze != mfreeze_prop:
                mfreeze_prop = mfreeze
                thrustf = thrust
            thrust = thrustf

        p[54].gets(thrustf)
        p[57].gets(mfreeze_prop)
        p[55].gets(thrust)
        p[56].gets(thrust_req)
        p[52].gets(throttle)

    def propulsion_thrust(self, throttle: float, int_step: float) -> float:
        f, p, tbl = self.flat6, self.plane, self.proptable

        powerd = p[58].real()
        power = p[59].real()
        vmach = f[56].real()
        hbe = f[239].real()

        if throttle <= 0.77:
            power_com = 64.94 * throttle
        else:
            power_com = 217.38 * throttle - 117.38

        tpower = 1.0 if power_com <= 50 else 0.2

        powerd_new = (power_com - power) / tpower
        power = integrate(powerd_new, powerd, power, int_step)
        powerd = powerd_new

        hbe_ft = hbe * FOOT
        idle = tbl.look_up("idle_vs_mach_alt", vmach, hbe_ft) * NT
        mil = tbl.look_up("mil_vs_mach_alt", vmach, hbe_ft) * NT
        maxthr = tbl.look_up("max_vs_mach_alt", vmach, hbe_ft) * NT

        if power < 50:
            thrust = idle + power * 0.02 * (mil - idle)
        else:
            thrust = mil + (power - 50) * 0.02 * (maxthr - mil)

        p[58].gets(powerd)
        p[59].gets(power)
        p[60].gets(power_com)
        p[61].gets(tpower)
        p[62].gets(idle)
        p[63].gets(mil)
        p[64].gets(maxthr)

        return thrust

    # ------------------------------------------------------------------ #
    # forces.cpp -- flat6[200-201]
    # ------------------------------------------------------------------ #
    def def_forces(self) -> None:
        f = self.flat6
        f[200].init("FAPB", 0, 0, 0, "Aerodynamic and propulsive forces in body axes - N", "forces", "out", "")
        f[201].init("FMB", 0, 0, 0, "Aerodynamic and propulsive moments in body axes - N*m", "forces", "out", "")

    def forces(self) -> None:
        f, p = self.flat6, self.plane

        pdynmc = f[57].real()
        thrust = p[55].real()
        refa = p[104].real(); refb = p[105].real(); refc = p[106].real()
        cxt = p[111].real(); cyt = p[112].real(); czt = p[113].real()
        clt = p[114].real(); cmt = p[115].real(); cnt = p[116].real()

        FAPB = Matrix().build_vec3(pdynmc * refa * cxt + thrust, pdynmc * refa * cyt, pdynmc * refa * czt)
        FMB = Matrix().build_vec3(pdynmc * refa * refb * clt, pdynmc * refa * refc * cmt, pdynmc * refa * refb * cnt)

        f[200].gets_vec(FAPB)
        f[201].gets_vec(FMB)

    # ------------------------------------------------------------------ #
    # actuator.cpp -- plane[600-649]
    # ------------------------------------------------------------------ #
    def def_actuator(self) -> None:
        p = self.plane
        p[600].init("mact", "int", 0, "=0:no dynamics, =2:second order", "actuator", "data", "")
        p[602].init("dlimx", 0.0, "Control fin limiter - deg", "actuator", "data", "")
        p[604].init("ddlimx", 0.0, "Control fin rate limiter - deg/s", "actuator", "data", "")
        p[605].init("wnact", 0.0, "Natural frequency of actuator - rad/s", "actuator", "data", "")
        p[606].init("zetact", 0.0, "Damping of actuator - ND", "actuator", "data", "")
        p[619].init("delax", 0.0, "Aileron control deflection - deg", "actuator", "out", "scrn,plot")
        p[620].init("delex", 0.0, "Elevator control deflection - deg", "actuator", "out", "scrn,plot")
        p[621].init("delrx", 0.0, "Rudder control deflection - deg", "actuator", "out", "scrn,plot")
        p[630].init("DXD", 0, 0, 0, "Fin position derivative - deg/s", "actuator", "state", "")
        p[631].init("DX", 0, 0, 0, "Fin position - deg", "actuator", "state", "")
        p[632].init("DDXD", 0, 0, 0, "Fin rate derivative - deg/s^2", "actuator", "state", "")
        p[633].init("DDX", 0, 0, 0, "Fin rate - deg/s", "actuator", "state", "")

    def actuator(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        mact = p[600].integer()
        dlimx = p[602].real()
        delacx = p[519].real(); delecx = p[520].real(); delrcx = p[521].real()

        ACTCX = Matrix().build_vec3(delacx, delecx, delrcx)

        if mact == 0:
            ACTX = ACTCX
            for i in range(3):
                if abs(ACTX.get_loc(i, 0)) > dlimx:
                    ACTX.assign_loc(i, 0, dlimx * sign(ACTX.get_loc(i, 0)))
        elif mact == 2:
            ACTX = self.actuator_scnd(ACTCX, int_step)
        else:
            ACTX = ACTCX

        delax = ACTX[0]; delex = ACTX[1]; delrx = ACTX[2]

        p[619].gets(delax)
        p[620].gets(delex)
        p[621].gets(delrx)

    def actuator_scnd(self, ACTCX: Matrix, int_step: float) -> Matrix:
        p = self.plane

        dlimx = p[602].real()
        ddlimx = p[604].real()
        wnact = p[605].real()
        zetact = p[606].real()
        DXD = p[630].vec()
        DX = p[631].vec()
        DDXD = p[632].vec()
        DDX = p[633].vec()

        for i in range(3):
            if abs(DX[i]) > dlimx:
                DX[i] = dlimx * sign(DX[i])
                if DX[i] * DDX[i] > 0:
                    DDX[i] = 0
            iflag = 0
            if abs(DDX[i]) > ddlimx:
                iflag = 1
                DDX[i] = ddlimx * sign(DDX[i])
            DXD_new_i = DDX[i]
            DX[i] = integrate(DXD_new_i, DXD[i], DX[i], int_step)
            DXD[i] = DXD_new_i
            edx = ACTCX[i] - DX[i]
            DDXD_new_i = wnact * wnact * edx - 2.0 * zetact * wnact * DXD[i]
            DDX[i] = integrate(DDXD_new_i, DDXD[i], DDX[i], int_step)
            DDXD[i] = DDXD_new_i
            if iflag and DDX[i] * DDXD[i] > 0:
                DDXD[i] = 0

        ACTX = DX

        p[630].gets_vec(DXD)
        p[631].gets_vec(DX)
        p[632].gets_vec(DDXD)
        p[633].gets_vec(DDX)

        return ACTX

    # ------------------------------------------------------------------ #
    # control.cpp -- plane[500-599]
    # ------------------------------------------------------------------ #
    def def_control(self) -> None:
        p = self.plane
        p[500].init("maut", "int", 0, "maut=|mauty|mautp| see 'control' module ", "control", "data", "")
        p[501].init("mroll", "int", 0, "=0:Roll postion; =1:Roll rate control", "control", "data", "")
        p[503].init("mfreeze", "int", 0, "=0:Unfreeze; =1:Freeze; increment for more", "control", "data", "")
        p[504].init("waclp", 0.0, "Nat freq of accel close loop complex pole - rad/s", "control", "data", "")
        p[505].init("zaclp", 0.0, "Damping of accel close loop complex pole - ND", "control", "data", "")
        p[506].init("paclp", 0.0, "Close loop real pole - ND", "control", "data", "")
        p[508].init("dalimx", 0.0, "Aileron limiter - deg", "control", "data", "")
        p[509].init("delimx", 0.0, "Elevator limiter - deg", "control", "data", "")
        p[510].init("drlimx", 0.0, "Rudder limiter - deg", "control", "data", "")
        p[511].init("philimx", 0.0, "Roll angle limiter - deg", "control", "data", "")
        p[512].init("wrcl", 0.0, "Freq of roll closed loop complex pole - rad/s", "control", "data", "")
        p[513].init("zrcl", 0.0, "Damping of roll closed loop pole - ND", "control", "data", "")
        p[514].init("yyd", 0.0, "Yaw feed-foreward derivative variable- m/s", "control", "state", "")
        p[515].init("yy", 0.0, "Yaw feed-foreward integration variable- m/s", "control", "state", "")
        p[516].init("zzd", 0.0, "Pitch feed-foreward derivative variable- m/s", "control", "state", "")
        p[517].init("zz", 0.0, "Pitch feed-foreward integration variable- m/s", "control", "state", "")
        p[519].init("delacx", 0.0, "Aileron command deflection - deg", "control", "out", "plot")
        p[520].init("delecx", 0.0, "Elevator command deflection - deg", "control", "out", "plot")
        p[521].init("delrcx", 0.0, "Rudder  command deflection - deg", "control", "out", "plot")
        p[522].init("alcomx", 0.0, "Lateral (horizontal) acceleration comand - g's", "control", "data", "plot")
        p[523].init("ancomx", 0.0, "Pitch acceleration command - g's", "control", "data", "plot")
        p[524].init("GAINFP", 0, 0, 0, "Feedback gains of pitch accel controller", "control", "diag", "")
        p[525].init("gainp", 0.0, "Proportional gain in pitch acceleration loop - s^2/m", "control", "data", "")
        p[526].init("gainl", 0.0, "Gain in lateral acceleration loop - rad/g's", "control", "data", "")
        p[527].init("altcom", 0.0, "Altitude command - m", "control", "data", "plot")
        p[528].init("gainalt", 0.0, "Altitude gain - 1/s", "control", "data", "")
        p[529].init("gainaltrate", 0.0, "Altitude rate gain - 1/s", "control", "data", "")
        p[530].init("altrate", 0.0, "Altitude rate - m/s", "control", "diag", "")
        p[531].init("gkp", 0.0, "Gain of roll rate feedback - s", "control", "diag", "")
        p[532].init("gkphi", 0.0, "Gain of roll angle feedback -", "control", "diag", "")
        p[533].init("fspb2m", 0.0, "Max pitch accel transient - m/s^2", "control", "diag", "")
        p[534].init("fspb2mt", 0.0, "Stagetime at max pitch accel transient - s", "control", "diag", "")
        p[535].init("fspb3m", 0.0, "Max yaw accel transient - m/s^2", "control", "diag", "")
        p[536].init("fspb3mt", 0.0, "Stagetime at max yaw accel transient - s", "control", "diag", "")
        p[537].init("qqxm", 0.0, "Max pitch rate transient - deg/s", "control", "diag", "")
        p[538].init("qqxmt", 0.0, "Stagetime at max pitch rate transient - s", "control", "diag", "")
        p[539].init("rrxm", 0.0, "Max yaw rate transient - deg/s", "control", "diag", "")
        p[540].init("rrxmt", 0.0, "Stagetime at max yaw rate transient - s", "control", "diag", "")
        p[541].init("dqcxm", 0.0, "Max pitch flap transient - deg", "control", "diag", "")
        p[542].init("dqcxmt", 0.0, "Stagetime at max pitch flap transient - s", "control", "diag", "")
        p[543].init("drcxm", 0.0, "Max yaw flap transient - deg", "control", "diag", "")
        p[544].init("drcxmt", 0.0, "Stagetime at max yaw flap transient - s", "control", "diag", "")
        p[545].init("isetc2", 0.0, "Flag to print freeze variables", "control", "init", "")
        p[546].init("factwacl", 0.0, "Fact assoc with closed loop natural frequency", "control", "data", "")
        p[547].init("factzacl", 0.0, "Fact assoc with closed loop damping", "control", "data", "")
        p[548].init("tp", 0.0, "Desired closed loop roll rate time constant - s", "control", "data", "")
        p[549].init("zetlagr", 0.0, "Desired damping of closed rate loop ND", "control", "data", "plot")
        p[550].init("psivlcomx", 0.0, "Heading command - deg", "control", "data", "")
        p[551].init("facthead", 0.0, "Fact to reduce heading gain gainpsi*(1.+facthead) - ND", "control", "data", "")
        p[552].init("gainpsi", 0.0, "Heading control gain - ND", "control", "diag", "")
        p[553].init("phicomx", 0.0, "Roll angle command - deg", "control", "data", "")
        p[554].init("pcomx", 0.0, "Roll rate command - deg/s", "control", "data", "")
        p[555].init("qcomx", 0.0, "Pitch rate command - deg/s", "control", "data", "")
        p[556].init("rcomx", 0.0, "Yaw rate command - deg/s", "control", "data", "")
        p[557].init("thtvlcomx", 0.0, "Flight path angle command - deg", "control", "data", "")
        p[558].init("anlimpx", 0.0, "Positive structural acceleration limiter - g's", "control", "data", "")
        p[559].init("anlimnx", 0.0, "Neg structural accel limiter (data is positive) - g's", "control", "data", "")
        p[560].init("zrate", 0.0, "Pole of yaw rate TR.FCT. - 1/rad", "control", "diag", "")
        p[561].init("grate", 0.0, "Feedback gain of yaw rate loop - ND", "control", "diag", "")
        p[562].init("wnlagr", 0.0, "Nat freq of closed yaw rate loop - rad/s", "control", "diag", "")
        p[563].init("pgam", 0.0, "Loc of gamma close loop real pole (pos. stable)- rad/s", "control", "data", "")
        p[564].init("wgam", 0.0, "Nat freq of gamma close loop complex pole (- rad/s", "control", "data", "")
        p[565].init("zgam", 0.0, "Damping of gamma close loop complex pole - rad/s", "control", "data", "")
        p[566].init("GAINGAM", 0, 0, 0, "Gamma fbck gain of q, theta and gamma", "control", "diag", "")
        p[567].init("gainff", 0.0, "Gamma feed-forward gain", "control", "diag", "")

    def control(self, int_step: float) -> None:
        f, p = self.flat6, self.plane

        maut = p[500].integer()
        mroll = p[501].integer()
        dalimx = p[508].real(); delimx = p[509].real(); drlimx = p[510].real()
        philimx = p[511].real()
        alcomx = p[522].real(); ancomx = p[523].real()
        altcom = p[527].real()
        psivlcomx = p[550].real()
        phicomx = p[553].real()
        pcomx = p[554].real(); qcomx = p[555].real(); rcomx = p[556].real()
        thtvlcomx = p[557].real()
        anlimpx = p[558].real(); anlimnx = p[559].real()
        gmax = p[163].real(); gminx = p[167].real()

        delacx = delecx = delrcx = 0.0

        if maut == 0:
            return

        mauty = maut // 10
        mautp = maut % 10

        if mauty == 2:
            delrcx = self.control_yaw_rate(rcomx)
        if mautp == 2:
            delecx = self.control_pitch_rate(qcomx)

        if mauty == 3:
            phicomx = self.control_lateral_accel(alcomx)

        if mautp == 3:
            if ancomx > gmax:
                ancomx = gmax
            if ancomx < gminx:
                ancomx = gminx
            delecx = self.control_normal_accel(ancomx, int_step)

        if mautp == 4:
            delecx = self.control_gamma(thtvlcomx)

        if mauty == 4:
            phicomx = self.control_heading(psivlcomx)
            delrcx = self.control_yaw_rate(rcomx)

        if mautp == 5:
            ancomx = self.control_altitude(altcom)
            if ancomx > anlimpx:
                ancomx = anlimpx
            if ancomx < -anlimnx:
                ancomx = -anlimnx
            if ancomx > gmax:
                ancomx = gmax
            if ancomx < gminx:
                ancomx = gminx
            delecx = self.control_normal_accel(ancomx, int_step)

        if mroll == 0:
            if abs(phicomx) > philimx:
                phicomx = philimx * sign(phicomx)
            delacx = self.control_roll(phicomx)
        elif mroll == 1:
            delacx = self.control_roll_rate(pcomx)

        if abs(delacx) > dalimx:
            delacx = dalimx * sign(delacx)
        if abs(delecx) > delimx:
            delecx = delimx * sign(delecx)
        if abs(delrcx) > drlimx:
            delrcx = drlimx * sign(delrcx)

        p[519].gets(delacx)
        p[520].gets(delecx)
        p[521].gets(delrcx)
        p[523].gets(ancomx)
        p[553].gets(phicomx)

    def control_roll(self, phicomx: float) -> float:
        f, p = self.flat6, self.plane
        wrcl = p[512].real(); zrcl = p[513].real()
        phiblx = f[139].real(); ppx = f[160].real()
        dllp = p[155].real(); dllda = p[156].real()

        gkp = (2.0 * zrcl * wrcl + dllp) / dllda
        gkphi = wrcl * wrcl / dllda

        ephi = gkphi * (phicomx - phiblx) * RAD
        dpc = ephi - gkp * ppx * RAD
        delacx = dpc * DEG

        p[531].gets(gkp)
        p[532].gets(gkphi)
        return delacx

    def control_roll_rate(self, pcomx: float) -> float:
        f, p = self.flat6, self.plane
        tp = p[548].real()
        ppx = f[160].real()
        dllp = p[155].real(); dllda = p[156].real()

        kp = (1.0 / tp + dllp) / dllda
        delacx = kp * (pcomx - ppx)
        return delacx

    def control_pitch_rate(self, qcomx: float) -> float:
        f, p = self.flat6, self.plane
        zetlagr = p[549].real()
        dla = p[145].real(); dlde = p[146].real()
        dma = p[147].real(); dmq = p[148].real(); dmde = p[149].real()
        qqx = f[161].real(); dvbe = f[236].real()

        zrate = dla / dvbe - dma * dlde / (dvbe * dmde)
        aa = dla / dvbe - dmq
        bb = -dma - dmq * dla / dvbe

        dum1 = aa - 2.0 * zetlagr * zetlagr * zrate
        dum2 = aa * aa - 4.0 * zetlagr * zetlagr * bb
        radix = dum1 * dum1 - dum2
        if radix < 0.0:
            radix = 0.0
        if abs(dmde) < SMALL:
            dmde = SMALL * sign(dmde)
        grate = (-dum1 + math.sqrt(radix)) / (-dmde)

        delecx = grate * (qqx - qcomx)
        return delecx

    def control_yaw_rate(self, rcomx: float) -> float:
        f, p = self.flat6, self.plane
        zetlagr = p[549].real()
        dyb = p[150].real(); dydr = p[151].real()
        dnb = p[152].real(); dnr = p[153].real(); dndr = p[154].real()
        rrx = f[162].real(); dvbe = f[236].real()

        zrate = -dyb / dvbe + dnb * dydr / (dvbe * dndr)
        aa = -dyb / dvbe - dnr
        bb = dnb + dyb * dnr / dvbe

        dum1 = aa - 2.0 * zetlagr * zetlagr * zrate
        dum2 = aa * aa - 4.0 * zetlagr * zetlagr * bb
        radix = dum1 * dum1 - dum2
        if radix < 0.0:
            radix = 0.0
        if abs(dndr) < SMALL:
            dndr = SMALL * sign(dndr)
        grate = (-dum1 + math.sqrt(radix)) / (-dndr)

        dum3 = grate * dndr * zrate
        radix = bb + dum3
        if radix < 0.0:
            radix = 0.0
        wnlagr = math.sqrt(radix)

        delrcx = grate * (rrx - rcomx)

        p[560].gets(zrate)
        p[561].gets(grate)
        p[562].gets(wnlagr)
        return delrcx

    def control_normal_accel(self, ancomx: float, int_step: float) -> float:
        f, p = self.flat6, self.plane

        gainp = p[525].real()
        pdynmc = f[57].real()
        dla = p[145].real(); dma = p[147].real(); dmq = p[148].real(); dmde = p[149].real()
        qqx = f[161].real()
        FSPB = f[230].vec()
        dvbe = f[236].real()
        zzd = p[516].real(); zz = p[517].real()

        waclp = 2 + 0.0001226 * (pdynmc - 5500)
        zaclp = 0.7 - 0.0000245 * (pdynmc - 5500)
        paclp = 5 + 0.0003067 * (pdynmc - 5500)

        gainfb3 = waclp * waclp * paclp / (dla * dmde)
        gainfb2 = (2.0 * zaclp * waclp + paclp + dmq - dla / dvbe) / dmde
        gainfb1 = ((waclp * waclp + 2.0 * zaclp * waclp * paclp + dma + dmq * dla / dvbe
                    - gainfb2 * dmde * dla / dvbe) / (dla * dmde) - gainp)

        fspb3 = FSPB[2]
        zzd_new = AGRAV * ancomx + fspb3
        zz = integrate(zzd_new, zzd, zz, int_step)
        zzd = zzd_new
        dqc = -gainfb1 * (-fspb3) - gainfb2 * qqx * RAD + gainfb3 * zz + gainp * zzd
        delecx = dqc * DEG

        GAINFP = Matrix().build_vec3(gainfb1, gainfb2, gainfb3)

        p[516].gets(zzd)
        p[517].gets(zz)
        p[524].gets_vec(GAINFP)
        p[504].gets(waclp); p[505].gets(zaclp); p[506].gets(paclp)

        return delecx

    def control_lateral_accel(self, alcomx: float) -> float:
        f, p = self.flat6, self.plane
        gainl = p[526].real()
        grav = f[55].real()
        FSPB = f[230].vec()
        fspb3 = FSPB[2]
        phicomx = -DEG * gainl * math.atan(alcomx) * sign(fspb3)
        return phicomx

    def control_gamma(self, thtvlcomx: float) -> float:
        f, p = self.flat6, self.plane

        pgam = p[563].real(); wgam = p[564].real(); zgam = p[565].real()
        pdynmc = f[57].real()
        thtblx = f[138].real()
        qqx = f[161].real()
        thtvlx = f[241].real()
        dvbe = f[236].real()
        dla = p[145].real(); dlde = p[146].real()
        dma = p[147].real(); dmq = p[148].real(); dmde = p[149].real()

        AA = Matrix().build_mat33(dmq, dma, -dma, 1.0, 0.0, 0.0, 0.0, dla / dvbe, -dla / dvbe)
        BB = Matrix().build_vec3(dmde, 0.0, dlde / dvbe)

        am = 2.0 * zgam * wgam + pgam
        bm = wgam * wgam + 2.0 * zgam * wgam * pgam
        cm = wgam * wgam * pgam
        v11 = dmde
        v12 = 0.0
        v13 = dlde / dvbe
        v21 = dmde * dla / dvbe - dlde * dma / dvbe
        v22 = dmde
        v23 = -dmq * dlde / dvbe
        v31 = 0.0
        v32 = v21
        v33 = v21
        DP = Matrix().build_mat33(v11, v12, v13, v21, v22, v23, v31, v32, v33)
        DD = Matrix().build_vec3(am + dmq - dla / dvbe, bm + dma + dmq * dla / dvbe, cm)
        DPI = DP.inverse()
        GAINGAM = DPI * DD

        DUM33 = AA - BB * (~GAINGAM)
        IDUM33 = DUM33.inverse()
        DUM3 = IDUM33 * BB
        HH = Matrix().build_vec3(0.0, 0.0, 1.0)
        denom = HH ^ DUM3
        gainff = -1.0 / denom

        thtc = gainff * thtvlcomx * RAD
        qqf = GAINGAM[0] * qqx * RAD
        thtblf = GAINGAM[1] * thtblx * RAD
        thtvlf = GAINGAM[2] * thtvlx * RAD
        delec = thtc - (qqf + thtblf + thtvlf)
        delecx = delec * DEG

        p[566].gets_vec(GAINGAM)
        p[567].gets(gainff)
        return delecx

    def control_heading(self, psivlcomx: float) -> float:
        f, p = self.flat6, self.plane
        wrcl = p[512].real(); zrcl = p[513].real()
        facthead = p[551].real()
        grav = f[55].real()
        dvbe = f[236].real()
        psivlx = f[240].real()

        gainpsi = (dvbe / grav) * zrcl * wrcl * (1.0 - zrcl * zrcl) * (1.0 + facthead)
        phicomx = gainpsi * (psivlcomx - psivlx)

        p[552].gets(gainpsi)
        return phicomx

    def control_altitude(self, altcom: float) -> float:
        f, p = self.flat6, self.plane
        gainalt = p[528].real(); gainaltrate = p[529].real()
        grav = f[55].real()
        phiblx = f[139].real()
        VBEL = f[233].vec()
        hbe = f[239].real()

        altrate = -VBEL[2]

        eh = gainalt * (altcom - hbe)
        if phiblx == 0:
            phiblx = SMALL
        ancomx = (1.0 / math.cos(phiblx * RAD)) * (gainaltrate * (eh - altrate) + grav) / AGRAV

        p[530].gets(altrate)
        return ancomx

    # ------------------------------------------------------------------ #
    # guidance.cpp -- plane[400-499]
    # ------------------------------------------------------------------ #
    def def_guidance(self) -> None:
        p = self.plane
        p[400].init("mguid", "int", 0, "Switch for guidance options - ND", "guidance", "data", "scrn")
        p[402].init("line_gain", 0.0, "Line guidance gain - 1/s", "guidance", "data", "")
        p[403].init("nl_gain_fact", 0.0, "Nonlinear gain factor - ND", "guidance", "data", "")
        p[404].init("decrement", 0.0, "distance decrement - m", "guidance", "data", "")
        p[405].init("swel1", 0.0, "North coordiante of way point - m", "guidance", "data", "")
        p[406].init("swel2", 0.0, "East coordinate of way point - m", "guidance", "data", "")
        p[407].init("swel3", 0.0, "Altitude of way point - m", "guidance", "data", "")
        p[408].init("psiflx", 0.0, "Heading line-of-attack angle - deg", "guidance", "data", "")
        p[409].init("thtflx", 0.0, "Pitch line-of-attack angle - deg", "guidance", "data", "")
        p[411].init("dwb", 0.0, "Slant range to waypoint - m", "guidance", "diag", "")
        p[412].init("nl_gain", 0.0, "Nonlinear gain - rad", "guidance", "diag", "scrn,plot")
        p[413].init("VBEO", 0, 0, 0, "Vehicle velocity in LOS coordinats - m/s", "guidance", "diag", "")
        p[414].init("VBEF", 0, 0, 0, "Vehicle velocity in LOA coordinats - m/s", "guidance", "diag", "")
        p[415].init("dwbh", 0.0, "Ground range to waypoint - m", "guidance", "diag", "scrn,plot")
        p[416].init("SWBL", 0, 0, 0, "Vehicle wrt waypoint/target in geo coor - m", "guidance", "diag", "")
        p[417].init("turn_min", 0.0, "Minimum turning radius - m", "guidance", "diag", "")
        p[419].init("wp_flag", "int", 0, "=1:closing on trgt; =-1:fleeting; =0:outside - ND", "guidance", "diag", "plot")

    def guidance(self) -> None:
        f, p = self.flat6, self.plane

        mguid = p[400].integer()
        swel1 = p[405].real(); swel2 = p[406].real(); swel3 = p[407].real()
        psiflx = p[408].real(); thtflx = p[409].real()
        halt = f[1].integer()
        grav = f[55].real()
        SBEL = f[219].vec()
        VBEL = f[233].vec()
        dvbe = f[236].real()
        psivlx = f[240].real()
        thtvlx = f[241].real()
        philimx = p[511].real()

        if mguid == 0:
            return

        mguidl = mguid // 10
        mguidp = mguid % 10

        SWEL = Matrix().build_vec3(swel1, swel2, swel3)
        SWBL = SWEL - SBEL
        dwb = SWBL.absolute()
        dwbh = math.sqrt(SWBL[0] * SWBL[0] + SWBL[1] * SWBL[1])

        wp_flag = 1
        turn_min = dvbe * dvbe / (grav * math.tan(philimx * RAD))

        if dwbh < (2 * turn_min):
            closing = SWBL[0] * VBEL[0] + SWBL[1] * VBEL[1]
            wp_flag = sign(closing)
            if wp_flag < 0 and wp_flag == -1:
                if mguid == 30:
                    print(f" *** Closest horizontal approach to waypoint dwbh = {dwbh} m")
                    print(f"     Heading angle psivlx = {psivlx} deg")
                if mguidp == 3 or mguid == 33:
                    print(f" *** Closest approach to IP dwb = {dwb} m")
                    print(f"     Heading angle psivlx = {psivlx} deg")
                    print(f"     Flight path angle thtvlx = {thtvlx} deg")

        alcomx = 0.0
        ancomx = 0.0
        ALGV = self.guidance_line(SWBL, psiflx, thtflx)
        if mguidl == 3:
            alcomx = ALGV[1] / grav
        if mguidp == 3:
            ancomx = -ALGV[2] / grav

        p[411].gets(dwb)
        p[415].gets(dwbh)
        p[416].gets_vec(SWBL)
        p[417].gets(turn_min)
        p[419].gets(int(wp_flag))
        p[522].gets(alcomx)
        p[523].gets(ancomx)

    def guidance_line(self, SWBL: Matrix, psiflx: float, thtflx: float) -> Matrix:
        f, p = self.flat6, self.plane

        line_gain = p[402].real()
        nl_gain_fact = p[403].real()
        decrement = p[404].real()
        grav = f[55].real()
        VBEL = f[233].vec()
        thtvlx = f[241].real()

        TFL = mat2tr(psiflx * RAD, thtflx * RAD)

        POLAR = SWBL.pol_from_cart()
        wp_sltrange = POLAR[0]
        psiol = POLAR[1]
        thtol = POLAR[2]
        TOL = mat2tr(psiol, thtol)

        VBEO = TOL * VBEL
        vbeo2 = VBEO[1]; vbeo3 = VBEO[2]

        VBEF = TFL * VBEL
        vbef2 = VBEF[1]; vbef3 = VBEF[2]

        nl_gain = nl_gain_fact * (1 - math.exp(-wp_sltrange / decrement))

        algv1 = grav * math.sin(thtvlx * RAD)
        algv2 = line_gain * (-vbeo2 + nl_gain * vbef2)
        algv3 = line_gain * (-vbeo3 + nl_gain * vbef3) - grav * math.cos(thtvlx * RAD)

        ALGV = Matrix().build_vec3(algv1, algv2, algv3)

        p[411].gets(wp_sltrange)
        p[412].gets(nl_gain)
        p[413].gets_vec(VBEO)
        p[414].gets_vec(VBEF)

        return ALGV

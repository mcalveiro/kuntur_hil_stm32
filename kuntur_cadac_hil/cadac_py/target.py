"""
Port of 'class_hierarchy.hpp' -> class 'Target' (TARGET3, 3-DoF).

Ported so far (faithful, 1:1 with the .cpp source):
    - flat3_modules.cpp -> Flat3.def_environment/environment/
                           def_newton/init_newton/newton   (COMPLETE,
                           see cadac.py -- Target inherits Flat3)
    - target_modules.cpp -> Target.def_guidance/guidance   (COMPLETE)
                            Target.def_control/control      (COMPLETE)
                            Target.def_forces/forces         (COMPLETE)

Target is the second-level derived class (Flat3 -> Target) modeling the
attacked aircraft: a simple 3-DoF point-mass with a banked-turn
approximation for lateral maneuvers (no full 6-DoF rotational dynamics --
def_kinematics/kinematics/def_euler/euler are dummy no-ops in the
original, see Flat3 in cadac.py). It has 3 active modules of its own
(guidance/control/forces) plus the inherited Flat3 environment/newton.

Target::intercept()/seeker()/aerodynamics()/propulsion()/actuator()/tvc()
are all dummy no-ops in the original C++ (class_hierarchy.hpp lines
447-482) -- Target is the passive/maneuvering side of the engagement,
never the shooter -- so they are ported here as no-ops too, for
interface parity with Missile.

Not yet ported: Target::vehicle_data()/sizing_arrays()/com_index_arrays()/
loading_packet()/document() (target_functions.cpp) -- these are
input.asc-parsing and combus-packet-building infrastructure that belongs
to execution.cpp's territory, still pending.
"""
from __future__ import annotations

import math

from .cadac import Flat3, NTARGET, RAD, DEG, EPS, _new_variable_array
from .matrix import Matrix, sign
from .integrate import integrate


class Target(Flat3):
    """Derived class 'Target' (class_hierarchy.hpp) -- 3-DoF target
    aircraft: guidance (steady/turn/escape maneuvers), control (bank
    angle + normal load factor lag and limiting), forces (specific
    force from load factor)."""

    def __init__(self):
        super().__init__()
        self.target = _new_variable_array(NTARGET)

    # ------------------------------------------------------------------ #
    # dummy returns of unused modules (class_hierarchy.hpp: Target)
    # ------------------------------------------------------------------ #
    def def_aerodynamics(self) -> None:
        pass

    def init_aerodynamics(self) -> None:
        pass

    def aerodynamics(self) -> None:
        pass

    def def_propulsion(self) -> None:
        pass

    def propulsion(self) -> None:
        pass

    def def_actuator(self) -> None:
        pass

    def actuator(self, int_step: float) -> None:
        pass

    def def_tvc(self) -> None:
        pass

    def tvc(self, int_step: float) -> None:
        pass

    def def_seeker(self) -> None:
        pass

    def seeker(self, combus, num_vehicles: int, int_step: float) -> None:
        pass

    def def_intercept(self) -> None:
        pass

    def intercept(self, combus, vehicle_slot: int, int_step: float, title: str) -> None:
        pass

    # ------------------------------------------------------------------ #
    # target_modules.cpp  (Target::def_guidance / guidance)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_guidance(self) -> None:
        t = self.target
        t[1].init("msl_num", "int", 0, "Missile tail number attacking this tgt - ND", "guidance", "data", "")
        t[10].init("tgt_option", "int", 0, "=0:steady manvr; =1 hor g-manvr; =2:escape - ND", "guidance", "data", "")
        t[11].init("guid_gain", 0.0, "Guidance gain for target maneuvers - ND", "guidance", "data", "")
        t[12].init("ACOML", 0, 0, 0, "Commanded accel in loc lev coord - m/s^2", "guidance", "out", "")
        t[13].init("gturn", 0.0, "G-accel for horiz turn (+ right, - left) - g's", "guidance", "data", "")

    def guidance(self, combus, num_vehicles: int) -> None:
        t, f = self.target, self.flat3
        msl_num = t[1].integer()
        tgt_option = t[10].integer()
        guid_gain = t[11].real()
        gturn = t[13].real()
        grav = f[11].real()
        TVL = f[24].mat()
        SAEL = f[26].vec()
        VAEL = f[27].vec()
        # -----------------------------------------------------------
        ACOML = Matrix(3, 1)

        if tgt_option == 0:
            ACOML = Matrix().build_vec3(0.0, 0.0, -grav)

        if tgt_option == 1:
            ACOMV = Matrix().build_vec3(0.0, gturn * grav, -grav)
            ACOML = (~TVL) * ACOMV

        if tgt_option == 2:
            data_msl = combus[msl_num - 1].get_data()
            mseek = data_msl[6].integer()

            if mseek == 4:
                SBEL = data_msl[2].vec()
                VBEL = data_msl[3].vec()

                SABL = SAEL - SBEL
                dab = SABL.absolute()

                dum = (VAEL.skew_sym() * VBEL).absolute()
                gain = guid_gain * dum / dab

                dvbe = VBEL.absolute()
                UVBEL = VBEL * (1.0 / dvbe)
                dvae = VAEL.absolute()
                UVAEL = VAEL * (1.0 / dvae)

                EPSL = UVAEL.skew_sym() * UVBEL
                ACOML = EPSL.skew_sym() * UVAEL * gain

                GRAVL = Matrix().build_vec3(0.0, 0.0, -grav)
                ACOML = ACOML + GRAVL
        # -----------------------------------------------------------
        t[12].gets_vec(ACOML)

    # ------------------------------------------------------------------ #
    # target_modules.cpp  (Target::def_control / control)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_control(self) -> None:
        t, f = self.target, self.flat3
        t[20].init("phiav", 0.0, "Target aircraft bank angle - rad", "control", "state", "")
        t[21].init("phiavd", 0.0, "Derivative of bank angle - rad/s", "control", "state", "")
        t[22].init("tphi", 0.2, "Time lag constant of bank angle - sec", "control", "data", "")
        t[23].init("philimx", 120.0, "Bank angle limiter - deg", "control", "data", "")
        t[24].init("phiavx", 0.0, "Bank angle - deg", "control", "out", "com")
        t[25].init("phiavcx", 0.0, "Commanded bank angle - deg", "control", "diag", "")
        t[26].init("anx", 0.0, "Normal load factor - g's", "control", "state", "com")
        t[27].init("anxd", 0.0, "Normal load factor derivative - g's/s", "control", "state", "")
        t[28].init("tanx", 0.1, "Time lag constant of normal load factor  - sec", "control", "data", "")
        t[29].init("alplimx", 40.0, "Angle of attack limiter - deg", "control", "data", "")
        t[30].init("ancomx", 0.0, "Commanded load factor - g's", "control", "diag", "")
        t[31].init("clalpha", 0.0523, "Aircraft lift slope - 1/deg", "control", "data", "")
        t[32].init("wingloading", 3247.0, "Aircraft wing loading - N/m^2", "control", "data", "")
        # feedback path to 'Flat3'
        f[21].init("phiavout", 0.0, "Target aircraft bank angle - rad", "control", "out", "")

    def control(self, int_step: float) -> None:
        t, f = self.target, self.flat3
        tphi = t[22].real()
        philimx = t[23].real()
        tanx = t[28].real()
        alplimx = t[29].real()
        clalpha = t[31].real()
        wingloading = t[32].real()
        grav = f[11].real()
        pdynmc = f[13].real()
        TVL = f[24].mat()
        tgt_option = t[10].integer()
        ACOML = t[12].vec()
        phiav = t[20].real()
        phiavd = t[21].real()
        anx = t[26].real()
        anxd = t[27].real()
        # -----------------------------------------------------------
        ACOMV = TVL * ACOML

        acoma2 = ACOMV.get_loc(1, 0)
        acoma3 = ACOMV.get_loc(2, 0)
        if abs(acoma2) < EPS and abs(acoma3) < EPS:
            phiavc = 0.0
        else:
            phiavc = math.atan2(acoma2, -acoma3)
        phiavcx = phiavc * DEG

        if tphi:
            phiavd_new = (phiavc - phiav) / tphi
            phiav = integrate(phiavd_new, phiavd, phiav, int_step)
            phiavd = phiavd_new
        else:
            phiav = phiavc

        phiavx = phiav * DEG
        if abs(phiavx) >= philimx:
            phiavx = philimx * sign(phiavx)

        phiavout = phiavx * RAD

        ancomx = math.sqrt(acoma2 * acoma2 + acoma3 * acoma3) / grav

        if tanx:
            anxd_new = (ancomx - anx) / tanx
            anx = integrate(anxd_new, anxd, anx, int_step)
            anxd = anxd_new
        else:
            anx = ancomx
        if tgt_option > 0:
            anlimx = pdynmc * clalpha * alplimx / (wingloading * grav)
            if abs(anx) >= anlimx:
                anx = anlimx * sign(anx)
        # -----------------------------------------------------------
        t[20].gets(phiav)
        t[21].gets(phiavd)
        t[26].gets(anx)
        t[27].gets(anxd)
        f[21].gets(phiavout)
        t[24].gets(phiavx)
        t[25].gets(phiavcx)
        t[30].gets(ancomx)

    # ------------------------------------------------------------------ #
    # target_modules.cpp  (Target::def_forces / forces)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_forces(self) -> None:
        t, f = self.target, self.flat3
        f[20].init("FSPA", 0, 0, 0, "Specific force - m/s^2", "forces", "out", "")
        t[40].init("acc_longx", 0.0, "Longitudinal acceleration - g's", "forces", "data", "")

    def forces(self) -> None:
        t, f = self.target, self.flat3
        acc_longx = t[40].real()
        grav = f[11].real()
        anx = t[26].real()
        # -----------------------------------------------------------
        acoma1 = acc_longx * grav
        acoma2 = 0.0
        acoma3 = -anx * grav
        FSPA = Matrix().build_vec3(acoma1, acoma2, acoma3)
        # -----------------------------------------------------------
        f[20].gets_vec(FSPA)

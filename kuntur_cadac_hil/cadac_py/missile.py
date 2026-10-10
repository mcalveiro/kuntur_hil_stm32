"""
Port of 'class_hierarchy.hpp' -> class 'Missile' (SRAAM6).

Ported so far (faithful, 1:1 with the .cpp source):
    - propulsion.cpp   -> def_propulsion / propulsion       (COMPLETE)
    - forces.cpp       -> def_forces / forces               (COMPLETE)
    - aerodynamics.cpp -> def_aerodynamics / init_aerodynamics /
                          aerodynamics / aerodynamics_der    (COMPLETE)

    With aerodynamics.cpp complete, the vehicle now has real lift, side
    force and all three moments as a function of Mach/alpha/roll-attitude
    from the real 2-D aero tables, plus the on-line aerodynamic-derivative
    /airframe-root calculation used by an aero-adaptive autopilot
    (aerodynamics_der()). It still will NOT steer, because the autopilot
    (control), fin dynamics (actuator/tvc), seeker and guidance that would
    *command* a non-zero fin deflection (dpx/dqx/drx) are not ported yet --
    those variables are declared with inert defaults (0) below so
    aerodynamics() has what it needs to run standalone. A vehicle with
    zero commanded deflection but real aerodynamics will still show
    realistic weathervaning/restoring-moment behavior from its own
    static margin, unlike the fully-zeroed placeholder this module used
    to be.

Not yet ported (raise NotImplementedError with a pointer to the source):
    - actuator.cpp, tvc.cpp, control.cpp, guidance.cpp, seeker.cpp,
      intercept.cpp

    Their module-variables that 'aerodynamics()'/'forces()' depend on
    (mtvc, FPB, FMPB, alimit, dpx, dqx, drx) are declared here with inert
    defaults so the physics chain that IS implemented (environment ->
    kinematics -> newton -> euler -> propulsion -> aerodynamics ->
    forces) runs end-to-end.
"""
from __future__ import annotations

import math

from .cadac import Flat6, NMISSILE, _new_variable_array, DEG, RAD, AGRAV, SMALL
from .matrix import Matrix, sign, mat2tr
from .tables import Datadeck
from .integrate import integrate


class Missile(Flat6):
    def __init__(self):
        super().__init__()
        self.missile = _new_variable_array(NMISSILE)
        self.aerotable: Datadeck | None = None
        self.proptable: Datadeck | None = None

    # ------------------------------------------------------------------ #
    # propulsion.cpp  (Missile::def_propulsion / Missile::propulsion)
    # ------------------------------------------------------------------ #
    def def_propulsion(self) -> None:
        m = self.missile
        m[50].init("mprop", "int", 0, "=0: Motor off, =1:Motor on", "propulsion", "data", "")
        m[51].init("aexit", 0.0, "Nozzle exit area - m^2", "propulsion", "data", "")
        m[59].init("vmass", 92.0, "Vehicle mass - kg", "propulsion", "out", "plot")
        m[63].init("thrust", 0.0, "Rocket thrust parallel to vehicle centerline - N", "propulsion", "out", "plot")
        m[64].init("xcgref", 1.536, "Launch CG aft of vehicle nose - m", "propulsion", "init", "")
        m[70].init("xcg", 1.536, "Vehicle CG aft of vehicle nose - m", "propulsion", "diag", "")
        m[71].init("ai11", 0.308, "Roll moment of inertia - kg*m^2", "propulsion", "out", "plot")
        m[72].init("ai33", 59.80, "Pitch/Yaw moment of inertia - kg*m^2", "propulsion", "out", "plot")
        m[73].init("mfreeze_prop", "int", 0, "Saving mfreeze variable", "propulsion", "save", "")
        m[74].init("thrustf", 0.0, "Saved thrust when mfreez=1 - N", "propulsion", "save", "")
        m[75].init("vmassf", 0.0, "Saved mass when mfreez=1 - kg", "propulsion", "save", "")
        m[76].init("xcgf", 0.0, "Saved cg when mfreez=1 - m", "propulsion", "save", "")
        m[77].init("ai11f", 0.0, "Saved ai11 when mfreez=1 - kgm^2", "propulsion", "save", "")
        m[78].init("ai33f", 0.0, "Saved ai33 when mfreez=1 - kgm^2", "propulsion", "save", "")

    def propulsion(self) -> None:
        m, f = self.missile, self.flat6
        psl = 101325.0  # sea level pressure - Pa

        mprop = m[50].integer()
        aexit = m[51].real()
        vmass = m[59].real()
        xcg = m[70].real()
        ai11 = m[71].real()
        ai33 = m[72].real()
        mfreeze_prop = m[73].integer()
        thrustf = m[74].real()
        vmassf = m[75].real()
        xcgf = m[76].real()
        ai11f = m[77].real()
        ai33f = m[78].real()
        mfreeze = m[501].integer()
        time = f[0].real()
        press = f[52].real()
        # -----------------------------------------------------------
        if mprop == 1:
            tsl = self.proptable.look_up("thrust_vs_time", time)
            thrust = tsl + (psl - press) * aexit
            vmass = self.proptable.look_up("mass_vs_time", time)
            xcg = self.proptable.look_up("cg_vs_time", time)
            ai33 = self.proptable.look_up("moipitch_vs_time", time)
            ai11 = self.proptable.look_up("moiroll_vs_time", time)
            if time > 2.69:
                mprop = 0
        else:
            thrust = 0.0

        if mfreeze == 0:
            mfreeze_prop = 0
        else:
            if mfreeze != mfreeze_prop:
                mfreeze_prop = mfreeze
                thrustf, vmassf, xcgf, ai11f, ai33f = thrust, vmass, xcg, ai11, ai33
            thrust, vmass, xcg, ai11, ai33 = thrustf, vmassf, xcgf, ai11f, ai33f
        # -----------------------------------------------------------
        m[50].gets(mprop)
        m[63].gets(thrust)
        m[59].gets(vmass)
        m[70].gets(xcg)
        m[71].gets(ai11)
        m[72].gets(ai33)
        m[73].gets(mfreeze_prop)
        m[74].gets(thrustf)
        m[75].gets(vmassf)
        m[76].gets(xcgf)
        m[77].gets(ai11f)
        m[78].gets(ai33f)

    # ------------------------------------------------------------------ #
    # forces.cpp  (Missile::def_forces / Missile::forces)
    # ------------------------------------------------------------------ #
    def def_forces(self) -> None:
        f = self.flat6
        f[200].init("FAPB", 0, 0, 0, "Aerodynamic and propulsive forces in body axes - N", "forces", "out", "")
        f[201].init("FMB", 0, 0, 0, "Aerodynamic and propulsive moments in body axes - N*m", "forces", "out", "")

    def forces(self) -> None:
        f, m = self.flat6, self.missile
        time = f[0].real()
        pdynmc = f[57].real()
        thrust = m[63].real()
        refl = m[103].real()
        refa = m[104].real()
        ca = m[120].real()
        cy = m[121].real()
        cn = m[122].real()
        cll = m[123].real()
        clm = m[124].real()
        cln = m[125].real()
        mtvc = m[700].integer()
        FPB = m[710].vec()
        FMPB = m[711].vec()
        # -----------------------------------------------------------
        FAPB = Matrix(3, 1)
        FMB = Matrix(3, 1)

        FAPB[0] = -pdynmc * refa * ca
        FAPB[1] = pdynmc * refa * cy
        FAPB[2] = -pdynmc * refa * cn
        if mtvc == 0:
            FAPB[0] = FAPB[0] + thrust
        else:
            FAPB = FAPB + FPB

        FMB[0] = pdynmc * refa * refl * cll
        FMB[1] = pdynmc * refa * refl * clm
        FMB[2] = pdynmc * refa * refl * cln
        if mtvc != 0:
            FMB = FMB + FMPB
        # -----------------------------------------------------------
        f[200].gets_vec(FAPB)
        f[201].gets_vec(FMB)

    # ------------------------------------------------------------------ #
    # aerodynamics.cpp  (Missile::def_aerodynamics / init_aerodynamics /
    #                    aerodynamics / aerodynamics_der)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_aerodynamics(self) -> None:
        m = self.missile
        m[103].init("refl", 0.0, "Reference length for moment derivatives - m", "aerodynamics", "init", "")
        m[104].init("refa", 0.0, "Reference area for aero coefficients - m^2", "aerodynamics", "init", "")
        m[120].init("ca", 0.0, "Axial force coefficient", "aerodynamics", "out", "")
        m[121].init("cy", 0.0, "Side force coefficient", "aerodynamics", "out", "")
        m[122].init("cn", 0.0, "Normal force coefficient", "aerodynamics", "out", "")
        m[123].init("cll", 0.0, "Rolling moment coefficient", "aerodynamics", "out", "")
        m[124].init("clm", 0.0, "Pitching moment coefficient", "aerodynamics", "out", "")
        m[125].init("cln", 0.0, "Yawing moment coefficient", "aerodynamics", "out", "")
        m[130].init("cn0", 0.0, "Normal force coeff as function of Mach and alpha", "aerodynamics", "diag", "")
        m[131].init("cnp", 0.0, "Normal force coeff of roll attitude", "aerodynamics", "diag", "")
        m[132].init("clm0", 0.0, "Pitching moment coeff as a function of Mach and alpha", "aerodynamics", "diag", "")
        m[133].init("clmp", 0.0, "Pitching moment coeff of roll attitude", "aerodynamics", "diag", "")
        m[134].init("cyp", 0.0, "Side force coeff of roll attitude", "aerodynamics", "diag", "")
        m[135].init("clnp", 0.0, "Yawing moment coeff of roll attitude", "aerodynamics", "diag", "")
        m[136].init("ca0", 0.0, "Axial force coeff at zero incidence", "aerodynamics", "diag", "")
        m[137].init("caa", 0.0, "Axial force coeff alpha derivative - 1/deg", "aerodynamics", "diag", "")
        m[138].init("cad", 0.0, "Axial force coeff of control surface drag - 1/deg^2", "aerodynamics", "diag", "")
        m[139].init("cndq", 0.0, "Normal force coeff of pitch control deflection - 1/deg", "aerodynamics", "diag", "")
        m[140].init("clmdq", 0.0, "Pitching moment coeff of pitch control deflec - 1/deg", "aerodynamics", "diag", "")
        m[141].init("clmq", 0.0, "Pitching moment coeff pitch damping - 1/deg", "aerodynamics", "diag", "")
        m[142].init("cllap", 0.0, "Rolling moment coeff of incidence angle - 1/deg^2", "aerodynamics", "diag", "")
        m[143].init("clldp", 0.0, "Rolling moment coeff of roll control deflec - 1/deg", "aerodynamics", "diag", "")
        m[144].init("cllp", 0.0, "Rolling moment coeff of roll damping - 1/deg", "aerodynamics", "diag", "")
        m[145].init("dna", 0.0, "Normal force slope derivative - m/s^2", "aerodynamics", "out", "")
        m[146].init("dnd", 0.0, "Pitch control force derivative - m/s^2", "aerodynamics", "out", "")
        m[147].init("dma", 0.0, "Pitch moment derivative - 1/s^2", "aerodynamics", "out", "")
        m[148].init("dmq", 0.0, "Pitch damping derivative - 1/s", "aerodynamics", "out", "")
        m[149].init("dmd", 0.0, "Pitch control derivative - 1/s^2", "aerodynamics", "out", "")
        m[150].init("dlp", 0.0, "Roll damping derivative - 1/s", "aerodynamics", "out", "")
        m[151].init("dld", 0.0, "Roll control derivative - 1/s^2", "aerodynamics", "out", "")
        m[152].init("cna", 0.0, "Normal force coeff in aeroballistic axes", "aerodynamics", "diag", "")
        m[153].init("cya", 0.0, "Side force coeff in aeroballistic axes", "aerodynamics", "diag", "")
        m[154].init("clma", 0.0, "Pitching moment coeff in aeroballistic axes", "aerodynamics", "diag", "")
        m[155].init("clna", 0.0, "Yawing moment coeff in aeroballistic axes", "aerodynamics", "diag", "")
        m[159].init("stmarg", 0.0, "Static margin (+stable, -unstable) - diameter", "aerodynamics", "diag", "plot")
        m[160].init("realq1", 0.0, "First real root of airframe pitch dynamics - rad/s", "aerodynamics", "diag", "plot")
        m[161].init("realq2", 0.0, "Second real root of airframe pitch dynamics - rad/s", "aerodynamics", "diag", "plot")
        m[162].init("wnq", 0.0, "Natural frequency of airframe pitch dynamics - rad/s", "aerodynamics", "diag", "plot")
        m[163].init("zetq", 0.0, "Damping of airframe pitch dynamics - NA", "aerodynamics", "diag", "plot")
        m[164].init("realp", 0.0, "Real root of airframe roll dynamics - rad/s", "aerodynamics", "diag", "")
        m[165].init("alplimx", 0.0, "Maximum total alpha permissible - deg", "aerodynamics", "data", "")
        m[166].init("gavail", 0.0, "Maneuver headroom - g's", "aerodynamics", "diag", "plot")
        m[167].init("gmax", 0.0, "Max maneuverability limited by ALPLIMX - g's", "aerodynamics", "diag", "plot")
        m[168].init("pqreal", 0.0, "Real part or mean value (real roots) of airf dyn - rad", "aerodynamics", "diag", "")
        m[180].init("trcode", 0.0, "Termination code number", "aerodynamics", "init", "")
        m[181].init("tmcode", 0.0, "Dummy variable initialized to zero", "aerodynamics", "data", "")
        m[182].init("trcvel", 0.0, "Minimum closing speed - m/s", "aerodynamics", "data", "")
        m[183].init("trmach", 0.0, "Minimum Mach number", "aerodynamics", "data", "")
        m[184].init("trdynm", 0.0, "Minimum dynamic pressure - Pa", "aerodynamics", "data", "")
        m[185].init("trload", 0.0, "Minimum load capacity - g's", "aerodynamics", "data", "")
        m[186].init("tralp", 0.0, "Maximum total angle of attack - rad", "aerodynamics", "data", "")
        m[187].init("trtht", 0.0, "Maximum pitch gimbal angle - rad", "aerodynamics", "data", "")
        m[189].init("trthtd", 0.0, "Maximum pitch gimbal rate - rad/s", "aerodynamics", "data", "")
        m[190].init("trphid", 0.0, "Maximum roll gimbal rate - rad/s", "aerodynamics", "data", "")
        m[191].init("trate", 0.0, "Maximum tracking rate - rad/s", "aerodynamics", "data", "")

    def init_aerodynamics(self) -> None:
        m = self.missile
        refl = 0.1524
        refa = 0.01824

        trcvel = 0.0
        trmach = 0.8
        trdynm = 10.0e3
        trload = 3.0
        tralp = 1.0
        trtht = 1.57
        trphid = 14.0
        trthtd = 10.0
        trate = 1.0

        trcode = 0.0
        tmcode = 0.0

        m[103].gets(refl)
        m[104].gets(refa)
        m[182].gets(trcvel)
        m[183].gets(trmach)
        m[184].gets(trdynm)
        m[185].gets(trload)
        m[186].gets(tralp)
        m[187].gets(trtht)
        m[189].gets(trthtd)
        m[190].gets(trphid)
        m[191].gets(trate)
        m[180].gets(trcode)
        m[181].gets(tmcode)

    def aerodynamics(self) -> None:
        f, m = self.flat6, self.missile
        alplimx = m[165].real()
        refl = m[103].real()
        refa = m[104].real()
        trcode = m[180].real()
        trload = m[185].real()
        time = f[0].real()
        vmach = f[56].real()
        pdynmc = f[57].real()
        alppx = f[140].real()
        phip = f[143].real()
        ppx = f[160].real()
        qqx = f[161].real()
        rrx = f[162].real()
        dvbe = f[236].real()
        mprop = m[50].integer()
        vmass = m[59].real()
        xcgref = m[64].real()
        xcg = m[70].real()
        alimit = m[507].real()
        dpx = m[619].real()
        dqx = m[620].real()
        drx = m[621].real()
        # -----------------------------------------------------------
        cphip = math.cos(phip)
        sphip = math.sin(phip)
        dqax = dqx * cphip - drx * sphip
        drax = dqx * sphip + drx * cphip

        qqax = qqx * cphip - rrx * sphip
        rrax = qqx * sphip + rrx * cphip

        ca0 = self.aerotable.look_up("ca0_vs_mach", vmach)
        caa = self.aerotable.look_up("caa_vs_mach", vmach)
        cad = self.aerotable.look_up("cad_vs_mach", vmach)
        caoff = self.aerotable.look_up("caoff_vs_mach", vmach)

        deff = (abs(dqax) + abs(drax)) / 2.0
        ca = ca0 + caa * alppx + cad * deff * deff + (1 - mprop) * caoff

        cyp = self.aerotable.look_up("cyp_vs_mach_alpha", vmach, alppx)
        cydr = self.aerotable.look_up("cndq_vs_mach_alpha", vmach, alppx)

        s4phi = math.sin(4.0 * phip)
        cya = cyp * s4phi + cydr * drax

        cn0 = self.aerotable.look_up("cn0_vs_mach_alpha", vmach, alppx)
        cnp = self.aerotable.look_up("cnp_vs_mach_alpha", vmach, alppx)
        cndq = self.aerotable.look_up("cndq_vs_mach_alpha", vmach, alppx)

        s2phi = math.sin(2.0 * phip) ** 2
        cna = cn0 + cnp * s2phi + cndq * dqax

        cllap = self.aerotable.look_up("cllap_vs_mach_alpha", vmach, alppx)
        cllp = self.aerotable.look_up("cllp_vs_mach_alpha", vmach, alppx)
        clldp = self.aerotable.look_up("clldp_vs_mach_alpha", vmach, alppx)

        cll = cllap * alppx * alppx * s4phi + cllp * ppx * refl / (2.0 * dvbe) + clldp * dpx

        clm0 = self.aerotable.look_up("clm0_vs_mach_alpha", vmach, alppx)
        clmp = self.aerotable.look_up("clmp_vs_mach_alpha", vmach, alppx)
        clmq = self.aerotable.look_up("clmq_vs_mach", vmach)
        clmdq = self.aerotable.look_up("clmdq_vs_mach_alpha", vmach, alppx)

        clmaref = clm0 + clmp * s2phi + clmq * qqax * refl / (2.0 * dvbe) + clmdq * dqax
        clma = clmaref - cna * (xcgref - xcg) / refl

        clnp = self.aerotable.look_up("clnp_vs_mach_alpha", vmach, alppx)

        clnr = clmq
        clndr = clmdq
        clnaref = clnp * s4phi + clnr * rrax * refl / (2.0 * dvbe) + clndr * drax
        clna = clnaref - cya * (xcgref - xcg) / refl

        cy = cya * cphip - cna * sphip
        cn = cya * sphip + cna * cphip

        clm = clma * cphip + clna * sphip
        cln = -clma * sphip + clna * cphip

        cn0mx = self.aerotable.look_up("cn0_vs_mach_alpha", vmach, alplimx)
        almx = cn0mx * pdynmc * refa
        weight = vmass * AGRAV
        gmax = almx / weight
        if gmax >= alimit:
            gmax = alimit
        aload = cn0 * pdynmc * refa
        gg = aload / weight
        gavail = gmax - gg

        if gmax < trload:
            trcode = 4.0
        # -----------------------------------------------------------
        m[120].gets(ca)
        m[121].gets(cy)
        m[122].gets(cn)
        m[123].gets(cll)
        m[124].gets(clm)
        m[125].gets(cln)
        m[167].gets(gmax)
        m[130].gets(cn0)
        m[131].gets(cnp)
        m[132].gets(clm0)
        m[133].gets(clmp)
        m[134].gets(cyp)
        m[136].gets(ca0)
        m[135].gets(clnp)
        m[137].gets(caa)
        m[138].gets(cad)
        m[139].gets(cndq)
        m[140].gets(clmdq)
        m[141].gets(clmq)
        m[142].gets(cllap)
        m[143].gets(clldp)
        m[144].gets(cllp)
        m[152].gets(cna)
        m[153].gets(cya)
        m[154].gets(clma)
        m[155].gets(clna)
        m[166].gets(gavail)
        m[180].gets(trcode)
        # -----------------------------------------------------------
        self.aerodynamics_der()

    def aerodynamics_der(self) -> None:
        f, m = self.flat6, self.missile
        alplimx = m[165].real()
        refl = m[103].real()
        refa = m[104].real()
        dna = m[145].real()
        dnd = m[146].real()
        dma = m[147].real()
        dmq = m[148].real()
        dmd = m[149].real()
        dlp = m[150].real()
        dld = m[151].real()
        stmarg = m[159].real()
        time = f[0].real()
        vmach = f[56].real()
        pdynmc = f[57].real()
        alppx = f[140].real()
        phip = f[143].real()
        dvbe = f[236].real()
        vmass = m[59].real()
        xcgref = m[64].real()
        xcg = m[70].real()
        ai11 = m[71].real()
        ai33 = m[72].real()
        cndq = m[139].real()
        clmdq = m[140].real()
        clmq = m[141].real()
        cllp = m[144].real()
        clldp = m[143].real()
        # -----------------------------------------------------------
        if alppx < (alplimx - 3.0):
            alpp = alppx + 3.0
            if alpp < 3.0:
                alpp = 3.0
            alpm = alppx - 3.0
            if alpm < 0.0:
                alpm = 0.0

            cn0p = self.aerotable.look_up("cn0_vs_mach_alpha", vmach, alpp)
            cn0m = self.aerotable.look_up("cn0_vs_mach_alpha", vmach, alpm)
            dum = cn0p - cn0m
            cna = DEG * dum / (alpp - alpm)

            cnd = DEG * cndq

            clm0p = self.aerotable.look_up("clm0_vs_mach_alpha", vmach, alpp)
            clm0m = self.aerotable.look_up("clm0_vs_mach_alpha", vmach, alpm)
            cma = DEG * (clm0p - clm0m) / (alpp - alpm) - cna * (xcgref - xcg) / refl

            cmq = DEG * clmq
            cmd = DEG * clmdq
            clp = DEG * cllp
            cld = DEG * clldp

            dumn = pdynmc * refa / vmass
            dna = dumn * cna
            dnd = dumn * cnd

            dumm = pdynmc * refa * refl / ai33
            dma = dumm * cma
            dmq = dumm * (refl / (2.0 * dvbe)) * cmq
            dmd = dumm * cmd

            duml = pdynmc * refa * refl / ai11
            dlp = duml * (refl / (2.0 * dvbe)) * clp
            dld = duml * cld

            stmarg = -cma / cna

        a11 = dmq
        a12 = dma / dna
        a21 = dna
        a22 = -dna / dvbe

        arg = (a11 + a22) ** 2 - 4.0 * (a11 * a22 - a12 * a21)
        if arg >= 0.0:
            wnq = 0.0
            zetq = 0.0
            dum = a11 + a22
            realq1 = (dum + math.sqrt(arg)) / 2.0
            realq2 = (dum - math.sqrt(arg)) / 2.0
            pqreal = (realq1 + realq2) / 2.0
        else:
            realq1 = 0.0
            realq2 = 0.0
            wnq = math.sqrt(a11 * a22 - a12 * a21)
            zetq = -(a11 + a22) / (2.0 * wnq)
            pqreal = -zetq * wnq

        realp = dlp
        # -----------------------------------------------------------
        m[145].gets(dna)
        m[146].gets(dnd)
        m[147].gets(dma)
        m[148].gets(dmq)
        m[149].gets(dmd)
        m[150].gets(dlp)
        m[151].gets(dld)
        m[159].gets(stmarg)
        m[160].gets(realq1)
        m[161].gets(realq2)
        m[162].gets(wnq)
        m[163].gets(zetq)
        m[164].gets(realp)
        m[168].gets(pqreal)

    # ------------------------------------------------------------------ #
    # actuator.cpp  (Missile::def_actuator / actuator / actuator_scnd)
    #   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_actuator(self) -> None:
        m = self.missile
        m[600].init("mact", "int", 0, "=0:no dynamics, =2:second order", "actuator", "data", "")
        m[602].init("dlimx", 0.0, "Control fin limiter - deg", "actuator", "data", "")
        m[604].init("ddlimx", 0.0, "Control fin rate limiter - deg/s", "actuator", "data", "")
        m[605].init("wnact", 0.0, "Natural frequency of actuator - rad/s", "actuator", "data", "")
        m[606].init("zetact", 0.0, "Damping of actuator - ND", "actuator", "data", "")
        m[619].init("dpx", 0.0, "Roll control deflection - deg", "actuator", "out", "plot")
        m[620].init("dqx", 0.0, "Pitch control deflection - deg", "actuator", "out", "plot")
        m[621].init("drx", 0.0, "Yaw control deflection - deg", "actuator", "out", "plot")
        m[622].init("delx1", 0.0, "Fin #1 output position - deg", "actuator", "diag", "plot")
        m[623].init("delx2", 0.0, "Fin #2 output position - deg", "actuator", "diag", "")
        m[624].init("delx3", 0.0, "Fin #3 output position - deg", "actuator", "diag", "")
        m[625].init("delx4", 0.0, "Fin #4 output position - deg", "actuator", "diag", "")
        m[630].init("dxd1", 0.0, "Fin #1 position derivative - deg/s", "actuator", "state", "")
        m[631].init("dxd2", 0.0, "Fin #2 position derivative - deg/s", "actuator", "state", "")
        m[632].init("dxd3", 0.0, "Fin #3 position derivative - deg/s", "actuator", "state", "")
        m[633].init("dxd4", 0.0, "Fin #4 position derivative - deg/s", "actuator", "state", "")
        m[634].init("dx1", 0.0, "Fin #1 position - deg", "actuator", "state", "")
        m[635].init("dx2", 0.0, "Fin #2 position - deg", "actuator", "state", "")
        m[636].init("dx3", 0.0, "Fin #3 position - deg", "actuator", "state", "")
        m[637].init("dx4", 0.0, "Fin #4 position - deg", "actuator", "state", "")
        m[638].init("ddxd1", 0.0, "Fin #1 rate derivative - deg/s^2", "actuator", "state", "")
        m[639].init("ddxd2", 0.0, "Fin #2 rate derivative - deg/s^2", "actuator", "state", "")
        m[640].init("ddxd3", 0.0, "Fin #3 rate derivative - deg/s^2", "actuator", "state", "")
        m[641].init("ddxd4", 0.0, "Fin #4 rate derivative - deg/s^2", "actuator", "state", "")
        m[642].init("ddx1", 0.0, "Fin #1 rate - deg/s", "actuator", "state", "plot")
        m[643].init("ddx2", 0.0, "Fin #2 rate - deg/s", "actuator", "state", "")
        m[644].init("ddx3", 0.0, "Fin #3 rate - deg/s", "actuator", "state", "")
        m[645].init("ddx4", 0.0, "Fin #4 rate - deg/s", "actuator", "state", "")
        m[646].init("delcx1", 0.0, "Fin #1 position command - deg", "actuator", "diag", "")
        m[647].init("delcx2", 0.0, "Fin #2 position command - deg", "actuator", "diag", "")
        m[648].init("delcx3", 0.0, "Fin #3 position command - deg", "actuator", "diag", "")
        m[649].init("delcx4", 0.0, "Fin #4 position command - deg", "actuator", "diag", "")

    def actuator(self, int_step: float) -> None:
        m, f = self.missile, self.flat6
        mact = m[600].integer()
        dlimx = m[602].real()
        time = f[0].real()
        dpcx = m[519].real()
        dqcx = m[520].real()
        drcx = m[521].real()
        # -----------------------------------------------------------
        delcx1 = -dpcx + dqcx - drcx
        delcx2 = -dpcx + dqcx + drcx
        delcx3 = dpcx + dqcx - drcx
        delcx4 = dpcx + dqcx + drcx

        if mact < 2:
            delx1 = delcx1
            if abs(delx1) > dlimx:
                delx1 = dlimx * (1 if delx1 >= 0 else -1)
            delx2 = delcx2
            if abs(delx2) > dlimx:
                delx2 = dlimx * (1 if delx2 >= 0 else -1)
            delx3 = delcx3
            if abs(delx3) > dlimx:
                delx3 = dlimx * (1 if delx3 >= 0 else -1)
            delx4 = delcx4
            if abs(delx4) > dlimx:
                delx4 = dlimx * (1 if delx4 >= 0 else -1)

            dpx = (-delx1 - delx2 + delx3 + delx4) / 4.0
            dqx = (delx1 + delx2 + delx3 + delx4) / 4.0
            drx = (-delx1 + delx2 - delx3 + delx4) / 4.0
            # -----------------------------------------------------------
            m[619].gets(dpx)
            m[620].gets(dqx)
            m[621].gets(drx)
            m[622].gets(delx1)
            m[623].gets(delx2)
            m[624].gets(delx3)
            m[625].gets(delx4)
            m[646].gets(delcx1)
            m[647].gets(delcx2)
            m[648].gets(delcx3)
            m[649].gets(delcx4)

        if mact == 2:
            self.actuator_scnd(int_step)

    def actuator_scnd(self, int_step: float) -> None:
        m, f = self.missile, self.flat6
        dlimx = m[602].real()
        ddlimx = m[604].real()
        wnact = m[605].real()
        zetact = m[606].real()
        time = f[0].real()
        dpcx = m[519].real()
        dqcx = m[520].real()
        drcx = m[521].real()
        dxd1, dxd2, dxd3, dxd4 = m[630].real(), m[631].real(), m[632].real(), m[633].real()
        dx1, dx2, dx3, dx4 = m[634].real(), m[635].real(), m[636].real(), m[637].real()
        ddxd1, ddxd2, ddxd3, ddxd4 = m[638].real(), m[639].real(), m[640].real(), m[641].real()
        ddx1, ddx2, ddx3, ddx4 = m[642].real(), m[643].real(), m[644].real(), m[645].real()
        # -----------------------------------------------------------
        delcx1 = -dpcx + dqcx - drcx
        delcx2 = -dpcx + dqcx + drcx
        delcx3 = dpcx + dqcx - drcx
        delcx4 = dpcx + dqcx + drcx

        def _fin(dx, ddx, delc, dxd, ddxd):
            if abs(dx) > dlimx:
                dx = dlimx * (1 if dx >= 0 else -1)
                if dx * ddx > 0.0:
                    ddx = 0.0
            iflag = False
            if abs(ddx) > ddlimx:
                iflag = True
                ddx = ddlimx * (1 if ddx >= 0 else -1)
            dxd_new = ddx
            dx = integrate(dxd_new, dxd, dx, int_step)
            dxd = dxd_new
            edx = delc - dx
            ddxd_new = wnact * wnact * edx - 2.0 * zetact * wnact * dxd
            ddx = integrate(ddxd_new, ddxd, ddx, int_step)
            ddxd = ddxd_new
            if iflag and ddx * ddxd > 0.0:
                ddxd = 0.0
            return dx, ddx, dxd, ddxd

        dx1, ddx1, dxd1, ddxd1 = _fin(dx1, ddx1, delcx1, dxd1, ddxd1)
        dx2, ddx2, dxd2, ddxd2 = _fin(dx2, ddx2, delcx2, dxd2, ddxd2)
        dx3, ddx3, dxd3, ddxd3 = _fin(dx3, ddx3, delcx3, dxd3, ddxd3)
        dx4, ddx4, dxd4, ddxd4 = _fin(dx4, ddx4, delcx4, dxd4, ddxd4)

        delx1, delx2, delx3, delx4 = dx1, dx2, dx3, dx4

        dpx = (-delx1 - delx2 + delx3 + delx4) / 4.0
        dqx = (delx1 + delx2 + delx3 + delx4) / 4.0
        drx = (-delx1 + delx2 - delx3 + delx4) / 4.0
        # -----------------------------------------------------------
        m[630].gets(dxd1); m[631].gets(dxd2); m[632].gets(dxd3); m[633].gets(dxd4)
        m[634].gets(dx1); m[635].gets(dx2); m[636].gets(dx3); m[637].gets(dx4)
        m[638].gets(ddxd1); m[639].gets(ddxd2); m[640].gets(ddxd3); m[641].gets(ddxd4)
        m[642].gets(ddx1); m[643].gets(ddx2); m[644].gets(ddx3); m[645].gets(ddx4)
        m[619].gets(dpx)
        m[620].gets(dqx)
        m[621].gets(drx)
        m[622].gets(delx1)
        m[623].gets(delx2)
        m[624].gets(delx3)
        m[625].gets(delx4)
        m[646].gets(delcx1)
        m[647].gets(delcx2)
        m[648].gets(delcx3)
        m[649].gets(delcx4)

    # ------------------------------------------------------------------ #
    # tvc.cpp  (Missile::def_tvc / tvc / tvc_scnd)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_tvc(self) -> None:
        m = self.missile
        m[700].init("mtvc", "int", 0, "=0:no TVC;=1:no dyn;=2:scnd order;=3:2+gain", "tvc", "data", "")
        m[702].init("tvclimx", 0.0, "Nozzle deflection limiter - deg", "tvc", "data", "")
        m[704].init("dtvclimx", 0.0, "Nozzle deflection rate limiter - deg/s", "tvc", "data", "")
        m[705].init("wntvc", 0.0, "Natural frequency of TVC - rad/s", "tvc", "data", "")
        m[706].init("zettvc", 0.0, "Damping of TVC - ND", "tvc", "data", "")
        m[707].init("factgtvc", 0.0, "Factor for TVC gain - ND", "tvc", "data", "")
        m[708].init("gtvc", 0.0, "TVC nozzle deflection gain - ND", "tvc", "data,diag", "")
        m[709].init("parm", 0.0, "Propulsion moment arm from vehicle nose - m", "tvc", "data", "")
        m[710].init("FPB", 0, 0, 0, "Thrust force in body axes - N", "tvc", "out", "")
        m[711].init("FMPB", 0, 0, 0, "Thrust moment in body axes - Nm", "tvc", "out", "")
        m[712].init("etax", 0.0, "Nozzle pitch deflection - deg", "tvc", "diag", "plot")
        m[713].init("zetx", 0.0, "Nozzle yaw deflection - deg", "tvc", "diag", "plot")
        m[714].init("etacx", 0.0, "Commanded nozzle pitch deflection - deg", "tvc", "diag", "plot")
        m[715].init("zetcx", 0.0, "Commanded nozzle yaw deflection - deg", "tvc", "diag", "plot")
        m[730].init("etasd", 0.0, "Pitch nozzle derivative - rad/s", "tvc", "state", "")
        m[731].init("zetad", 0.0, "Yaw nozzle derivative - rad/s", "tvc", "state", "")
        m[734].init("etas", 0.0, "Pitch nozzle deflection - rad", "tvc", "state", "")
        m[735].init("zeta", 0.0, "Yaw nozzle deflection - rad", "tvc", "state", "")
        m[738].init("detasd", 0.0, "Pitch nozzle rate derivative - rad/s^2", "tvc", "state", "")
        m[739].init("dzetad", 0.0, "Yaw nozzle rate derivative - rad/s^2", "tvc", "state", "")
        m[742].init("detas", 0.0, "Pitch nozzle rate - rad/s", "tvc", "state", "plot")
        m[743].init("dzeta", 0.0, "Yaw nozzle rate - rad/s", "tvc", "state", "plot")

    def tvc(self, int_step: float) -> None:
        m, f = self.missile, self.flat6
        mtvc = m[700].integer()
        factgtvc = m[707].real()
        gtvc = m[708].real()
        parm = m[709].real()
        time = f[0].real()
        thrust = m[63].real()
        xcg = m[70].real()
        dqcx = m[520].real()
        drcx = m[521].real()
        pdynmc = f[57].real()
        # -----------------------------------------------------------
        if mtvc == 0:
            return

        if mtvc == 3:
            if pdynmc > 1e5:
                gtvc = 0.0
            else:
                gtvc = (-5.0e-6 * pdynmc + 0.5) * (factgtvc + 1)

        etac = gtvc * dqcx * RAD
        zetc = gtvc * drcx * RAD

        if mtvc == 1:
            eta, zet = etac, zetc
        else:
            eta, zet = 0.0, 0.0  # placeholders, overwritten by tvc_scnd below

        if mtvc >= 2:
            eta, zet = self.tvc_scnd(etac, zetc, int_step)

        seta, ceta = math.sin(eta), math.cos(eta)
        czet, szet = math.cos(zet), math.sin(zet)
        FPB = Matrix(3, 1)
        FPB[0] = ceta * czet * thrust
        FPB[1] = ceta * szet * thrust
        FPB[2] = -seta * thrust

        arm = parm - xcg
        FMPB = Matrix(3, 1)
        FMPB[0] = 0.0
        FMPB[1] = arm * FPB[2]
        FMPB[2] = -arm * FPB[1]

        etax = eta * DEG
        zetx = zet * DEG
        etacx = etac * DEG
        zetcx = zetc * DEG
        # -----------------------------------------------------------
        m[710].gets_vec(FPB)
        m[711].gets_vec(FMPB)
        m[712].gets(etax)
        m[713].gets(zetx)
        m[714].gets(etax)  # NOTE: ported verbatim -- the original also
        m[715].gets(zetx)  # stores etax/zetx (not etacx/zetcx) at 714/715

    def tvc_scnd(self, etac: float, zetc: float, int_step: float) -> tuple[float, float]:
        m = self.missile
        tvclimx = m[702].real()
        dtvclimx = m[704].real()
        wntvc = m[705].real()
        zettvc = m[706].real()
        etasd = m[730].real()
        zetad = m[731].real()
        etas = m[734].real()
        zeta = m[735].real()
        detasd = m[738].real()
        dzetad = m[739].real()
        detas = m[742].real()
        dzeta = m[743].real()
        # -----------------------------------------------------------
        if abs(etas) > tvclimx * RAD:
            etas = tvclimx * RAD * (1 if etas >= 0 else -1)
            if etas * detas > 0.0:
                detas = 0.0
        iflag = False
        if abs(detas) > dtvclimx * RAD:
            iflag = True
            detas = dtvclimx * RAD * (1 if detas >= 0 else -1)
        etasd_new = detas
        etas = integrate(etasd_new, etasd, etas, int_step)
        etasd = etasd_new
        eetas = etac - etas
        detasd_new = wntvc * wntvc * eetas - 2.0 * zettvc * wntvc * etasd
        detas = integrate(detasd_new, detasd, detas, int_step)
        detasd = detasd_new
        if iflag and detas * detasd > 0.0:
            detasd = 0.0
        eta = etas

        if abs(zeta) > tvclimx * RAD:
            zeta = tvclimx * RAD * (1 if zeta >= 0 else -1)
            if zeta * dzeta > 0.0:
                dzeta = 0.0
        iflag = False
        if abs(dzeta) > dtvclimx * RAD:
            iflag = True
            dzeta = dtvclimx * RAD * (1 if dzeta >= 0 else -1)
        zetad_new = dzeta
        zeta = integrate(zetad_new, zetad, zeta, int_step)
        zetad = zetad_new
        ezeta = zetc - zeta
        dzetad_new = wntvc * wntvc * ezeta - 2.0 * zettvc * wntvc * zetad
        dzeta = integrate(dzetad_new, dzetad, dzeta, int_step)
        dzetad = dzetad_new
        if iflag and dzeta * dzetad > 0.0:
            dzetad = 0.0
        zet = zeta
        # -----------------------------------------------------------
        m[730].gets(etasd)
        m[731].gets(zetad)
        m[734].gets(etas)
        m[735].gets(zeta)
        m[738].gets(detasd)
        m[739].gets(dzetad)
        m[742].gets(detas)
        m[743].gets(dzeta)
        return eta, zet

    # ------------------------------------------------------------------ #
    # control.cpp  (Missile::def_control / control / control_roll /
    #               control_rate / control_accel)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_control(self) -> None:
        m = self.missile
        m[500].init("maut", "int", 0, "=2:Rate; =3:Accel controller", "control", "data", "")
        m[501].init("mfreeze", "int", 0, "=0:Unfreeze; =1:Freeze; increment for more", "control", "data", "")
        m[504].init("wacl", 0.0, "Nat freq of accel close loop complex pole - rad/s", "control", "data", "")
        m[505].init("zacl", 0.0, "Damping of accel close loop complex pole - ND", "control", "data", "")
        m[506].init("pacl", 0.0, "Close loop real pole - ND", "control", "data", "")
        m[507].init("alimit", 0.0, "Total structural acceleration limiter - g's", "control", "data", "")
        m[508].init("dqlimx", 0.0, "Pitch flap control limiter - deg", "control", "data", "")
        m[509].init("drlimx", 0.0, "Yaw flap control limiter - deg", "control", "data", "")
        m[510].init("dplimx", 0.0, "Roll command limiter - deg", "control", "data", "")
        m[511].init("phicomx", 0.0, "Commanded roll angle - deg", "control", "data", "plot")
        m[512].init("wrcl", 0.0, "Freq of roll closed loop complex pole - rad/s", "control", "data", "")
        m[513].init("zrcl", 0.0, "Damping of roll closed loop pole - ND", "control", "data", "")
        m[514].init("yyd", 0.0, "Yaw feed-forward derivative variable- m/s", "control", "state", "")
        m[515].init("yy", 0.0, "Yaw feed-forward integration variable- m/s", "control", "state", "")
        m[516].init("zzd", 0.0, "Pitch feed-forward derivative variable- m/s", "control", "state", "")
        m[517].init("zz", 0.0, "Pitch feed-forward integration variable- m/s", "control", "state", "")
        m[519].init("dpcx", 0.0, "Roll flap command deflection - deg", "control", "out", "plot")
        m[520].init("dqcx", 0.0, "Pitch flap command deflection - deg", "control", "out", "plot")
        m[521].init("drcx", 0.0, "Yaw flap command deflection - deg", "control", "out", "plot")
        m[524].init("GAINFB", 0, 0, 0, "Feedback gain of rate, accel and control", "control", "diag", "")
        m[525].init("gainp", 0.0, "Feed-forward gain - s^2/m", "control", "data", "")
        m[531].init("gkp", 0.0, "Gain of roll rate feedback - s", "control", "diag", "")
        m[532].init("gkphi", 0.0, "Gain of roll angle feedback -", "control", "diag", "")
        m[533].init("fspb2m", 0.0, "Max pitch accel transient - m/s^2", "control", "diag", "")
        m[534].init("fspb2mt", 0.0, "Stagetime at max pitch accel transient - s", "control", "diag", "")
        m[535].init("fspb3m", 0.0, "Max yaw accel transient - m/s^2", "control", "diag", "")
        m[536].init("fspb3mt", 0.0, "Stagetime at max yaw accel transient - s", "control", "diag", "")
        m[537].init("qqxm", 0.0, "Max pitch rate transient - deg/s", "control", "diag", "")
        m[538].init("qqxmt", 0.0, "Stagetime at max pitch rate transient - s", "control", "diag", "")
        m[539].init("rrxm", 0.0, "Max yaw rate transient - deg/s", "control", "diag", "")
        m[540].init("rrxmt", 0.0, "Stagetime at max yaw rate transient - s", "control", "diag", "")
        m[541].init("dqcxm", 0.0, "Max pitch flap transient - deg", "control", "diag", "")
        m[542].init("dqcxmt", 0.0, "Stagetime at max pitch flap transient - s", "control", "diag", "")
        m[543].init("drcxm", 0.0, "Max yaw flap transient - deg", "control", "diag", "")
        m[544].init("drcxmt", 0.0, "Stagetime at max yaw flap transient - s", "control", "diag", "")
        m[545].init("isetc2", 0.0, "Flag to print freeze variables", "control", "init", "")
        m[546].init("factwacl", 0.0, "Fact assoc with closed loop natural frequency", "control", "data", "")
        m[547].init("factzacl", 0.0, "Fact assoc with closed loop damping", "control", "data", "")
        m[550].init("zetlagr", 0.0, "Desired damping of closed rate loop ND", "control", "data", "")
        m[553].init("ratelimx", 0.0, "Rate command limiter - deg/s", "control", "data", "")
        m[560].init("zrate", 0.0, "Zero pole of rate TR.FCT. - 1/rad", "control", "diag", "")
        m[561].init("grate", 0.0, "Feedback gain of rate loop - ND", "control", "diag", "")
        m[562].init("wnlagr", 0.0, "Nat freq of closed rate loop - rad/s", "control", "diag", "")

    def control(self, int_step: float) -> None:
        m = self.missile
        maut = m[500].integer()
        # -----------------------------------------------------------
        if maut == 0:
            return

        self.control_roll()

        if maut == 2:
            self.control_rate()

        if maut == 3:
            self.control_accel(int_step)

    def control_roll(self) -> None:
        m, f = self.missile, self.flat6
        phicomx = m[511].real()
        wrcl = m[512].real()
        zrcl = m[513].real()
        time = f[0].real()
        phiblx = f[139].real()
        dlp = m[150].real()
        dld = m[151].real()
        pp = f[155].real()
        # -----------------------------------------------------------
        gkp = (2.0 * zrcl * wrcl + dlp) / dld
        gkphi = wrcl * wrcl / dld

        ephi = gkphi * (phicomx - phiblx) * RAD
        dpc = ephi - gkp * pp
        dpcx = dpc * DEG
        # -----------------------------------------------------------
        m[519].gets(dpcx)
        m[531].gets(gkp)
        m[532].gets(gkphi)

    def control_rate(self) -> None:
        m, f = self.missile, self.flat6
        zetlagr = m[550].real()
        time = f[0].real()
        qq = f[157].real()
        rr = f[159].real()
        dvbe = f[236].real()
        dna = m[145].real()
        dnd = m[146].real()
        dma = m[147].real()
        dmq = m[148].real()
        dmd = m[149].real()
        # -----------------------------------------------------------
        zrate = dna / dvbe - dma * dnd / (dvbe * dmd)
        aa = dna / dvbe - dmq
        bb = -dma - dmq * dna / dvbe

        dum1 = aa - 2.0 * zetlagr * zetlagr * zrate
        dum2 = aa * aa - 4.0 * zetlagr * zetlagr * bb
        radix = dum1 * dum1 - dum2
        if radix < 0.0:
            radix = 0.0
        if abs(dmd) < SMALL:
            dmd = SMALL * sign(dmd)
        grate = (-dum1 + math.sqrt(radix)) / (-dmd)

        dum3 = grate * dmd * zrate
        radix = bb + dum3
        if radix < 0.0:
            radix = 0.0
        wnlagr = math.sqrt(radix)

        dqcx = DEG * grate * qq
        drcx = DEG * grate * rr
        # -----------------------------------------------------------
        m[520].gets(dqcx)
        m[521].gets(drcx)
        m[560].gets(zrate)
        m[561].gets(grate)
        m[562].gets(wnlagr)

    def control_accel(self, int_step: float) -> None:
        m, f = self.missile, self.flat6
        wacl = m[504].real()
        zacl = m[505].real()
        pacl = m[506].real()
        alimit = m[507].real()
        gainp = m[525].real()
        factwacl = m[546].real()
        factzacl = m[547].real()
        time = f[0].real()
        pdynmc = f[57].real()
        ancomx = m[402].real()
        alcomx = m[403].real()
        dna = m[145].real()
        dma = m[147].real()
        dmq = m[148].real()
        dmd = m[149].real()
        FSPB = f[230].vec()
        dvbe = f[236].real()
        qq = f[157].real()
        rr = f[159].real()
        yyd = m[514].real()
        yy = m[515].real()
        zzd = m[516].real()
        zz = m[517].real()
        # -----------------------------------------------------------
        aa = math.sqrt(alcomx * alcomx + ancomx * ancomx)
        if aa > alimit:
            aa = alimit
        if abs(ancomx) < SMALL and abs(alcomx) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ancomx, alcomx)
        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)

        wacl = (0.013 * math.sqrt(pdynmc) + 7.1) * (factwacl + 1)
        zacl = (0.559e-3 * math.sqrt(pdynmc) + 0.232) * (factzacl + 1)
        pacl = 14

        gainfb3 = wacl * wacl * pacl / (dna * dmd)
        gainfb2 = (2.0 * zacl * wacl + pacl + dmq - dna / dvbe) / dmd
        gainfb1 = (wacl * wacl + 2.0 * zacl * wacl * pacl + dma + dmq * dna / dvbe
                   - gainfb2 * dmd * dna / dvbe) / (dna * dmd) - gainp
        GAINFB = Matrix().build_vec3(gainfb1, gainfb2, gainfb3)

        fspb3 = FSPB.get_loc(2, 0)
        zzd_new = AGRAV * ancomx + fspb3
        zz = integrate(zzd_new, zzd, zz, int_step)
        zzd = zzd_new
        dqc = -gainfb1 * (-fspb3) - gainfb2 * qq + gainfb3 * zz
        dqcx = dqc * DEG

        fspb2 = FSPB.get_loc(1, 0)
        yyd_new = AGRAV * alcomx - fspb2
        yy = integrate(yyd_new, yyd, yy, int_step)
        yyd = yyd_new
        drc = -gainfb1 * fspb2 - gainfb2 * rr + gainfb3 * yy
        drcx = drc * DEG
        # -----------------------------------------------------------
        m[514].gets(yyd)
        m[515].gets(yy)
        m[516].gets(zzd)
        m[517].gets(zz)
        m[520].gets(dqcx)
        m[521].gets(drcx)
        m[504].gets(wacl)
        m[505].gets(zacl)
        m[506].gets(pacl)
        m[524].gets_vec(GAINFB)

    # ------------------------------------------------------------------ #
    # guidance.cpp  (Missile::def_guidance / guidance / guidance_mid /
    #                guidance_term)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_guidance(self) -> None:
        m = self.missile
        m[400].init("mguid", "int", 0, "=0:None, =3:Pro-Nav, =6:Comp Pro-Nav", "guidance", "data", "")
        m[401].init("gnav", 0.0, "Navigation gain - ND", "guidance", "data", "")
        m[402].init("ancomx", 0.0, "Normal acceleration command - g's", "guidance", "out", "scrn,plot")
        m[403].init("alcomx", 0.0, "Lateral acceleration command - g's", "guidance", "out", "scrn,plot")
        m[404].init("gn", 0.0, "Guidance Gain - m/s", "guidance", "diag", "")
        m[405].init("mnav", "int", 0, "=0: Reset, =3:Update", "guidance", "data", "")
        m[406].init("apny", 0.0, "Pronav acceleration along P2 axis - m/s^2", "guidance", "diag", "plot")
        m[407].init("apnz", 0.0, "Pronav acceleration along P3 axis - m/s^2", "guidance", "diag", "")
        m[408].init("adely", 0.0, "Vehicle longit accel correction term along P2 - m/s^2", "guidance", "diag", "plot")
        m[409].init("adelz", 0.0, "Vehicle longit accel correction term along P3 - m/s^2", "guidance", "diag", "")
        m[410].init("all", 0.0, "Vehicle lateral accel before limiting - m/s^2", "guidance", "diag", "plot")
        m[411].init("ann", 0.0, "Vehicle normal accel before limiting - m/s^2", "guidance", "diag", "")
        m[412].init("epchta", 0.0, "Epoch of target data receipt - sec", "guidance", "save", "")
        m[440].init("WOELC", 0, 0, 0, "O LOS rate computed from kinematic data - rad/s", "guidance", "out", "")
        m[443].init("tgoc", 0.0, "Time-to-go, computed - s", "guidance", "diag", "plot")
        m[444].init("dtbc", 0.0, "Distance-to-target, computed - m", "guidance", "diag", "plot")
        m[445].init("dvtbc", 0.0, "Closing speed, computed - m/s", "guidance", "diag", "plot")
        m[446].init("psiobcx", 0.0, "Yaw LOS angle wrt missile - deg", "guidance", "diag", "plot")
        m[447].init("thtobcx", 0.0, "Pitch LOS angle wrt missile - deg", "guidance", "diag", "plot")
        m[448].init("UTBLC", 0, 0, 0, "LOS unit vector from extrapolated data - NA", "guidance", "out", "")
        m[455].init("STELC", 0, 0, 0, "Target location, extrapol onboard missile - m", "guidance", "diag", "")
        m[458].init("STBLC", 0, 0, 0, "Target wrt Missile position, extrapolated - m", "guidance", "diag", "")
        m[459].init("STELM", 0, 0, 0, "Stored target position - m", "guidance", "save", "")
        m[460].init("VTELC", 0, 0, 0, "Stored target velocity - m/s", "guidance", "save", "")

    def guidance(self, combus, num_vehicles: int) -> None:
        m, f = self.missile, self.flat6
        mguid = m[400].integer()
        mnav = m[405].integer()
        epchta = m[412].real()
        STELM = m[459].vec()
        VTELC = m[460].vec()
        time = f[0].real()
        STEL = m[2].vec()
        VTEL = m[3].vec()
        SBEL = f[219].vec()
        # -----------------------------------------------------------
        if mnav == 3:
            mnav = 0
            epchta = time
            STELM = STEL
            VTELC = VTEL

        dtime = time - epchta
        DUM3 = VTELC * dtime
        STELC = STELM + DUM3
        STBLC = STELC - SBEL

        if mguid == 3:
            self.guidance_mid(STBLC, VTELC)

        if mguid == 6:
            self.guidance_term()
        # -----------------------------------------------------------
        m[405].gets(mnav)
        m[412].gets(epchta)
        m[459].gets_vec(STELM)
        m[460].gets_vec(VTELC)
        m[455].gets_vec(STELC)
        m[458].gets_vec(STBLC)

    def guidance_mid(self, STBLC: Matrix, VTELC: Matrix) -> None:
        m, f = self.missile, self.flat6
        gnav = m[401].real()
        TBL = f[120].mat()
        VBEL = f[233].vec()
        # -----------------------------------------------------------
        dtbc = STBLC.absolute()

        UTBLC = STBLC * (1.0 / dtbc)
        UTBBC = TBL * UTBLC

        POLAR = UTBBC.pol_from_cart()
        psiobcx = POLAR.get_loc(1, 0) * DEG
        thtobcx = POLAR.get_loc(2, 0) * DEG

        VTBLC = VTELC - VBEL

        dvtbc = abs(UTBLC ^ VTBLC)

        tgoc = dtbc / dvtbc

        WOELC = UTBLC.skew_sym() * VTBLC * (1.0 / dtbc)

        AAPNB = TBL * WOELC.skew_sym() * UTBLC * gnav * dvtbc

        ancomx = -AAPNB.get_loc(2, 0) / AGRAV
        alcomx = AAPNB.get_loc(1, 0) / AGRAV
        # -----------------------------------------------------------
        m[402].gets(ancomx)
        m[403].gets(alcomx)
        m[440].gets_vec(WOELC)
        m[448].gets_vec(UTBLC)
        m[443].gets(tgoc)
        m[444].gets(dtbc)
        m[445].gets(dvtbc)
        m[446].gets(psiobcx)
        m[447].gets(thtobcx)

    def guidance_term(self) -> None:
        m, f = self.missile, self.flat6
        gnav = m[401].real()
        time = f[0].real()
        STEL = m[2].vec()
        VTEL = m[3].vec()
        thtpb = m[247].real()
        psipb = m[248].real()
        sigdpy = m[287].real()
        sigdpz = m[288].real()
        TBL = f[120].mat()
        FSPB = f[230].vec()
        gmax = m[167].real()
        trcode = m[180].real()
        trcvel = m[182].real()
        SBEL = f[219].vec()
        VBEL = f[233].vec()
        # -----------------------------------------------------------
        SBTL = SBEL - STEL
        dbt = SBTL.absolute()
        dum = SBTL ^ (VBEL - VTEL)
        dcvel = abs(dum / dbt)

        if time > 3.0:
            if dcvel < trcvel:
                trcode = 1.0

        fspcb1 = FSPB.get_loc(0, 0)
        adely = fspcb1 * math.tan(psipb) / AGRAV
        adelz = fspcb1 * math.tan(thtpb) / (math.cos(psipb) * AGRAV)

        GRAVL = Matrix().build_vec3(0.0, 0.0, 1.0)
        GRAVB = TBL * GRAVL

        gn = gnav * dcvel
        apny = gn * sigdpz / (math.cos(psipb) * AGRAV)
        apnz = gn * (sigdpz * math.tan(thtpb) * math.tan(psipb) + sigdpy / math.cos(thtpb)) / AGRAV
        all_ = apny + adely - GRAVB.get_loc(1, 0)
        ann = apnz + adelz + GRAVB.get_loc(2, 0)

        aa = math.sqrt(all_ * all_ + ann * ann)
        if aa > gmax:
            aa = gmax
        if abs(ann) < SMALL and abs(all_) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ann, all_)
        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)
        # -----------------------------------------------------------
        m[402].gets(ancomx)
        m[403].gets(alcomx)
        m[404].gets(gn)
        m[406].gets(apny)
        m[407].gets(apnz)
        m[408].gets(adely)
        m[409].gets(adelz)
        m[410].gets(all_)
        m[411].gets(ann)
        m[180].gets(trcode)

    # ------------------------------------------------------------------ #
    # seeker.cpp  (Missile::def_seeker / seeker / seeker_kin / seeker_dyn /
    #              seeker_aimp / seeker_uthpb / seeker_thb)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_seeker(self) -> None:
        m = self.missile
        m[1].init("tgt_num", "int", 0, "Target tail # attacked by 'this' missile", "combus", "data", "")
        m[2].init("STEL", 0, 0, 0, "Position of target, downloaded from 'combus' - m", "combus", "out", "")
        m[3].init("VTEL", 0, 0, 0, "Velocity of target, downloaded from 'combus' - m", "combus", "out", "")
        m[4].init("TTL", 1, 0, 0, 0, 1, 0, 0, 0, 1, "TM of target, downloaded from 'combus' - m", "combus", "", "")
        m[5].init("tgt_com_slot", "int", 0, "This target slot in combus - ND", "combus", "out", "")
        m[6].init("sht_num", "int", 0, "Shooter tail # launching 'this' missile", "combus", "data", "")
        m[7].init("SSEL", 0, 0, 0, "Position of shooter, downloaded from 'combus' - m", "combus", "", "")
        m[8].init("dts", 0.0, "Target-shooter distance - m", "combus", "out", "scrn,plot")
        m[9].init("STSL", 0, 0, 0, "Velocity of shooter, downloaded from 'combus' - m", "combus", "out", "")
        m[200].init("mseek", "int", 0, "=2:Enable, =3:Acquisition, =4:Lock", "seeker", "data/diag", "com")
        m[201].init("ms1dyn", "int", 0, "=0: Kinemtic, =1:Dynamic", "seeker", "data", "")
        m[202].init("isets1", "int", 0, "Seeker flag", "seeker", "init", "")
        m[203].init("epchac", 0.0, "Epoch of start of seeker acquisition - s", "seeker", "save", "")
        m[204].init("ibreak", "int", 0, "Flag for seeker break-lock ND", "seeker", "init", "")
        m[206].init("dblind", 0.0, "Blind range - m", "seeker", "data", "")
        m[207].init("racq", 0.0, "Acquisition range - m", "seeker", "data", "")
        m[208].init("dtimac", 0.0, "Time duration to acquire target - s", "seeker", "data", "")
        m[209].init("dbt", 0.0, "true distance to target - m", "seeker", "diag", "scrn,plot")
        m[211].init("gk", 0.0, "K.F. gain - 1/s", "seeker", "data", "")
        m[212].init("zetak", 0.0, "K.F. damping", "seeker", "data", "")
        m[213].init("wnk", 0.0, "K.F. natural frequency - rad/s", "seeker", "data", "")
        m[215].init("biast", 0.0, "Pitch gimbal bias errors - rad", "seeker", "data", "")
        m[216].init("randt", 0.0, "Pitch gimbal random errors - rad", "seeker", "data", "")
        m[217].init("biasp", 0.0, "Roll gimbal bias error - rad", "seeker", "data", "")
        m[218].init("randp", 0.0, "Roll gimbal bias error - rad", "seeker", "data", "")
        m[219].init("wlq1d", 0.0, "Pitch sight line spin derivative - rad/s^2", "seeker", "state", "")
        m[220].init("wlq1", 0.0, "Pitch sight line spin - rad/s", "seeker", "state", "")
        m[221].init("wlqd", 0.0, "Pitch pointing rate derivative - rad/s^2", "seeker", "state", "")
        m[222].init("wlq", 0.0, "Pitch pointing rate - rad/s", "seeker", "state", "")
        m[223].init("wlr1d", 0.0, "Yaw sight line spin derivative - rad/s^2", "seeker", "state", "")
        m[224].init("wlr1", 0.0, "Yaw sight line spin - rad/s", "seeker", "state", "")
        m[225].init("wlrd", 0.0, "Yaw pointing rate derivative - rad/s^2", "seeker", "state", "")
        m[226].init("wlr", 0.0, "Yaw pointing rate - rad/s", "seeker", "state", "")
        m[227].init("wlq2d", 0.0, "Second state variable deriv in K.F. - rad/s^3", "seeker", "state", "")
        m[228].init("wlq2", 0.0, "Second state variable in K.F. - rad/s^2", "seeker", "state", "")
        m[229].init("wlr2d", 0.0, "Second state variable der in K.F. - rad/s^3", "seeker", "state", "")
        m[230].init("wlr2", 0.0, "Second state variable in K.F. - rad/s^2", "seeker", "state", "")
        m[231].init("fovyaw", 0.0, "Half yaw field-of-view at acquisition - rad", "seeker", "data", "")
        m[232].init("fovpitch", 0.0, "Half positive pitch field-of-view at acquis. - rad", "seeker", "data", "")
        m[234].init("dba", 0.0, "Distance between active seeker and its aimpoint - m", "seeker", "diag", "")
        m[235].init("daim", 0.0, "Dist from targ to initiate aimpoint mode - m", "seeker", "data", "")
        m[236].init("BIASAI", 1, 0.5, 0.2, "Bias error of aimpoint mode in target coor - m", "seeker", "data", "")
        m[239].init("BIASSC", 0, 0, 0, "Bias error of hot spot mode in target coor - m", "seeker", "data", "")
        m[242].init("RANDSC", 0, 0, 0, "Random error of hot spot mode in targ coor - m", "seeker", "data", "")
        m[245].init("epy", 0.0, "Error of pointing in pitch - rad", "seeker", "diag", "")
        m[246].init("epz", 0.0, "Error of pointing in yaw - rad", "seeker", "diag", "")
        m[247].init("thtpb", 0.0, "Pitch pointing angle - rad", "seeker", "out", "")
        m[248].init("psipb", 0.0, "Yaw pointing angle - rad", "seeker", "out", "")
        m[249].init("ththb", 0.0, "Head pitch angle - rad", "seeker", "diag", "")
        m[250].init("phihb", 0.0, "Head roll angle - rad", "seeker", "diag", "")
        m[251].init("ththbx", 0.0, "Head pitch angle - deg", "seeker", "diag", "scrn,plot")
        m[252].init("phihbx", 0.0, "Head roll angle - deg", "seeker", "diag", "scrn,plot")
        m[253].init("TPB", 0, 0, 0, 0, 0, 0, 0, 0, 0, "I/G TM of pointing axes wrt body axes", "seeker", "init", "")
        m[262].init("THB", 0, 0, 0, 0, 0, 0, 0, 0, 0, "I/G TM of head axes wrt body axes", "seeker", "init", "")
        m[271].init("timeac", 0.0, "Time duration of seeker acquisition phase - s", "seeker", "diag", "")
        m[272].init("psiot1", 0.0, "Azimuth of sensor LOS in target axes - rad", "seeker", "diag", "")
        m[273].init("thtot1", 0.0, "Elevation of sensor LOS in target axes - rad", "seeker", "diag", "")
        m[274].init("dvbtc", 0.0, "Closing velocity, computed - m/s", "seeker", "diag", "")
        m[276].init("EAHH", 0, 0, 0, "Aimpoint displacement wrt center of F.P. - rad", "seeker", "diag", "plot")
        m[279].init("EPHH", 0, 0, 0, "Computer pointing error of sensor wrt center of F.P.", "seeker", "diag", "")
        m[282].init("EAPH", 0, 0, 0, "Aimpoint to computer pointing displacement - rad", "seeker", "diag", "")
        m[285].init("thtpbx", 0.0, "Pitch pointing angle - deg", "seeker", "diag", "scrn,plot")
        m[286].init("psipbx", 0.0, "Yaw pointing angle - deg", "seeker", "diag", "scrn,plot")
        m[287].init("sigdpy", 0.0, "Pitch sight line spin - rad/s", "seeker", "out", "plot")
        m[288].init("sigdpz", 0.0, "Yaw sight line spin - rad/s", "seeker", "out", "plot")
        m[289].init("biaseh", 0.0, "Image blur and pixel bias errors - rad", "seeker", "data", "")
        m[290].init("randeh", 0.0, "Image blur and pixel random errors - rad", "seeker", "data", "")
        m[291].init("psihlx", 0.0, "Yaw angle of CL seeker wrt local axes - deg", "seeker", "diag", "")
        m[292].init("ththlx", 0.0, "Pitch angle of CL seeker wrt local axes - deg", "seeker", "diag", "")
        m[293].init("phihlx", 0.0, "Roll angle of CL seeker wrt local axes - deg", "seeker", "diag", "")
        m[294].init("SBTL", 0, 0, 0, "Position of missile wrt target - m", "seeker", "diag", "")

    def seeker(self, combus, num_vehicles: int, int_step: float) -> None:
        m, f = self.missile, self.flat6
        tgt_num = m[1].integer()
        sht_num = m[6].integer()
        mseek = m[200].integer()
        ms1dyn = m[201].integer()
        isets1 = m[202].integer()
        racq = m[207].real()
        dtimac = m[208].real()
        fovyaw = m[231].real()
        fovpitch = m[232].real()
        epchac = m[203].real()
        time = f[0].real()
        SBEL = f[219].vec()
        mguid = m[400].integer()
        THB = m[262].mat()
        # -----------------------------------------------------------
        tgt_com_slot = 0
        SSEL = Matrix(3, 1)
        STEL = Matrix(3, 1)
        VTEL = Matrix(3, 1)
        TTL = Matrix(3, 3).identity()  # shortcut, eventually comes from 'combus'

        shooter_id = f"t{sht_num}"
        for i in range(num_vehicles):
            if combus[i].get_id() == shooter_id:
                tgt_com_slot = i
                data_t = combus[i].get_data()
                SSEL = data_t[1].vec()

        target_id = f"t{tgt_num}"
        for i in range(num_vehicles):
            if combus[i].get_id() == target_id:
                tgt_com_slot = i
                data_t = combus[i].get_data()
                STEL = data_t[1].vec()
                VTEL = data_t[2].vec()

        STSL = STEL - SSEL
        dts = STSL.absolute()

        SBTL = SBEL - STEL
        SBTT = TTL * SBTL
        POLAR = SBTT.pol_from_cart()
        dbt = POLAR.get_loc(0, 0)
        psiot1 = POLAR.get_loc(1, 0)
        thtot1 = POLAR.get_loc(2, 0)

        thtpb = 0.0
        psipb = 0.0
        sigdpy = 0.0
        sigdpz = 0.0
        timeac = 0.0

        if mseek == 2:
            isets1 = 1
            if dbt < racq:
                mseek = 3

        if mseek == 3:
            if isets1 == 1:
                thtpb, psipb, sigdy, sigdz = self.seeker_kin(SBTL, VTEL, dbt)
                ththb, phihb = self.seeker_uthpb(psipb, thtpb)
                THB = self.seeker_thb(ththb, phihb)
                isets1 = 0
                epchac = time
            if ms1dyn == 1:
                (mseek, mguid, thtpb, psipb, sigdy, sigdz, ehz, ehy,
                 THB) = self.seeker_dyn(mseek, mguid, THB, SBTL, dbt, int_step)
                if abs(ehz) <= fovyaw and abs(ehy) <= fovpitch:
                    timeac = time - epchac
                    if timeac > dtimac:
                        mseek = 4
            else:
                thtpb, psipb, sigdy, sigdz = self.seeker_kin(SBTL, VTEL, dbt)
                timeac = time - epchac
                if timeac > dtimac:
                    mseek = 4

        if mseek == 4:
            mguid = 6
            if ms1dyn == 1:
                (mseek, mguid, thtpb, psipb, sigdy, sigdz, ehz, ehy,
                 THB) = self.seeker_dyn(mseek, mguid, THB, SBTL, dbt, int_step)
            else:
                thtpb, psipb, sigdy, sigdz = self.seeker_kin(SBTL, VTEL, dbt)

            sigdpy = sigdy
            sigdpz = sigdz

        thtpbx = thtpb * DEG
        psipbx = psipb * DEG
        # -----------------------------------------------------------
        m[2].gets_vec(STEL)
        m[3].gets_vec(VTEL)
        m[5].gets(tgt_com_slot)
        m[7].gets_vec(SSEL)
        m[8].gets(dts)
        m[9].gets_vec(STSL)
        m[247].gets(thtpb)
        m[248].gets(psipb)
        m[287].gets(sigdpy)
        m[288].gets(sigdpz)
        m[400].gets(mguid)
        m[262].gets_mat(THB)
        m[203].gets(epchac)
        m[200].gets(mseek)
        m[202].gets(isets1)
        m[209].gets(dbt)
        m[271].gets(timeac)
        m[272].gets(psiot1)
        m[273].gets(thtot1)
        m[285].gets(thtpbx)
        m[286].gets(psipbx)
        m[294].gets_vec(SBTL)

    def seeker_kin(self, SBTL: Matrix, VTEL: Matrix, dbt: float):
        f = self.flat6
        TBL = f[120].mat()
        VBEL = f[233].vec()
        # -----------------------------------------------------------
        STBL = SBTL * (-1.0)
        STBB = TBL * STBL
        dum1 = 1.0 / dbt
        UTBL = STBL * dum1

        VTBL = VTEL - VBEL

        dvbtc = abs(UTBL ^ VTBL)

        WOEB = TBL * UTBL.skew_sym() * VTBL * dum1

        POLAR = STBB.pol_from_cart()
        psipb = POLAR.get_loc(1, 0)
        thtpb = POLAR.get_loc(2, 0)
        TPB = mat2tr(psipb, thtpb)

        WOEP = TPB * WOEB
        sigdy = WOEP.get_loc(1, 0)
        sigdz = WOEP.get_loc(2, 0)
        # -----------------------------------------------------------
        self.missile[274].gets(dvbtc)
        return thtpb, psipb, sigdy, sigdz

    def seeker_dyn(self, mseek: int, mguid: int, THB: Matrix, SBTL: Matrix,
                    dbt: float, int_step: float):
        m, f = self.missile, self.flat6
        ibreak = m[204].integer()
        dblind = m[206].real()
        gk = m[211].real()
        zetak = m[212].real()
        wnk = m[213].real()
        biast = m[215].real()
        randt = m[216].real()
        biasp = m[217].real()
        randp = m[218].real()
        biaseh = m[289].real()
        randeh = m[290].real()
        TPB = m[253].mat()
        TTL = m[4].mat()
        TBL = f[120].mat()
        WBEB = f[163].vec()
        trcode = m[180].real()
        trtht = m[187].real()
        trthtd = m[189].real()
        trphid = m[190].real()
        trate = m[191].real()
        wlq1d = m[219].real()
        wlq1 = m[220].real()
        wlqd = m[221].real()
        wlq = m[222].real()
        wlr1d = m[223].real()
        wlr1 = m[224].real()
        wlrd = m[225].real()
        wlr = m[226].real()
        wlq2d = m[227].real()
        wlq2 = m[228].real()
        wlr2d = m[229].real()
        wlr2 = m[230].real()
        # -----------------------------------------------------------
        THL = THB * TBL
        SBTH = THL * SBTL
        SATH = self.seeker_aimp(THL, TTL, dbt)
        SABH = SATH - SBTH

        sabh1 = SABH.get_loc(0, 0)
        sabh2 = SABH.get_loc(1, 0)
        sabh3 = SABH.get_loc(2, 0)
        ey = math.atan2(-sabh3, sabh1)
        ez = math.atan2(sabh2, sabh1)
        ehy = ey + biaseh + randeh
        ehz = ez + biaseh + randeh
        EAHH = Matrix().build_vec3(0.0, ehz, -ehy)

        TBH = THB.trans()  # THB of previous integration cycle is used
        TPH = TPB * TBH
        THP = TPH.trans()
        U1PP = Matrix().build_vec3(1.0, 0.0, 0.0)
        U1HH = Matrix().build_vec3(1.0, 0.0, 0.0)
        EPHH = THP * U1PP - U1HH
        EAPH = EAHH - EPHH
        EAPP = TPH * EAPH
        epy = -EAPP.get_loc(2, 0)
        epz = EAPP.get_loc(1, 0)

        wsq = wnk * wnk
        gg = gk * wsq
        wlr1d_new = wlr2
        wlr1 = integrate(wlr1d_new, wlr1d, wlr1, int_step)
        wlr1d = wlr1d_new
        wlr2d_new = gg * epz - 2.0 * zetak * wnk * wlr1d - wsq * wlr1
        wlr2 = integrate(wlr2d_new, wlr2d, wlr2, int_step)
        wlr2d = wlr2d_new
        wlq1d_new = wlq2
        wlq1 = integrate(wlq1d_new, wlq1d, wlq1, int_step)
        wlq1d = wlq1d_new
        wlq2d_new = gg * epy - 2.0 * zetak * wnk * wlq1d - wsq * wlq1
        wlq2 = integrate(wlq2d_new, wlq2d, wlq2, int_step)
        wlq2d = wlq2d_new

        sigdz = wlr1
        sigdy = wlq1

        WBEP = TPB * WBEB
        wbep2 = WBEP.get_loc(1, 0)
        wbep3 = WBEP.get_loc(2, 0)
        wlrd_new = wlr1 - wbep3
        wlr = integrate(wlrd_new, wlrd, wlr, int_step)
        wlrd = wlrd_new
        psipb = wlr
        psipbd = wlrd
        wlqd_new = wlq1 - wbep2
        wlq = integrate(wlqd_new, wlqd, wlq, int_step)
        wlqd = wlqd_new
        thtpb = wlq
        thtpbd = wlqd
        TPB = mat2tr(psipb, thtpb)

        ththbc, phihbc = self.seeker_uthpb(psipb, thtpb)
        ththb = ththbc + biast + randt
        phihb = phihbc + biasp + randp
        THB = self.seeker_thb(ththb, phihb)

        ththbx = ththb * DEG
        phihbx = phihb * DEG

        if mseek == 4:
            ibreak = 0
            phihbd = -thtpbd * math.sin(psipb)
            eh = math.sqrt(ehy * ehy + ehz * ehz)
            if abs(ththb) > trtht:
                trcode = 6.0
                ibreak = 1
            elif abs(thtpbd) > trthtd:
                trcode = 7.0
                ibreak = 1
            elif abs(phihbd) > trphid:
                trcode = 8.0
                ibreak = 1
            elif eh > trate:
                trcode = 9.0
                ibreak = 1
            if ibreak == 1:
                mseek = 2
                mguid = 3
            if dbt < dblind:
                mseek = 5
        # -----------------------------------------------------------
        m[219].gets(wlq1d)
        m[220].gets(wlq1)
        m[221].gets(wlqd)
        m[222].gets(wlq)
        m[223].gets(wlr1d)
        m[224].gets(wlr1)
        m[225].gets(wlrd)
        m[226].gets(wlr)
        m[227].gets(wlq2d)
        m[228].gets(wlq2)
        m[229].gets(wlr2d)
        m[230].gets(wlr2)
        m[245].gets(epy)
        m[246].gets(epz)
        m[249].gets(ththb)
        m[250].gets(phihb)
        m[251].gets(ththbx)
        m[252].gets(phihbx)
        m[253].gets_mat(TPB)
        m[276].gets_vec(EAHH)
        m[279].gets_vec(EPHH)
        m[282].gets_vec(EAPH)
        m[180].gets(trcode)
        return mseek, mguid, thtpb, psipb, sigdy, sigdz, ehz, ehy, THB

    def seeker_aimp(self, THL: Matrix, TTL: Matrix, dbt: float) -> Matrix:
        m = self.missile
        daim = m[235].real()
        BIASAI = m[236].vec()
        BIASSC = m[239].vec()
        RANDSC = m[242].vec()
        # -----------------------------------------------------------
        THT = THL * TTL.trans()
        if dbt < daim:
            SATH = THT * BIASAI
        else:
            SATH = THT * (BIASSC + RANDSC)
        return SATH

    def seeker_uthpb(self, psipb: float, thtpb: float):
        ththb = math.acos(math.cos(thtpb) * math.cos(psipb))
        sinpsi = math.sin(psipb)
        tantht = math.tan(thtpb)
        if abs(sinpsi) and abs(tantht) < SMALL:
            phihb = 0.0
        else:
            phihb = math.atan2(sinpsi, tantht)
        return ththb, phihb

    def seeker_thb(self, tht: float, phi: float) -> Matrix:
        THB = Matrix(3, 3)
        THB.assign_loc(0, 0, math.cos(tht))
        THB.assign_loc(2, 0, math.sin(tht))
        THB.assign_loc(1, 1, math.cos(phi))
        THB.assign_loc(1, 2, math.sin(phi))

        THB.assign_loc(0, 1, THB.get_loc(2, 0) * THB.get_loc(1, 2))
        THB.assign_loc(0, 2, (-THB.get_loc(2, 0)) * THB.get_loc(1, 1))
        THB.assign_loc(2, 1, (-THB.get_loc(0, 0)) * THB.get_loc(1, 2))
        THB.assign_loc(2, 2, THB.get_loc(0, 0) * THB.get_loc(1, 1))
        THB.assign_loc(1, 0, 0.0)

        return THB

    # ------------------------------------------------------------------ #
    # intercept.cpp  (Missile::def_intercept / intercept)   -- COMPLETE
    # ------------------------------------------------------------------ #
    def def_intercept(self) -> None:
        m = self.missile
        m[651].init("write", "int", 1, "True flag for writing miss to console - ND", "intercept", "init", "")
        m[652].init("miss", 0.0, "Miss distance - m", "intercept", "diag", "plot")
        m[653].init("hit_time", 0.0, "Intercept time - s", "intercept", "diag", "")
        m[654].init("MISS_L", 0, 0, 0, "Miss vector in local level coord. - m", "intercept", "diag", "")
        m[655].init("time_m", 0.0, "Previous time - s", "intercept", "save", "")
        m[656].init("SBTLM", 0, 0, 0, "Previous displacment vector. - m", "intercept", "save", "")
        m[657].init("STMEL", 0, 0, 0, "Previous aircraft taget displacment vector. - m", "intercept", "save", "")
        m[658].init("SBMEL", 0, 0, 0, "Previous missile displacment vector. - m", "intercept", "save", "")
        m[659].init("mode", "int", 0, "Mode flags |mseek|mguid|maut|mprop|  - ND", "intercept", "diag", "scrn")

    def intercept(self, combus, vehicle_slot: int, int_step: float, title: str) -> None:
        m, f = self.missile, self.flat6
        time = f[0].real()
        halt = f[1].integer()
        TBL = f[120].mat()
        SBEL = f[219].vec()
        VBEL = f[233].vec()
        dvbe = f[236].real()
        psivlx = f[240].real()
        thtvlx = f[241].real()
        STEL = m[2].vec()
        VTEL = m[3].vec()
        tgt_com_slot = m[5].integer()
        mprop = m[50].integer()
        mseek = m[200].integer()
        mguid = m[400].integer()
        maut = m[500].integer()
        write = m[651].integer()
        time_m = m[655].real()
        SBTLM = m[656].vec()
        STMEL = m[657].vec()
        SBMEL = m[658].vec()
        # -----------------------------------------------------------
        hit_time = 0.0
        MISS_L = Matrix(3, 1)
        miss = 0.0

        mode = 1000 * mseek + 100 * mguid + 10 * maut + mprop

        STBL = STEL - SBEL
        TBL * STBL  # STBB -- computed in the original but unused beyond this
        dbt = STBL.absolute()

        if halt:
            id_missl = combus[vehicle_slot].get_id()
            print(f"\n *** Halt of Missile_{id_missl}   Time = {time} sec ***")
            print(f"      speed = {dvbe} m/s  heading = {psivlx} deg      gamma = {thtvlx} deg\n")
            combus[vehicle_slot].set_status(0)

        alt = -SBEL.get_loc(2, 0)
        if alt <= 0 and write:
            write = 0
            id_missl = combus[vehicle_slot].get_id()
            print(f"\n *** Ground impact of Missile_{id_missl} (or: #IND/#QNAN)  Time = {time} sec ***")
            print(f"      speed = {dvbe} m/s  heading = {psivlx} deg      gamma = {thtvlx} deg\n")
            combus[vehicle_slot].set_status(0)

        if mseek >= 3:
            if dbt < 100:
                UTBL = STBL * (1.0 / dbt)
                VTBL = VTEL - VBEL
                closing_speed = UTBL ^ VTBL

                SBTL = STBL * (-1.0)
                if closing_speed > 0 and write:
                    write = 0

                    SBBML = SBEL - SBMEL
                    STTML = STEL - STMEL

                    hit_time = time_m - int_step * ((SBBML - STTML) ^ SBTLM) / (SBBML ^ SBBML)

                    tau = hit_time - time_m
                    MISS_L = (SBBML - STTML) * (tau / int_step) + SBTLM
                    miss = MISS_L.absolute()

                    id_targ = combus[tgt_com_slot].get_id()
                    id_missl = combus[vehicle_slot].get_id()

                    print(f"\n *** Intercept of Missile_{id_missl} and Target_{id_targ} ***")
                    print(f"      miss distance = {miss} m    intercept time = {hit_time} sec")
                    print(f"      north = {MISS_L.get_loc(0, 0)} m      east = {MISS_L.get_loc(1, 0)}"
                          f" m        down = {MISS_L.get_loc(2, 0)} m")
                    print(f"      speed = {dvbe} m/s  heading = {psivlx} deg       gamma = {thtvlx} deg\n")

                    combus[tgt_com_slot].set_status(0)
                    combus[vehicle_slot].set_status(0)

                SBTLM = SBTL
                STMEL = STEL
                SBMEL = SBEL
                time_m = time
        # -----------------------------------------------------------
        m[651].gets(write)
        m[653].gets(hit_time)
        m[655].gets(time_m)
        m[656].gets_vec(SBTLM)
        m[657].gets_vec(STMEL)
        m[658].gets_vec(SBMEL)
        m[652].gets(miss)
        m[654].gets_vec(MISS_L)
        m[659].gets(mode)

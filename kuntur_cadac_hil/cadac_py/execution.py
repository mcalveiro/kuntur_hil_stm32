"""
Port of 'execution.cpp' + the 'input.asc'-parsing parts of
'global_functions.cpp' / 'missile_functions.cpp' / 'target_functions.cpp'.
 
This is the generic driver that ties every previously-ported module
together: it parses an 'input.asc' scenario file (TITLE/OPTIONS/MODULES/
TIMING/VEHICLES/events/ENDTIME), builds the vehicle objects it names
(Missile/Target), applies their initial-condition data, and runs the
same generic module-dispatch loop as 'execute()' in execution.cpp --
so any 'input.asc' in the CADAC/SRAAM6 tree can be run directly, not
just the hand-coded demos.
 
Fidelity notes / deliberate simplifications:
- The 'combus' packet layout follows the exact convention the original
  com_index_arrays()/loading_packet() produce: for each vehicle, the
  packet's data list is [flat6/flat3 variables tagged 'com', in index
  order] + [missile/target variables tagged 'com', in index order].
  This is *derived* here (by scanning for "com" in each Variable's
  get_out()), not hand-listed, so it stays correct if module-variable
  tags change. It reproduces the fixed positions seeker.cpp's RTI note
  documents (target position @1, velocity @2) because SAEL/VAEL happen
  to be the 2nd/3rd 'com'-tagged flat3[] variables by index -- see
  target.py's module docstring.
- Vehicle ids follow the original convention: missiles "m1",
  "m2",...,targets "t1","t2",... (see missile_functions.cpp /
  target_functions.cpp loading_packet_init()).
- Health/status bookkeeping (combus_status() + explicit set_status()
  after loading_packet()) is reproduced exactly: a vehicle's own combus
  slot is rebuilt with fresh data each step, then its status is
  re-applied from a snapshot taken right after that vehicle's module
  loop -- so a set_status(0) from e.g. intercept() during this step is
  preserved instead of being clobbered by the rebuild.
- Event (IF ... ENDIF) blocks are supported: a watched module-variable
  compared against a threshold ('<','=','>'), applying a list of
  variable=value assignments once when the condition first fires --
  same semantics as Missile::event()/missile_functions.cpp, but
  resolved to direct Variable references at parse time instead of
  through the index-array indirection the original uses (Event class
  in core.py is kept for structural fidelity but this module does its
  own lighter-weight resolution).
- NOT ported (deliberately out of scope, pure I/O/reporting, no
  physics): scrn_data()/scrn_banner() (console dump), tabout.asc,
  doc.asc / document_input() (module-variable documentation generator),
  Monte Carlo run merging.
  Trajectory output here is a plain CSV (one row per vehicle-combus
  snapshot at 'traj_step'), which is the Python-native equivalent the
  project's README already commits to ("directly to pandas/CSV").
  'ploti.asc'/'plot.asc' (CADAC Studio/KPlot plot files) WAS also out
  of scope (Etapa 1) but is now covered by cadac_py/plotfile.py
  (Etapa 2), which hooks into run_scenario() below through the
  optional 'plot_step_override'/'on_plot_sample' parameters added for
  that purpose. Both default to None/unused, so every pre-existing
  call to run_scenario() is completely unaffected -- this file's
  validated Etapa-1 physics are otherwise untouched.
- Multiple missiles/targets are supported by the parser and module
  dispatch loop (matches the original's generic num_vehicles design),
  though it has only been exercised end-to-end against SRAAM6's
  1-missile-vs-1-target 'input.asc' scenarios.
"""
from __future__ import annotations
 
import csv
import os
import string
from dataclasses import dataclass, field
from typing import Optional
 
from .core import Module, Packet, Variable
from .missile import Missile
from .target import Target
from .falcon6 import Plane
from .tables import read_deck
 
_PUNCT = set(string.punctuation)
 
class _Scanner:
    def __init__(self, text: str):
        self.text = text
        self.pos = 0
        self.n = len(text)
 
    def token(self) -> Optional[str]:
        while self.pos < self.n and self.text[self.pos].isspace():
            self.pos += 1
        if self.pos >= self.n:
            return None
        start = self.pos
        while self.pos < self.n and not self.text[self.pos].isspace():
            self.pos += 1
        return self.text[start:self.pos]
 
    def read_char(self) -> Optional[str]:
        while self.pos < self.n and self.text[self.pos].isspace():
            self.pos += 1
        if self.pos >= self.n:
            return None
        ch = self.text[self.pos]
        self.pos += 1
        return ch
 
    def getline(self) -> str:
        idx = self.text.find("\n", self.pos)
        if idx == -1:
            line, self.pos = self.text[self.pos:], self.n
        else:
            line, self.pos = self.text[self.pos:idx], idx + 1
        return line
 
    def eof(self) -> bool:
        return self.pos >= self.n
 
@dataclass
class VehicleSpec:
    vtype: str            # "MISSILE6" or "TARGET3"
    name: str             # free-text name line (e.g. "Missile AIM")
    assignments: list = field(default_factory=list)   # [(var_name, value_or_filename)]
    events: list = field(default_factory=list)         # [{"watch","op","value","assignments"}]
 
@dataclass
class ParsedInput:
    title: str
    options: str
    modules: list          # List[Module]
    timing: dict
    vehicles: list         # List[VehicleSpec]
    end_time: float
 
def parse_input_asc(path: str) -> ParsedInput:
    text = open(path, "r").read()
    sc = _Scanner(text)
 
    # ---- acquire_title_options -----------------------------------
    title = None
    n = 0
    tok = sc.token()
    while tok != "OPTIONS" and n < 50 and tok is not None:
        n += 1
        if tok and tok[0] in _PUNCT:
            sc.getline()
        elif tok == "TITLE":
            title = sc.getline().strip()
        tok = sc.token()
    options = sc.getline().strip()
    if title is None:
        title = "*** No Title found ***"
 
    # ---- number_modules + order_modules ---------------------------
    tok = sc.token()
    if tok != "MODULES":
        raise ValueError("'MODULES' must follow 'OPTIONS'")
    sc.getline()
    modules = []
    while True:
        tok = sc.token()
        if tok is None or tok == "END":
            break
        flags = sc.getline()
        modules.append(Module(
            name=tok,
            definition="def" if "def" in flags else "0",
            initialization="init" if "init" in flags else "0",
            execution="exec" if "exec" in flags else "0",
            termination="term" if "term" in flags else "0",
        ))
 
    # ---- acquire_timing ---------------------------------------------
    tok = sc.token()
    if tok != "TIMING":
        raise ValueError("'TIMING' must follow 'MODULES'")
    sc.getline()
    timing = {"plot_step": 0.0, "scrn_step": 0.0, "int_step": 0.0,
              "com_step": 0.0, "traj_step": 0.0}
    while True:
        tok = sc.token()
        if tok is None or tok == "END":
            break
        if tok in timing:
            timing[tok] = float(sc.token())
        sc.getline()
 
    # ---- number_objects + per-vehicle vehicle_data -------------------
    tok = sc.token()
    if tok != "VEHICLES":
        raise ValueError("'VEHICLES' must follow 'TIMING'")
    num_vehicles = int(sc.token())
    sc.getline()
 
    vehicles = []
    for _ in range(num_vehicles):
        tok = sc.token()
        while tok is not None and tok[0] in _PUNCT:
            sc.getline()
            tok = sc.token()
        vtype = tok
        vname = sc.getline().strip()
        spec = VehicleSpec(vtype=vtype, name=vname)
        current_event = None
        while True:
            tok = sc.token()
            if tok is None:
                raise ValueError(f"unexpected EOF inside vehicle block {vtype!r}")
            if tok[0] in _PUNCT:
                sc.getline()
                continue
            if tok == "END":
                sc.getline()
                break
            if tok in ("AERO_DECK", "PROP_DECK"):
                fname = sc.token()
                sc.getline()
                spec.assignments.append((tok, fname))
                continue
            if tok == "IF":
                watch = sc.token()
                op = sc.read_char()
                value = float(sc.token())
                sc.getline()
                current_event = {"watch": watch, "op": op, "value": value, "assignments": []}
                continue
            if tok == "ENDIF":
                sc.getline()
                if current_event is not None:
                    spec.events.append(current_event)
                current_event = None
                continue
            # ordinary "name  value  //comment" line
            val_tok = sc.token()
            sc.getline()
            value = float(val_tok)
            if current_event is not None:
                current_event["assignments"].append((tok, value))
            else:
                spec.assignments.append((tok, value))
        vehicles.append(spec)
 
    # ---- acquire_endtime ----------------------------------------------
    end_time = 0.0
    while True:
        tok = sc.token()
        if tok is None:
            break
        if tok == "ENDTIME":
            end_time = float(sc.token())
            break
        sc.getline()
 
    return ParsedInput(title=title, options=options, modules=modules,
                        timing=timing, vehicles=vehicles, end_time=end_time)
 
def build_vehicle(vtype: str, modules: list):
    if vtype == "MISSILE6":
        v = Missile()
    elif vtype == "TARGET3":
        v = Target()
    elif vtype == "PLANE6":
        v = Plane()
    else:
        raise ValueError(f"unknown vehicle type {vtype!r} (expected MISSILE6, TARGET3 or PLANE6)")
 
    for m in modules:
        if m.definition == "def":
            fn = getattr(v, f"def_{m.name}", None)
            if fn is not None:
                fn()
    return v
 
def _module_arrays(vehicle):
    if isinstance(vehicle, Missile):
        return [vehicle.flat6, vehicle.missile]
    if isinstance(vehicle, Target):
        return [vehicle.flat3, vehicle.target]
    if isinstance(vehicle, Plane):
        return [vehicle.flat6, vehicle.plane]
    raise TypeError(type(vehicle))
 
def _resolve_var(vehicle, name: str) -> Optional[Variable]:
    for arr in _module_arrays(vehicle):
        for var in arr:
            if var.get_name() == name:
                return var
    return None
 
def apply_vehicle_data(vehicle, assignments, base_dir: str) -> None:
    for name, value in assignments:
        if name == "AERO_DECK":
            vehicle.aerotable = read_deck(os.path.join(base_dir, value))
            continue
        if name == "PROP_DECK":
            vehicle.proptable = read_deck(os.path.join(base_dir, value))
            continue
        var = _resolve_var(vehicle, name)
        if var is None:
            print(f" *** Warning: variable '{name}' not found (check spelling) ***")
            continue
        if var.get_type() == "int":
            var.gets(int(value))
        else:
            var.gets(float(value))
 
def build_events(vehicle, raw_events):
    events = []
    for ev in raw_events:
        watch_var = _resolve_var(vehicle, ev["watch"])
        assigns = [(_resolve_var(vehicle, n), v) for n, v in ev["assignments"]]
        events.append({"watch": watch_var, "op": ev["op"], "value": ev["value"],
                        "assignments": assigns, "fired": False})
    return events
 
def check_events(events) -> None:
    """Missile::event()/Plane::event(): a vehicle's events form a QUEUE
    (the original's 'nevent' index into 'event_ptr_list') -- only the
    first not-yet-fired event is ever evaluated on a given call; later
    events in the queue are not even looked at until every earlier one
    has fired. This matters whenever several 'IF..ENDIF' blocks watch
    the SAME variable/condition in sequence (e.g. FALCON6's waypoint
    list, all watching 'wp_flag = -1'): checking every event every tick
    (as an earlier version of this function did) would fire all of them
    on the same tick the condition first becomes true, instead of one
    waypoint at a time."""
    for ev in events:
        if ev["fired"]:
            continue
        var = ev["watch"]
        if var is None:
            return
        if var.get_type() == "int":
            cur, crit = var.integer(), int(ev["value"])
        else:
            cur, crit = var.real(), ev["value"]
        op = ev["op"]
        fired = (op == "<" and cur < crit) or (op == "=" and cur == crit) or (op == ">" and cur > crit)
        if fired:
            for avar, aval in ev["assignments"]:
                if avar is None:
                    continue
                avar.gets(int(aval) if avar.get_type() == "int" else float(aval))
            ev["fired"] = True
        return
 
def make_packet(vehicle, vehicle_id: str) -> Packet:
    arr1, arr2 = _module_arrays(vehicle)
    data = [v for v in arr1 if "com" in v.get_out()] + [v for v in arr2 if "com" in v.get_out()]
    pkt = Packet()
    pkt.set_id(vehicle_id)
    pkt.set_status(1)
    pkt.set_data(data)
    pkt.set_ndata(len(data))
    return pkt
 
def _combus_row(sim_time: float, combus: list) -> dict:
    """CSV-native equivalent of global_functions.cpp's traj_data(): one
    flattened row per output time, all vehicles' 'com' variables side by
    side, vectors expanded into _1/_2/_3 components (same 'uppercase name
    = vector' convention the original uses)."""
    row = {"time": sim_time}
    for pkt in combus:
        vid = pkt.get_id()
        for var in pkt.get_data():
            name = var.get_name()
            if name == "time":
                continue
            col = f"{name}_{vid}"
            if var.get_type() == "int":
                row[col] = var.integer()
            elif name[:1].isupper():
                vec = var.vec()
                row[f"{name}1_{vid}"] = vec[0]
                row[f"{name}2_{vid}"] = vec[1]
                row[f"{name}3_{vid}"] = vec[2]
            else:
                row[col] = var.real()
    return row
 
_EXEC_DISPATCH = {
    # 'environment', 'propulsion' and 'guidance' have a different
    # signature on 'Plane' (FALCON6) than on 'Missile'/'Target'
    # (SRAAM6): FALCON6's environment()/propulsion() need 'int_step'
    # (its wind filter / engine spool-up lag integrate), and its
    # guidance() is single-vehicle (no combus lookup) -- see
    # falcon6.py's module docstring. Dispatch on vehicle type below
    # rather than assuming one signature per module name.
    "environment": lambda v, t, dt, combus, idx, title: (
        v.environment(dt) if isinstance(v, Plane) else v.environment()),
    "kinematics": lambda v, t, dt, combus, idx, title: v.kinematics(dt),
    "newton": lambda v, t, dt, combus, idx, title: v.newton(t, dt),
    "euler": lambda v, t, dt, combus, idx, title: v.euler(dt),
    "aerodynamics": lambda v, t, dt, combus, idx, title: v.aerodynamics(),
    "propulsion": lambda v, t, dt, combus, idx, title: (
        v.propulsion(dt) if isinstance(v, Plane) else v.propulsion()),
    "forces": lambda v, t, dt, combus, idx, title: v.forces(),
    "actuator": lambda v, t, dt, combus, idx, title: v.actuator(dt),
    "tvc": lambda v, t, dt, combus, idx, title: v.tvc(dt),
    "control": lambda v, t, dt, combus, idx, title: v.control(dt),
    "guidance": lambda v, t, dt, combus, idx, title: (
        v.guidance() if isinstance(v, Plane) else v.guidance(combus, len(combus))),
    "seeker": lambda v, t, dt, combus, idx, title: v.seeker(combus, len(combus), dt),
    "intercept": lambda v, t, dt, combus, idx, title: v.intercept(combus, idx, dt, title),
}
 
def run_scenario(input_path: str, out_csv: Optional[str] = None,
                  end_time_override: Optional[float] = None,
                  int_step_override: Optional[float] = None,
                  verbose: bool = True,
                  plot_step_override: Optional[float] = None,
                  on_plot_sample=None):
    """Parses and runs an 'input.asc' scenario end-to-end (execution.cpp's
    main()+execute(), minus the console/doc.asc I/O -- see module
    docstring). Returns (parsed, vehicles, combus, rows) where 'rows' is
    the list of dict rows written to 'out_csv' (traj.asc equivalent).
 
    'plot_step_override'/'on_plot_sample' are Etapa-2 hooks used ONLY by
    cadac_py/plotfile.py's run_scenario_with_plot(); when 'on_plot_sample'
    is None (the default, and the value used by every pre-existing call
    site) the two extra branches below are skipped entirely and this
    function's Etapa-1 behavior/output is byte-for-byte unchanged.
    When set, 'on_plot_sample(vehicle_index, vehicle, sim_time, merge)'
    is invoked at the exact sampling cadence execution.cpp's execute()
    uses for 'ploti.asc' (cadence 'plot_step', first sample at
    sim_time==0, one extra final call per vehicle with merge=True after
    the integration loop ends -- see plotfile.py for what 'merge' does).
    """
    base_dir = os.path.dirname(os.path.abspath(input_path))
    parsed = parse_input_asc(input_path)
    int_step = int_step_override if int_step_override is not None else parsed.timing["int_step"]
    traj_step = parsed.timing["traj_step"] or int_step
    plot_step = plot_step_override if plot_step_override is not None else (parsed.timing["plot_step"] or int_step)
    end_time = end_time_override if end_time_override is not None else parsed.end_time
 
    vehicles, combus, events_per_vehicle = [], [], []
    m_count = t_count = p_count = 0
    for spec in parsed.vehicles:
        v = build_vehicle(spec.vtype, parsed.modules)
        apply_vehicle_data(v, spec.assignments, base_dir)
        for m in parsed.modules:
            if m.initialization == "init":
                fn = getattr(v, f"init_{m.name}", None)
                if fn is not None:
                    fn()
        vehicles.append(v)
        events_per_vehicle.append(build_events(v, spec.events))
        if spec.vtype == "MISSILE6":
            m_count += 1
            vid = f"m{m_count}"
        elif spec.vtype == "TARGET3":
            t_count += 1
            vid = f"t{t_count}"
        else:
            p_count += 1
            vid = f"p{p_count}"
        combus.append(make_packet(v, vid))
 
    exec_modules = [m for m in parsed.modules if m.execution == "exec"]
 
    rows = [_combus_row(0.0, combus)]
    sim_time = 0.0
    next_traj = traj_step
    next_plot = 0.0
    increment_plot_time = False
    while sim_time <= end_time + int_step / 2:
        for i, v in enumerate(vehicles):
            check_events(events_per_vehicle[i])
            if combus[i].get_status() == 1:
                for m in exec_modules:
                    fn = _EXEC_DISPATCH.get(m.name)
                    if fn is not None:
                        fn(v, sim_time, int_step, combus, i, parsed.title)
                status_snapshot = [pkt.get_status() for pkt in combus]
                combus[i] = make_packet(v, combus[i].get_id())
                combus[i].set_status(status_snapshot[i])
 
            # Etapa-2 hook (execution.cpp's plot-sampling block, cadence
            # 'plot_step' -- independent of 'traj_step'/'com'). No-op
            # unless a caller (only plotfile.py does) passes
            # 'on_plot_sample'; see the docstring above.
            if on_plot_sample is not None and abs(next_plot - sim_time) < int_step / 2 + 1e-9:
                on_plot_sample(i, v, sim_time, False)
                if i == len(vehicles) - 1:
                    increment_plot_time = True
            if increment_plot_time:
                next_plot += plot_step
                increment_plot_time = False
 
        sim_time += int_step
        if sim_time >= next_traj - int_step / 2:
            rows.append(_combus_row(sim_time, combus))
            next_traj += traj_step
 
        if all(pkt.get_status() != 1 for pkt in combus):
            if verbose:
                print(f" *** All vehicles halted/dead at t={sim_time:.3f}s -- stopping run ***")
            break
 
    if on_plot_sample is not None:
        # execution.cpp: "writing last integration out to 'ploti.asc'
        # with time set to '-1' for multiple CADAC-Studio plots" --
        # unconditional whenever plotting is on, regardless of 'y_merge'.
        for i, v in enumerate(vehicles):
            on_plot_sample(i, v, sim_time, True)
 
    if out_csv:
        with open(out_csv, "w", newline="") as fh:
            fieldnames = sorted({k for r in rows for k in r.keys()}, key=lambda k: (k != "time", k))
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        if verbose:
            print(f"Wrote {len(rows)} rows to {out_csv}")
 
    return parsed, vehicles, combus, rows
 
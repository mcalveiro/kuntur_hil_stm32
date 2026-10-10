"""
Port of the 'ploti.asc'/'plot.asc' writer (CADAC Studio / KPlot plot
files) -- Etapa 2 of the project, previously out of scope (see
manual-usuario-cadac-py.md, section 8).
 
Origin (C++, byte-verified against the source, not inferred):
    missile_functions.cpp   Missile::plot_array/plot_banner/
                             plot_index_arrays/plot_data
    plane_functions.cpp     Plane::plot_array/plot_banner/
                             plot_index_arrays/plot_data   (FALCON6 --
                             identical logic to Missile's, only the
                             array/name/count symbols differ)
    class_hierarchy.hpp     Target::plot_banner/plot_data are inline
                             empty stubs ('{}') -- TARGET3 vehicles
                             never write to 'ploti.asc' in the
                             original. execution.cpp only opens a plot
                             file for MISSILE6/PLANE6 vehicles
                             ("currently only the MISSILE6 plot files
                             contain any data").
    execution.cpp            execute()'s plot-sampling block (cadence
                             'plot_step', NOT 'traj_step') and the
                             post-loop "writing last integration out
                             to 'ploti.asc' with time set to '-1'"
                             sentinel row.
    global_functions.cpp     merge_plot_files() ('y_merge' + 'y_plot').
 
Scope / fidelity notes:
- This module does ONE thing: reproduce the exact byte layout of
  'ploti.asc'/'plot.asc' that the original writes, so the already-
  installed CADAC Studio / KPlot can read it. It does not try to
  reimplement anything KPlot itself does (2Dim, Globe, Bivar, MCAP,
  ...) -- see manual-usuario-cadac-py.md discussion of Etapa 2 scope.
- Deliberately does NOT touch cadac_py/execution.py's validated
  Etapa-1 physics loop. It hooks into it via the optional
  'on_plot_sample' callback parameter added to run_scenario() (a
  no-op when the parameter is omitted, so every existing call site
  is byte-for-byte unaffected). See run_scenario_with_plot() below,
  which is the only supported entry point for Etapa 2.
- Numeric formatting: the original relies on the C++ stream default
  (precision 6, general/'%g'-style) for 'operator<<' on doubles --
  reproduced here with Python's '%.6g' rather than repr()/str(),
  which would emit full float precision and not match what KPlot's
  own (unknown, closed-source) tokenizer was written against.
- Line endings: the original 'ofstream' is opened in Windows TEXT mode
  (no 'ios::binary'), where every '\\n' the C++ runtime writes is
  transparently translated to CRLF ('\\r\\n') on disk. CADAC Studio/
  KPlot -- itself a Windows app -- expects that CRLF-terminated text.
  All files this module writes are therefore opened with
  newline='\\r\\n' so Python's text-mode translation reproduces the
  same on-disk bytes, even though this module runs on Linux. Getting
  this wrong does not corrupt the token content (whitespace-only
  difference) but was observed to break KPlot 3.3's own header/record
  reader (ReadHeader "Input past end of file", GetMaxMin "Overflow"),
  which evidently relies on fixed-width/fixed-record assumptions
  sensitive to the exact byte layout of each line.
- A vehicle without a second ('missile'/'plane') Variable array --
  i.e. anything that is not a Missile/Plane-shaped flight vehicle,
  chiefly Target -- raises TypeError from _flight_vehicle_arrays() if
  asked to write a banner/row; run_scenario_with_plot() catches this
  per-vehicle and treats it exactly like the original's empty stub:
  silently skipped, no file ever opened for it.
"""
from __future__ import annotations
 
import datetime
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, TextIO
 
from .core import Variable
from .missile import Missile
from .target import Target
from .falcon6 import Plane
from . import execution as _execution
 
_TAG = "plot"
_FIELD_WIDTH = 16
_LABEL_TRUNC = 8
_COLS_PER_LINE = 5
 
 
# --------------------------------------------------------------------- #
# Missile::plot_array()/plot_index_arrays() and Plane's twins: the set
# of Variable objects tagged 'plot', flat6[] first then the vehicle-
# specific array, in index order -- same convention execution.py's
# make_packet() already uses for the 'com' tag.
# --------------------------------------------------------------------- #
def _flight_vehicle_arrays(vehicle):
    if isinstance(vehicle, Missile):
        return vehicle.flat6, vehicle.missile
    if isinstance(vehicle, Plane):
        return vehicle.flat6, vehicle.plane
    raise TypeError(
        f"{type(vehicle).__name__} has no 'ploti.asc' output in the "
        "original CADAC++ (Target::plot_banner/plot_data are empty "
        "inline stubs in class_hierarchy.hpp) -- do not call "
        "write_plot_banner()/write_plot_data() on it"
    )
 
 
def _plot_variables(vehicle) -> List[Variable]:
    arr1, arr2 = _flight_vehicle_arrays(vehicle)
    return [v for v in arr1 if _TAG in v.get_out()] + [v for v in arr2 if _TAG in v.get_out()]
 
 
def _fmt(value: float) -> str:
    """C++ 'ofstream << double' with default precision(6), general
    format -- NOT Python's repr()/str(), which would print full
    float precision and not match the original byte-for-byte."""
    return f"{value:.6g}"
 
 
class _ColumnWriter:
    """Reproduces the '5 across, field width 16, ios::left' wrapping
    that both plot_banner() and plot_data() implement by hand with a
    running 'k' counter reset every 5 fields."""
 
    def __init__(self) -> None:
        self._col = 0
        self._parts: List[str] = []
 
    def emit(self, text: str) -> None:
        if self._col > 4:
            self._col = 0
            self._parts.append("\n")
        self._parts.append(text.ljust(_FIELD_WIDTH))
        self._col += 1
 
    def text(self) -> str:
        return "".join(self._parts)
 
    @property
    def col(self) -> int:
        return self._col
 
 
# --------------------------------------------------------------------- #
# Missile::plot_banner() / Plane::plot_banner() -- byte-exact port
# --------------------------------------------------------------------- #
def write_plot_banner(fplot: TextIO, vehicle, vehicle_name: str, title: str,
                       date_time: Optional[str] = None) -> None:
    if date_time is None:
        date_time = datetime.datetime.now().strftime("%b %d %Y %H:%M:%S")
 
    plot_vars = _plot_variables(vehicle)
    n_vectors = sum(1 for v in plot_vars if v.get_name()[:1].isupper())
    nvariables = len(plot_vars) + 2 * n_vectors
 
    # line 1: "1" + title + " '" + vehicle_name + " ' " + date + " " + time
    fplot.write(f"1{title} '{vehicle_name} ' {date_time}\n")
    # line 2: "  0  0 " + nvariables
    fplot.write(f"  0  0 {nvariables}\n")
 
    cw = _ColumnWriter()
    for var in plot_vars:
        name = var.get_name()[:_LABEL_TRUNC]
        if name[:1].isupper():
            for k in (1, 2, 3):
                cw.emit(name + str(k))
        else:
            cw.emit(name)
 
    fplot.write(cw.text())
    # ported as-is: the original only appends a trailing '\n' here if
    # nvariables isn't a multiple of 5 -- an asymmetry with plot_data(),
    # which always ends its row with '\n'.
    if nvariables % 5:
        fplot.write("\n")
 
 
# --------------------------------------------------------------------- #
# Missile::plot_data() / Plane::plot_data() -- byte-exact port,
# including the merge-sentinel ("-1.0" in place of the time field).
# --------------------------------------------------------------------- #
def write_plot_data(fplot: TextIO, vehicle, merge: bool = False) -> None:
    plot_vars = _plot_variables(vehicle)
    cw = _ColumnWriter()
 
    for i, var in enumerate(plot_vars):
        name = var.get_name()
        if var.get_type() == "int":
            cw.emit(_fmt(float(var.integer())))
        elif name[:1].isupper():
            vec = var.vec()
            for k in range(3):
                cw.emit(_fmt(vec[k]))
        else:
            # original: 'if(merge && index==0)' -- index 0 is always
            # flat6[0] ('time'), which is always the FIRST 'plot'-tagged
            # variable in every vehicle def_newton()/def_environment(),
            # so 'i == 0' here is the exact Python equivalent.
            if merge and i == 0:
                cw.emit("-1.0")
            else:
                cw.emit(_fmt(var.real()))
 
    fplot.write(cw.text())
    fplot.write("\n")
 
 
# --------------------------------------------------------------------- #
# global_functions.cpp merge_plot_files() -- byte-exact port
# --------------------------------------------------------------------- #
def merge_plot_files(plot_paths: Sequence[str], out_path: str, title: str,
                      date_time: Optional[str] = None) -> None:
    if date_time is None:
        date_time = datetime.datetime.now().strftime("%b %d %Y %H:%M:%S")
    if not plot_paths:
        return
 
    # newline="" on read: don't let universal-newline translation hide
    # which line-ending convention the source files actually use --
    # splitlines() below strips whichever it finds either way.
    with open(plot_paths[0], "r", newline="") as f0:
        first_line = f0.readline()
        second_line = f0.readline()
        num_labels = int(second_line.split()[-1])
    num_label_lines = num_labels // 5 + (1 if num_labels % 5 else 0)
    strip_lines = num_label_lines + 2
 
    # CRLF on write: see module docstring -- matches the original's
    # Windows text-mode ofstream, which KPlot's own reader depends on.
    with open(out_path, "w", newline="\r\n") as fmerge:
        with open(plot_paths[0], "r", newline="") as f0:
            lines = f0.read().splitlines()
        fmerge.write(f"1{title}  {date_time}")
        for line in lines[1:]:
            fmerge.write("\n" + line)
 
        for path in plot_paths[1:]:
            with open(path, "r", newline="") as fi:
                lines = fi.read().splitlines()
            for line in lines[strip_lines:]:
                fmerge.write(line + "\n")
 
 
# --------------------------------------------------------------------- #
# Driver: runs Etapa-1's validated execution.run_scenario() unmodified
# in its physics, hooking plot-file writing through the optional
# 'on_plot_sample' callback. Mirrors execute()'s plot cadence exactly:
# first sample at sim_time==0, then every 'plot_step', with the same
# cross-vehicle 'only advance after the last vehicle in the group'
# synchronization, plus the unconditional post-loop merge-sentinel row.
# --------------------------------------------------------------------- #
@dataclass
class PlotRunResult:
    plot_files: Dict[str, str] = field(default_factory=dict)   # vehicle_id -> path
    merged_file: Optional[str] = None
 
 
def run_scenario_with_plot(input_path: str, plot_dir: str = ".",
                            out_csv: Optional[str] = None,
                            end_time_override: Optional[float] = None,
                            int_step_override: Optional[float] = None,
                            verbose: bool = True) -> PlotRunResult:
    """Runs 'input_path' exactly like execution.run_scenario(), and, if
    its OPTIONS line requests 'y_plot', additionally writes
    'plot<k>.asc' (k=1,2,... in vehicle order, matching the original's
    per-MISSILE6/PLANE6-object numbering) into 'plot_dir', merging them
    into 'plot.asc' if OPTIONS also requests 'y_merge'. TARGET3
    vehicles are skipped, matching the original silently."""
    os.makedirs(plot_dir, exist_ok=True)
    parsed = _execution.parse_input_asc(input_path)
    plot_step = parsed.timing["plot_step"] or parsed.timing["int_step"]
    y_plot = "y_plot" in parsed.options
    y_merge = "y_merge" in parsed.options
    date_time = datetime.datetime.now().strftime("%b %d %Y %H:%M:%S")
 
    result = PlotRunResult()
    if not y_plot:
        parsed_ret, vehicles, combus, rows = _execution.run_scenario(
            input_path, out_csv=out_csv, end_time_override=end_time_override,
            int_step_override=int_step_override, verbose=verbose)
        return result
 
    streams: Dict[int, TextIO] = {}
    paths: Dict[int, str] = {}
    names: List[Optional[str]] = [spec.name for spec in parsed.vehicles]
    banner_written: Dict[int, bool] = {}
    plot_index_by_kind: Dict[str, int] = {}
 
    def _open_stream_if_needed(idx: int, vehicle) -> Optional[TextIO]:
        if idx in streams:
            return streams[idx]
        try:
            _flight_vehicle_arrays(vehicle)   # raises for Target
        except TypeError:
            streams[idx] = None
            return None
        kind = "MISSILE6" if isinstance(vehicle, Missile) else "PLANE6"
        plot_index_by_kind[kind] = plot_index_by_kind.get(kind, 0) + 1
        path = os.path.join(plot_dir, f"plot{plot_index_by_kind[kind]}.asc")
        # CRLF: see module docstring -- matches the original's Windows
        # text-mode ofstream, which KPlot's own reader depends on.
        stream = open(path, "w", newline="\r\n")
        streams[idx] = stream
        paths[idx] = path
        result.plot_files[f"vehicle_{idx}"] = path
        return stream
 
    def on_plot_sample(idx: int, vehicle, sim_time: float, merge: bool) -> None:
        stream = _open_stream_if_needed(idx, vehicle)
        if stream is None:
            return
        if not banner_written.get(idx):
            write_plot_banner(stream, vehicle, names[idx] or "", parsed.title, date_time)
            banner_written[idx] = True
        write_plot_data(stream, vehicle, merge=merge)
 
    parsed_ret, vehicles, combus, rows = _execution.run_scenario(
        input_path, out_csv=out_csv, end_time_override=end_time_override,
        int_step_override=int_step_override, verbose=verbose,
        plot_step_override=plot_step, on_plot_sample=on_plot_sample,
    )
 
    for stream in streams.values():
        if stream is not None:
            stream.close()
 
    if y_merge:
        missile_paths = [paths[i] for i, v in enumerate(vehicles)
                          if i in paths and isinstance(v, Missile)]
        if missile_paths:
            merged_path = os.path.join(plot_dir, "plot.asc")
            merge_plot_files(missile_paths, merged_path, parsed.title, date_time)
            result.merged_file = merged_path
 
    return result
"""
Port of 'global_header.hpp' -> classes 'Module', 'Variable', 'Event',
'Packet', 'Document'.

Original: Peter H Zipfel, CADAC++ (global_header.hpp)

Design notes:
- C++ 'Variable::init(...)' is 4-times overloaded (real / int / vector /
  matrix). Python has no overloading, so this port dispatches on the
  number of positional arguments after 'name', exactly mirroring which
  C++ overload would have been selected by the compiler. Call sites
  copied verbatim from the .cpp files therefore work unchanged.
- Everything that was a getter/setter pair on a private C++ member is
  kept as a getter/setter pair here (rather than turned into a plain
  attribute) so the ported module code -- which calls e.g.
  'flat6[210].vec()' or 'flat6[230].gets_vec(FSPB)' -- reads identically
  to the original.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .matrix import Matrix

NVAR = 15  # global_constants.hpp
NEVENT = 20


# --------------------------------------------------------------------- #
# struct Module
# --------------------------------------------------------------------- #
@dataclass
class Module:
    name: str = ""
    definition: str = ""
    initialization: str = ""
    execution: str = ""
    termination: str = ""


# --------------------------------------------------------------------- #
# class Variable
# --------------------------------------------------------------------- #
class Variable:
    __slots__ = ("_name", "_type", "_rval", "_ival", "_vec", "_mat",
                 "_def", "_mod", "_role", "_out", "_error")

    def __init__(self):
        self._name = "empty"
        self._type = ""
        self._rval = 0.0
        self._ival = 0
        self._vec = Matrix(3, 1)
        self._mat = Matrix(3, 3)
        self._def = ""
        self._mod = ""
        self._role = ""
        self._out = ""
        self._error = " "

    # ------------------------------------------------------------------
    # init(...) - 4-times overloaded in C++, dispatched here by arg count
    # ------------------------------------------------------------------
    def init(self, name: str, *rest):
        self._name = name
        n = len(rest)
        if n == 5:
            # init(na, rv, de, mo, ro, ou)  -- real
            rv, de, mo, ro, ou = rest
            self._type = "real"
            self._rval = float(rv)
            self._def, self._mod, self._role, self._out = de, mo, ro, ou
        elif n == 6:
            # init(na, ty, iv, de, mo, ro, ou) -- integer
            ty, iv, de, mo, ro, ou = rest
            self._type = ty
            self._ival = int(iv)
            self._def, self._mod, self._role, self._out = de, mo, ro, ou
        elif n == 7:
            # init(na, v1, v2, v3, de, mo, ro, ou) -- vector
            v1, v2, v3, de, mo, ro, ou = rest
            self._type = "vec"
            self._vec = Matrix().build_vec3(v1, v2, v3)
            self._def, self._mod, self._role, self._out = de, mo, ro, ou
        elif n == 13:
            # init(na, v11..v33, de, mo, ro, ou) -- 3x3 matrix
            v11, v12, v13, v21, v22, v23, v31, v32, v33, de, mo, ro, ou = rest
            self._type = "mat"
            self._mat = Matrix().build_mat33(v11, v12, v13, v21, v22, v23, v31, v32, v33)
            self._def, self._mod, self._role, self._out = de, mo, ro, ou
        else:
            raise TypeError(f"Variable.init(): unsupported overload with {n} args after name")
        self._error = " "
        return self

    # ------------------------------------------------------------------ #
    # getters matching the C++ inline accessors
    # ------------------------------------------------------------------ #
    def get_name(self) -> str:
        return self._name

    def get_type(self) -> str:
        return self._type

    def real(self) -> float:
        return self._rval

    def integer(self) -> int:
        return self._ival

    def vec(self) -> Matrix:
        return self._vec.copy()

    def mat(self) -> Matrix:
        return self._mat.copy()

    def get_def(self) -> str:
        return self._def

    def get_mod(self) -> str:
        return self._mod

    def get_role(self) -> str:
        return self._role

    def get_out(self) -> str:
        return self._out

    def get_error(self) -> str:
        return self._error

    def put_error(self, error_code: str) -> None:
        self._error = error_code

    # ------------------------------------------------------------------ #
    # gets(...) - overloaded loaders, dispatch by type of argument
    # ------------------------------------------------------------------ #
    def gets(self, value):
        if isinstance(value, bool):
            self._ival = int(value)
            return self._ival
        if isinstance(value, int):
            self._ival = value
            return self._ival
        self._rval = float(value)
        return self._rval

    def gets_vec(self, vec: Matrix) -> Matrix:
        self._vec = vec.copy()
        return self._vec

    def gets_mat(self, mat: Matrix) -> Matrix:
        self._mat = mat.copy()
        return self._mat

    def __repr__(self) -> str:
        return f"Variable(name={self._name!r}, type={self._type!r})"


# --------------------------------------------------------------------- #
# class Event
# --------------------------------------------------------------------- #
class Event:
    def __init__(self):
        self.watch_variable: Optional[Variable] = None
        self.watch_value: float = 0.0
        self.event_operator: str = "="
        self.flat6_indices: List[int] = [0] * NVAR
        self.flat6_values: List[float] = [0.0] * NVAR
        self.flat6_size: int = 0
        self.missile_indices: List[int] = [0] * NVAR
        self.missile_values: List[float] = [0.0] * NVAR
        self.missile_size: int = 0

    def set_variable(self, variable: Variable) -> None:
        self.watch_variable = variable

    def set_value(self, value: float) -> None:
        self.watch_value = value

    def set_operator(self, criterion: str) -> None:
        self.event_operator = criterion

    def set_flat6_index(self, element: int, index: int) -> None:
        self.flat6_indices[element] = index

    def set_flat6_value(self, element: int, value: float) -> None:
        self.flat6_values[element] = value

    def set_flat6_size(self, size: int) -> None:
        self.flat6_size = size

    def set_missile_index(self, element: int, index: int) -> None:
        self.missile_indices[element] = index

    def set_missile_value(self, element: int, value: float) -> None:
        self.missile_values[element] = value

    def set_missile_size(self, size: int) -> None:
        self.missile_size = size

    def get_variable(self) -> Optional[Variable]:
        return self.watch_variable

    def get_value(self) -> float:
        return self.watch_value

    def get_operator(self) -> str:
        return self.event_operator

    def get_flat6_indices(self) -> List[int]:
        return self.flat6_indices

    def get_flat6_values(self) -> List[float]:
        return self.flat6_values

    def get_flat6_size(self) -> int:
        return self.flat6_size

    def get_missile_indices(self) -> List[int]:
        return self.missile_indices

    def get_missile_values(self) -> List[float]:
        return self.missile_values

    def get_missile_size(self) -> int:
        return self.missile_size


# --------------------------------------------------------------------- #
# class Packet - data cluster per vehicle, used on the 'combus'
# --------------------------------------------------------------------- #
class Packet:
    def __init__(self):
        self.id: str = ""
        self.status: int = 1  # alive=1, dead=0, hit=-1 (targets only)
        self.ndata: int = 0
        self.data: List[Variable] = []

    def set_id(self, identification: str) -> None:
        self.id = identification

    def set_status(self, vehicle_status: int) -> None:
        self.status = vehicle_status

    def set_ndata(self, vehicle_ndata: int) -> None:
        self.ndata = vehicle_ndata

    def set_data(self, vehicle_data: List[Variable]) -> None:
        self.data = vehicle_data

    def get_id(self) -> str:
        return self.id

    def get_status(self) -> int:
        return self.status

    def get_ndata(self) -> int:
        return self.ndata

    def get_data(self) -> List[Variable]:
        return self.data


# --------------------------------------------------------------------- #
# class Document - stores a subset of module-variables for documentation
# --------------------------------------------------------------------- #
@dataclass
class Document:
    doc_offset: int = 0
    name: str = "end_array"
    type: str = ""
    def_: str = ""
    mod: str = ""

    def get_doc_offset(self) -> int:
        return self.doc_offset

    def get_name(self) -> str:
        return self.name

    def get_type(self) -> str:
        return self.type

    def get_def(self) -> str:
        return self.def_

    def get_mod(self) -> str:
        return self.mod

    def put_doc_offset(self, mark: int) -> None:
        self.doc_offset = mark

    def put_name(self, name: str) -> None:
        self.name = name

    def put_type(self, type_: str) -> None:
        self.type = type_

    def put_def(self, def_: str) -> None:
        self.def_ = def_

    def put_mod(self, mod: str) -> None:
        self.mod = mod

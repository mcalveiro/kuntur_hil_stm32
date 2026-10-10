"""
Port of 'class_hierarchy.hpp'/'utility_functions.cpp' -> classes 'Table',
'Datadeck', plus the deck-file reader ported from
'Missile::read_tables()' (missile_functions.cpp).

Original: Peter H Zipfel, CADAC++

The C++ reader tokenizes the *_aero_deck.asc / *_prop_deck.asc files with
'ifstream >> token' (whitespace-delimited, cross-newline). This port uses
the same whitespace tokenization via a small TokenStream so deck files are
read identically regardless of how many values are packed per line.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import List

EPS = 1.0e-10


_WS_RE = re.compile(r"\s*")
_WORD_RE = re.compile(r"\S+")
_FLOAT_RE = re.compile(r"[+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?")
_INT_RE = re.compile(r"[+-]?\d+")


class _TokenStream:
    """Mimics C++ 'ifstream >> token' character-by-character extraction.

    Some CADAC deck files pack adjacent negative numbers with no separating
    whitespace (e.g. '-9.582-14.563', a fixed-column-width artifact). C++'s
    'stream >> double' stops as soon as the numeric grammar is no longer
    satisfied, so it silently splits that into '-9.582' then '-14.563' on
    the next extraction. A naive whitespace .split() would instead treat it
    as one unparseable token, so this stream advances a raw character
    position and applies the same numeric-grammar matching as C++.
    """

    def __init__(self, text: str):
        # strip '//' comments (rest of line), exactly like the original
        # which reads full lines with getline() only to skip comments and
        # otherwise free-tokenizes with '>>'
        cleaned_lines = []
        for line in text.splitlines():
            idx = line.find("//")
            cleaned_lines.append(line[:idx] if idx != -1 else line)
        self._text = "\n".join(cleaned_lines)
        self._pos = 0
        self._n = len(self._text)

    def _skip_ws(self) -> None:
        m = _WS_RE.match(self._text, self._pos)
        if m:
            self._pos = m.end()

    def next(self) -> str:
        """Whitespace-delimited word (C++ '>> char*'/'>> string')."""
        self._skip_ws()
        m = _WORD_RE.match(self._text, self._pos)
        if not m:
            raise EOFError("unexpected end of deck file")
        self._pos = m.end()
        return m.group(0)

    def next_float(self) -> float:
        """C++ '>> double': matches the longest valid float grammar,
        which may stop mid-word (see class docstring)."""
        self._skip_ws()
        m = _FLOAT_RE.match(self._text, self._pos)
        if not m:
            raise ValueError(f"expected a number at position {self._pos}: "
                              f"...{self._text[self._pos:self._pos+20]!r}...")
        self._pos = m.end()
        return float(m.group(0))

    def next_int(self) -> int:
        """C++ '>> int': integer grammar only."""
        self._skip_ws()
        m = _INT_RE.match(self._text, self._pos)
        if not m:
            raise ValueError(f"expected an integer at position {self._pos}")
        self._pos = m.end()
        return int(m.group(0))

    def eof(self) -> bool:
        self._skip_ws()
        return self._pos >= self._n


@dataclass
class Table:
    name: str = ""
    dim: int = 0
    var1_dim: int = 1
    var2_dim: int = 1
    var3_dim: int = 1
    var1_values: List[float] = field(default_factory=list)
    var2_values: List[float] = field(default_factory=list)
    var3_values: List[float] = field(default_factory=list)
    data: List[float] = field(default_factory=list)

    def get_dim(self) -> int:
        return self.dim

    def get_name(self) -> str:
        return self.name

    def get_var1_dim(self) -> int:
        return self.var1_dim

    def get_var2_dim(self) -> int:
        return self.var2_dim

    def get_var3_dim(self) -> int:
        return self.var3_dim


class Datadeck:
    """Port of class Datadeck: table look-up and interpolation."""

    def __init__(self):
        self.title: str = ""
        self.tables: List[Table] = []
        self._by_name = {}

    # -- construction (loading) ------------------------------------- #
    def set_title(self, title: str) -> None:
        self.title = title

    def get_title(self) -> str:
        return self.title

    def add_table(self, table: Table) -> None:
        self.tables.append(table)
        self._by_name[table.name] = table

    def get_capacity(self) -> int:
        return len(self.tables)

    def get_tbl(self, slot: int) -> Table:
        return self.tables[slot]

    def __getitem__(self, slot: int) -> Table:
        return self.tables[slot]

    # -- look-up (public API used by all module code) ----------------- #
    def look_up(self, name: str, value1: float, value2: float = None, value3: float = None) -> float:
        tbl = self._by_name[name]

        loc1 = self._find_index(tbl.var1_dim - 1, value1, tbl.var1_values)

        if value2 is None:
            # single-variable look-up: constant extrapolation at upper end
            if loc1 == tbl.var1_dim - 1:
                return tbl.data[loc1]
            return self._interpolate1(loc1, loc1 + 1, tbl, value1)

        loc2 = self._find_index(tbl.var2_dim - 1, value2, tbl.var2_values)

        if value3 is None:
            return self._interpolate2(loc1, loc1 + 1, loc2, loc2 + 1, tbl, value1, value2)

        loc3 = self._find_index(tbl.var3_dim - 1, value3, tbl.var3_values)
        return self._interp3(
            loc1, loc1 + 1, loc2, loc2 + 1, loc3, loc3 + 1, tbl, value1, value2, value3
        )

    # -- internals, ported verbatim ------------------------------------ #
    @staticmethod
    def _find_index(max_: int, value: float, values: List[float]) -> int:
        if value >= values[max_]:
            return max_
        if value <= values[0]:
            return 0
        index = 0
        mx = max_
        while index <= mx:
            mid = (index + mx) // 2
            if value < values[mid]:
                mx = mid - 1
            elif value > values[mid]:
                index = mid + 1
            else:
                return mid
        return mx

    @staticmethod
    def _interpolate1(ind1: int, ind2: int, tbl: Table, val: float) -> float:
        diff = val - tbl.var1_values[ind1]
        dx = tbl.var1_values[ind2] - tbl.var1_values[ind1]
        dumx = diff / dx if dx > EPS else 0.0
        dy = dumx * (tbl.data[ind2] - tbl.data[ind1])
        return tbl.data[ind1] + dy

    @staticmethod
    def _interpolate2(ind10: int, ind11: int, ind20: int, ind21: int,
                       tbl: Table, value1: float, value2: float) -> float:
        var1_dim, var2_dim = tbl.var1_dim, tbl.var2_dim
        diff1 = value1 - tbl.var1_values[ind10]
        diff2 = value2 - tbl.var2_values[ind20]

        dx1 = 0.0
        if ind10 == var1_dim - 1:
            ind11 = ind10
        else:
            dx1 = tbl.var1_values[ind11] - tbl.var1_values[ind10]

        dx2 = 0.0
        if ind20 == var2_dim - 1:
            ind21 = ind20
        else:
            dx2 = tbl.var2_values[ind21] - tbl.var2_values[ind20]

        dumx1 = diff1 / dx1 if dx1 > EPS else 0.0
        dumx2 = diff2 / dx2 if dx2 > EPS else 0.0

        y11 = tbl.data[ind10 * var2_dim + ind20]
        y12 = tbl.data[ind10 * var2_dim + ind21]
        y21 = tbl.data[ind11 * var2_dim + ind20]
        y22 = tbl.data[ind11 * var2_dim + ind21]
        y1 = dumx1 * (y21 - y11) + y11
        y2 = dumx1 * (y22 - y12) + y12
        return dumx2 * (y2 - y1) + y1

    # 3-D interpolation, kept literal to match the exact (unusual) flat-index
    # arithmetic of the original C++ function (Datadeck::interpolate, 3 vars).
    @staticmethod
    def _interp3(ind10, ind11, ind20, ind21, ind30, ind31, tbl: Table,
                 value1, value2, value3):
        var2_dim, var3_dim = tbl.var2_dim, tbl.var3_dim
        data = tbl.data
        diff1 = value1 - tbl.var1_values[ind10]
        diff2 = value2 - tbl.var2_values[ind20]
        diff3 = value3 - tbl.var3_values[ind30]

        dx1 = 0.0
        if ind10 == tbl.var1_dim - 1:
            ind11 = ind10
        else:
            dx1 = tbl.var1_values[ind11] - tbl.var1_values[ind10]
        dx2 = 0.0
        if ind20 == var2_dim - 1:
            ind21 = ind20
        else:
            dx2 = tbl.var2_values[ind21] - tbl.var2_values[ind20]
        dx3 = 0.0
        if ind30 == var3_dim - 1:
            ind31 = ind30
        else:
            dx3 = tbl.var3_values[ind31] - tbl.var3_values[ind30]

        dumx1 = diff1 / dx1 if dx1 > EPS else 0.0
        dumx2 = diff2 / dx2 if dx2 > EPS else 0.0
        dumx3 = diff3 / dx3 if dx3 > EPS else 0.0

        base = ind10 * var2_dim * var3_dim
        off = var2_dim * var3_dim

        y11 = data[base + ind20 * var3_dim + ind30]
        y12 = data[base + ind20 * var3_dim + ind30 + off]
        y31 = data[base + ind20 * var3_dim + ind31]
        y32 = data[base + ind20 * var3_dim + ind31 + off]
        y1 = dumx1 * (y12 - y11) + y11
        y3 = dumx1 * (y32 - y31) + y31
        y21 = dumx3 * (y3 - y1) + y1

        y11 = data[base + ind21 * var3_dim + ind30]
        y12 = data[base + ind21 * var3_dim + ind30 + off]
        y31 = data[base + ind21 * var3_dim + ind31]
        y32 = data[base + ind21 * var3_dim + ind31 + off]
        y1 = dumx1 * (y12 - y11) + y11
        y3 = dumx1 * (y32 - y31) + y31
        y22 = dumx3 * (y3 - y1) + y1

        return dumx2 * (y22 - y21) + y21


def read_deck(file_path: str) -> Datadeck:
    """Port of Missile::read_tables() (missile_functions.cpp).

    Parses a CADAC data-deck ASCII file (*_aero_deck.asc, *_prop_deck.asc)
    containing one or more 'NDIM <name>' tables, each followed by
    'NX1 n [NX2 m [NX3 p]]' and then the tabulated values.
    """
    with open(file_path, "r") as f:
        text = f.read()

    ts = _TokenStream(text)
    deck = Datadeck()

    title = ""
    tok = ts.next()
    if tok == "TITLE":
        # title is the remainder of tokens up to the first *DIM marker
        title_parts = []
        tok = ts.next()
        while "DIM" not in tok:
            title_parts.append(tok)
            tok = ts.next()
        title = " ".join(title_parts)
    deck.set_title(title)

    # 'tok' now holds the first '<n>DIM' marker
    while True:
        m = re.match(r"^(\d)DIM$", tok)
        if not m:
            break
        table_dim = int(m.group(1))

        name = ts.next()
        table = Table(name=name, dim=table_dim)

        var_dim = [1, 1, 1]
        for i in range(table_dim):
            _label = ts.next()  # 'NX1' / 'NX2' / 'NX3'
            var_dim[i] = ts.next_int()
        table.var1_dim, table.var2_dim, table.var3_dim = var_dim

        num_rows = max(var_dim)
        table.var1_values = [0.0] * var_dim[0]
        table.var2_values = [0.0] * var_dim[1]
        table.var3_values = [0.0] * var_dim[2]
        table.data = [0.0] * (var_dim[0] * var_dim[1] * var_dim[2])

        for tt in range(num_rows):
            if tt < var_dim[0]:
                table.var1_values[tt] = ts.next_float()
            if tt < var_dim[1] and var_dim[1] != 1:
                table.var2_values[tt] = ts.next_float()
            if tt < var_dim[2] and var_dim[2] != 1:
                table.var3_values[tt] = ts.next_float()
            if tt < var_dim[0]:
                for ttt in range(var_dim[1] * var_dim[2]):
                    table.data[tt * var_dim[1] * var_dim[2] + ttt] = ts.next_float()

        deck.add_table(table)

        if ts.eof():
            break
        tok = ts.next()

    return deck

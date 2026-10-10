"""
Port of 'utility_header.hpp' / 'utility_functions.cpp' -> class 'Matrix'
Original: Peter H Zipfel, CADAC++ (utility_header.hpp)

Faithful port of the CADAC++ Matrix class. In the original C++, Matrix is a
dynamically-sized (row x col) array of doubles with a large set of member
functions used throughout the simulation (vector algebra, 3x3 rotation
utilities, table-friendly conversions, etc). Here it is backed by a numpy
array but keeps the exact method names/semantics of the C++ class so the
module code that uses it (ported 1:1 from the .cpp files) reads the same.

Notes on fidelity:
- 'operator[]' in the C++ class only works for vectors (single-column or
  single-row matrices) -> implemented as __getitem__/__setitem__.
- 'get_loc'/'assign_loc' address arbitrary (row, col) elements.
- Relational operators (==, !=) use the same EPS tolerance as the original.
- Angles are in radians unless stated otherwise (matches the source).
"""
from __future__ import annotations

import numpy as np

EPS = 1.0e-10  # global_constants.hpp


class Matrix:
    __slots__ = ("_a",)

    # ------------------------------------------------------------------ #
    # Construction
    # ------------------------------------------------------------------ #
    def __init__(self, row_size: int = 0, col_size: int = 0):
        """Matrix() default ctor + Matrix(row_size, col_size) overload."""
        if row_size <= 0 or col_size <= 0:
            self._a = np.zeros((0, 0), dtype=float)
        else:
            self._a = np.zeros((row_size, col_size), dtype=float)

    @classmethod
    def _from_array(cls, arr: np.ndarray) -> "Matrix":
        m = cls.__new__(cls)
        m._a = np.array(arr, dtype=float, copy=True)
        if m._a.ndim == 1:
            m._a = m._a.reshape(-1, 1)
        return m

    def copy(self) -> "Matrix":
        """copy constructor Matrix(const Matrix &MAT)"""
        return Matrix._from_array(self._a)

    # ------------------------------------------------------------------ #
    # dimension / zero / ones / identity
    # ------------------------------------------------------------------ #
    def dimension(self, row: int, col: int) -> None:
        self._a = np.zeros((row, col), dtype=float)

    def zero(self) -> "Matrix":
        self._a[:, :] = 0.0
        return self

    def ones(self) -> "Matrix":
        self._a[:, :] = 1.0
        return self

    def identity(self) -> "Matrix":
        r, c = self._a.shape
        n = min(r, c)
        self._a[:, :] = 0.0
        for i in range(n):
            self._a[i, i] = 1.0
        return self

    # ------------------------------------------------------------------ #
    # sizes / raw access
    # ------------------------------------------------------------------ #
    def get_rows(self) -> int:
        return self._a.shape[0]

    def get_cols(self) -> int:
        return self._a.shape[1]

    def get_index(self, row: int, col: int) -> int:
        """Row-major offset index, matches C++ pointer arithmetic layout."""
        return row * self.get_cols() + col

    def get_pbody(self) -> np.ndarray:
        return self._a

    def get_loc(self, r: int, c: int) -> float:
        return float(self._a[r, c])

    def assign_loc(self, r: int, c: int, val: float) -> None:
        self._a[r, c] = val

    # ------------------------------------------------------------------ #
    # builders
    # ------------------------------------------------------------------ #
    def build_vec3(self, v1: float, v2: float, v3: float) -> "Matrix":
        self._a = np.array([[v1], [v2], [v3]], dtype=float)
        return self

    def build_mat33(
        self,
        v11: float, v12: float, v13: float,
        v21: float, v22: float, v23: float,
        v31: float, v32: float, v33: float,
    ) -> "Matrix":
        self._a = np.array(
            [[v11, v12, v13], [v21, v22, v23], [v31, v32, v33]], dtype=float
        )
        return self

    def cart_from_pol(self, magnitude: float, azimuth: float, elevation: float) -> "Matrix":
        v1 = magnitude * np.cos(elevation) * np.cos(azimuth)
        v2 = magnitude * np.cos(elevation) * np.sin(azimuth)
        v3 = -magnitude * np.sin(elevation)
        self._a = np.array([[v1], [v2], [v3]], dtype=float)
        return self

    # ------------------------------------------------------------------ #
    # extraction
    # ------------------------------------------------------------------ #
    def col_vec(self, col: int) -> "Matrix":
        return Matrix._from_array(self._a[:, [col]])

    def row_vec(self, row: int) -> "Matrix":
        return Matrix._from_array(self._a[[row], :])

    def sub_matrix(self, row: int, col: int) -> "Matrix":
        """Returns matrix with 'row' and 'col' omitted (0-based)."""
        sub = np.delete(self._a, row, axis=0)
        sub = np.delete(sub, col, axis=1)
        return Matrix._from_array(sub)

    # ------------------------------------------------------------------ #
    # linear algebra
    # ------------------------------------------------------------------ #
    def absolute(self) -> float:
        """||VEC|| for a vector (Euclidean norm)."""
        return float(np.sqrt(np.sum(self._a * self._a)))

    def determinant(self) -> float:
        return float(np.linalg.det(self._a))

    def adjoint(self) -> "Matrix":
        n = self.get_rows()
        det = self.determinant()
        inv = np.linalg.inv(self._a)
        # adjoint = det(A) * inv(A)
        return Matrix._from_array(inv * det)

    def inverse(self) -> "Matrix":
        return Matrix._from_array(np.linalg.inv(self._a))

    def trans(self) -> "Matrix":
        return Matrix._from_array(self._a.T)

    def __invert__(self) -> "Matrix":
        """operator~() alternate transpose Aij=>Aji"""
        return self.trans()

    def diamat_vec(self) -> "Matrix":
        """nxn diagonal matrix from nx1 vector."""
        v = self._a.flatten()
        return Matrix._from_array(np.diag(v))

    def diavec_mat(self) -> "Matrix":
        """nx1 diagonal vector from nxn matrix."""
        d = np.diag(self._a)
        return Matrix._from_array(d.reshape(-1, 1))

    def mat33_vec9(self) -> "Matrix":
        v = self._a.flatten()
        return Matrix._from_array(v.reshape(3, 3))

    def vec9_mat33(self) -> "Matrix":
        return Matrix._from_array(self._a.flatten().reshape(9, 1))

    def univec3(self) -> "Matrix":
        mag = self.absolute()
        if mag < EPS:
            return Matrix._from_array(self._a * 0.0)
        return Matrix._from_array(self._a / mag)

    def skew_sym(self) -> "Matrix":
        a, b, c = self._a.flatten()[:3]
        return Matrix._from_array(
            np.array([[0.0, -c, b], [c, 0.0, -a], [-b, a, 0.0]])
        )

    def ellipse(self) -> "Matrix":
        """Bi-variate ellipse from symmetric pos.-semi-def 2x2 matrix.
        Returns ELLIPSE(0,0)=major semi-axis, (1,0)=minor semi-axis, (2,0)=angle.
        """
        a = self._a[0, 0]
        b = self._a[0, 1]
        d = self._a[1, 1]
        # eigenvalues of [[a,b],[b,d]]
        term = np.sqrt(((a - d) / 2.0) ** 2 + b ** 2)
        lam1 = (a + d) / 2.0 + term
        lam2 = (a + d) / 2.0 - term
        major = np.sqrt(max(lam1, 0.0))
        minor = np.sqrt(max(lam2, 0.0))
        if abs(b) < EPS and abs(a - d) < EPS:
            theta = 0.0
        else:
            theta = 0.5 * np.arctan2(2.0 * b, a - d)
        return Matrix._from_array(np.array([[major], [minor], [theta]]))

    def pol_from_cart(self) -> "Matrix":
        v1, v2, v3 = self._a.flatten()[:3]
        magnitude = np.sqrt(v1 * v1 + v2 * v2 + v3 * v3)
        azimuth = np.arctan2(v2, v1)
        elevation = np.arctan2(-v3, np.sqrt(v1 * v1 + v2 * v2))
        return Matrix._from_array(np.array([[magnitude], [azimuth], [elevation]]))

    # ------------------------------------------------------------------ #
    # operators
    # ------------------------------------------------------------------ #
    def __getitem__(self, r: int) -> float:
        """operator[] : vector element access (row or column vector)."""
        if self._a.shape[1] == 1:
            return float(self._a[r, 0])
        if self._a.shape[0] == 1:
            return float(self._a[0, r])
        raise IndexError("Matrix::operator[] only valid for row/column vectors")

    def __setitem__(self, r: int, value: float) -> None:
        if self._a.shape[1] == 1:
            self._a[r, 0] = value
        elif self._a.shape[0] == 1:
            self._a[0, r] = value
        else:
            raise IndexError("Matrix::operator[] only valid for row/column vectors")

    def __mul__(self, other):
        if isinstance(other, Matrix):
            return Matrix._from_array(self._a @ other._a)
        return Matrix._from_array(self._a * float(other))

    def __rmul__(self, other):
        # scalar * Matrix (not in original C++, but harmless/handy)
        return Matrix._from_array(self._a * float(other))

    def __add__(self, other):
        if isinstance(other, Matrix):
            return Matrix._from_array(self._a + other._a)
        return Matrix._from_array(self._a + float(other))

    def __sub__(self, other):
        if isinstance(other, Matrix):
            return Matrix._from_array(self._a - other._a)
        return Matrix._from_array(self._a - float(other))

    def __neg__(self):
        return Matrix._from_array(-self._a)

    def __xor__(self, other: "Matrix") -> float:
        """operator^ : scalar (dot) product of two vectors."""
        return float(np.sum(self._a.flatten() * other._a.flatten()))

    def __eq__(self, other) -> bool:
        if not isinstance(other, Matrix) or other._a.shape != self._a.shape:
            return False
        return bool(np.all(np.abs(self._a - other._a) < EPS))

    def __ne__(self, other) -> bool:
        return not self.__eq__(other)

    def __repr__(self) -> str:
        return f"Matrix({self._a.tolist()!r})"

    def print(self) -> None:
        print(self._a)

    def as_numpy(self) -> np.ndarray:
        """Escape hatch for code that wants raw numpy (not part of the C++ API)."""
        return self._a


# --------------------------------------------------------------------- #
# Module utility functions (free functions in utility_header.hpp)
# --------------------------------------------------------------------- #

def mat2tr(psivg: float, thtvg: float) -> Matrix:
    """T.M. of the psivg -> thtvg sequence (2 Euler-angle sequence)."""
    cpsi, spsi = np.cos(psivg), np.sin(psivg)
    ctht, stht = np.cos(thtvg), np.sin(thtvg)
    return Matrix().build_mat33(
        ctht * cpsi, ctht * spsi, -stht,
        -spsi, cpsi, 0.0,
        stht * cpsi, stht * spsi, ctht,
    )


def mat3tr(psi: float, tht: float, phi: float) -> Matrix:
    """Euler T.M. of the psi->tht->phi sequence (3-2-1 Euler sequence).
    Ported verbatim (element by element) from utility_functions.cpp::mat3tr.
    """
    spsi, cpsi = np.sin(psi), np.cos(psi)
    stht, ctht = np.sin(tht), np.cos(tht)
    sphi, cphi = np.sin(phi), np.cos(phi)
    return Matrix().build_mat33(
        cpsi * ctht, spsi * ctht, -stht,
        cpsi * stht * sphi - spsi * cphi, spsi * stht * sphi + cpsi * cphi, ctht * sphi,
        cpsi * stht * cphi + spsi * sphi, spsi * stht * cphi - cpsi * sphi, ctht * cphi,
    )


def sign(variable: float) -> int:
    return 1 if variable >= 0.0 else -1


def angle(vec1: Matrix, vec2: Matrix) -> float:
    """Angle (rad) between two 3x1 vectors."""
    mag1 = vec1.absolute()
    mag2 = vec2.absolute()
    if mag1 < EPS or mag2 < EPS:
        return 0.0
    cos_a = (vec1 ^ vec2) / (mag1 * mag2)
    cos_a = max(-1.0, min(1.0, cos_a))
    return float(np.arccos(cos_a))

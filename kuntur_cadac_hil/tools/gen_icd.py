#!/usr/bin/env python3
"""
gen_icd.py -- Genera icd.py (Python), icd.rs (kse_server) e
kuntur_ic_generated.h (C++, incluido por kuntur_ic.cpp) a partir de
icd_schema/kuntur_ic_v1_3.json, la única fuente de verdad del contrato
Kuntur-IC v1.3.

Uso:
    python3 tools/gen_icd.py [ruta_al_schema.json]

Sin dependencias de terceros (usa json de la librería estándar).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCHEMA_PATH = ROOT / "icd_schema" / "kuntur_ic_v1_3.json"

# f64 agregado para el fork HIL (icd_schema/kuntur_ic_plant_state_v1.json,
# kuntur_ic_surface_cmd_v1.json): el STM32F767 (FPU FPv5-D16) opera en
# double nativo -- es lo que habilita la validación bit-exacta contra
# Python (ver plan-hil-stm32-portado-autopiloto.md §7). Nadie más lo usa
# todavía (kuntur_ic_v1_3.json sigue en f32, sin tocar).
TYPE_SIZES = {"u16": 2, "u32": 4, "u64": 8, "f32": 4, "f64": 8}
PY_STRUCT_CHAR = {"u16": "H", "u32": "I", "u64": "Q", "f32": "f", "f64": "d"}
RUST_TYPE = {"u16": "u16", "u32": "u32", "u64": "u64", "f32": "f32", "f64": "f64"}
C_TYPE = {"u16": "uint16_t", "u32": "uint32_t", "u64": "uint64_t", "f32": "float", "f64": "double"}


def field_size(f: dict) -> int:
    return TYPE_SIZES[f["type"]] * f.get("count", 1)


def compute_offsets(fields: list[dict]) -> list[tuple[dict, int, int]]:
    out, off = [], 0
    for f in fields:
        sz = field_size(f)
        out.append((f, off, sz))
        off += sz
    return out


def total_size(fields: list[dict]) -> int:
    return sum(field_size(f) for f in fields)


def validate(schema: dict) -> None:
    for name, pkt in schema["packets"].items():
        actual = total_size(pkt["fields"])
        expected = pkt["expected_size"]
        if actual != expected:
            raise SystemExit(
                f"ERROR de schema en {name}: suma de campos = {actual} bytes, "
                f"expected_size declarado = {expected} bytes -- corregir el schema "
                f"antes de generar nada."
            )


def _expand_names(fields: list[dict], skip: tuple[str, ...] = ()) -> list[str]:
    names = []
    for f in fields:
        if f["name"] in skip:
            continue
        names.extend(f["elements"] if f["kind"] == "array" else [f["name"]])
    return names


# ---------------------------------------------------------------------------
# Generador Python
# ---------------------------------------------------------------------------

def gen_python(schema: dict) -> str:
    states = schema["packets"]["KUNTUR_STATES"]
    command = schema["packets"]["KUNTUR_COMMAND"]

    def struct_fmt(fields: list[dict]) -> str:
        parts = []
        for f in fields:
            ch = PY_STRUCT_CHAR[f["type"]]
            n = f.get("count", 1)
            parts.append(ch * n if f["kind"] == "array" else ch)
        return "<" + "".join(parts)

    states_fmt = struct_fmt(states["fields"])
    command_fmt = struct_fmt(command["fields"])

    unpack_names = []
    for f in states["fields"]:
        if f["name"] == "magic_header":
            unpack_names.append("_magic")
        elif f["name"] == "_padding":
            unpack_names.append("_pad")
        elif f["kind"] == "array":
            unpack_names.extend(f["elements"])
        else:
            unpack_names.append(f["name"])

    dataclass_fields = _expand_names(states["fields"], skip=("magic_header", "_padding"))
    int_fields = {"t_sim_us", "seq_num", "status"}
    dataclass_body = "\n".join(
        f"    {n}: {'int' if n in int_fields else 'float'}" for n in dataclass_fields
    )
    ctor_kwargs = "\n".join(f"        {n}={n}," for n in dataclass_fields)

    command_args = [f["name"] for f in command["fields"] if f["name"] != "magic_header"]
    command_params = ", ".join(command_args)
    command_pack_args = ", ".join(
        ["MAGIC_COMMAND"] + [f"int(exec_mode)" if a == "exec_mode" else a for a in command_args]
    )

    # --- Extension fork CADAC-SIL: direccion inversa del mismo contrato ---
    # pack_states()   : lado planta (hoy kuntur_ic.cpp/Simulink) -> KSE.
    # unpack_command(): lado planta <- KSE (hoy los OUT_* de kuntur_ic.cpp).
    # Generado desde el mismo schema, misma disciplina de ICD que el resto
    # de este archivo -- no hay literales de formato nuevos, solo la
    # direccion que faltaba para poner un proceso Python del lado planta.
    states_pack_names = _expand_names(states["fields"], skip=("magic_header",))
    states_pack_params = ", ".join(n for n in states_pack_names if n != "_padding")
    states_pack_args = ["MAGIC_STATES"] + [
        "0" if n == "_padding" else n for n in states_pack_names
    ]
    states_pack_args_str = ", ".join(states_pack_args)

    command_unpack_names = []
    for f in command["fields"]:
        command_unpack_names.append("_magic" if f["name"] == "magic_header" else f["name"])
    command_dataclass_fields = [f["name"] for f in command["fields"] if f["name"] != "magic_header"]
    command_int_fields = {"t_sim_us_grant", "exec_mode"}
    command_dataclass_body = "\n".join(
        f"    {n}: {'int' if n in command_int_fields else 'float'}" for n in command_dataclass_fields
    )
    command_ctor_kwargs = "\n".join(f"        {n}={n}," for n in command_dataclass_fields)

    return f'''"""
icd.py -- Contrato Kuntur-IC v{schema["version"]} (KUNTUR_STATES / KUNTUR_COMMAND).

*** ARCHIVO GENERADO -- NO EDITAR A MANO ***
Generado por tools/gen_icd.py desde icd_schema/kuntur_ic_v1_3.json.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from .config import ExecMode

MAGIC_STATES = {states["magic_hex"]}   # "{states["magic_str"]}"
MAGIC_COMMAND = {command["magic_hex"]}  # "{command["magic_str"]}"

STATES_FMT = "{states_fmt}"
STATES_SIZE = struct.calcsize(STATES_FMT)

COMMAND_FMT = "{command_fmt}"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

assert STATES_SIZE == {states["expected_size"]}, f"STATES_FMT error: {{STATES_SIZE}} bytes (esperados {states["expected_size"]})"
assert COMMAND_SIZE == {command["expected_size"]}, f"COMMAND_FMT error: {{COMMAND_SIZE}} bytes (esperados {command["expected_size"]})"


class ICDFrameError(Exception):
    """Paquete recibido con tamaño o magic header inválido."""


@dataclass(frozen=True)
class StatesPacket:
    """Representación tipada de un KUNTUR_STATES ya validado."""
{dataclass_body}


def unpack_states(data: bytes) -> StatesPacket:
    """Valida tamaño + magic y decodifica un KUNTUR_STATES."""
    if len(data) != STATES_SIZE:
        raise ICDFrameError(f"tamaño {{len(data)}} != {{STATES_SIZE}} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_STATES:
        raise ICDFrameError(f"magic 0x{{magic:08X}} != 0x{{MAGIC_STATES:08X}}")

    ({", ".join(unpack_names)}) = struct.unpack(STATES_FMT, data)

    return StatesPacket(
{ctor_kwargs}
    )


def pack_command({command_params}) -> bytes:
    """Empaqueta un KUNTUR_COMMAND (Grant + actuadores + terreno + modo)."""
    return struct.pack(COMMAND_FMT, {command_pack_args})


@dataclass(frozen=True)
class CommandPacket:
    """Representacion tipada de un KUNTUR_COMMAND ya validado (lado planta)."""
{command_dataclass_body}


def unpack_command(data: bytes) -> CommandPacket:
    """Valida tamano + magic y decodifica un KUNTUR_COMMAND (lado planta,
    simetrico de unpack_states() -- lo que hoy hace kuntur_ic.cpp con los
    OUT_* al recibir desde KSE)."""
    if len(data) != COMMAND_SIZE:
        raise ICDFrameError(f"tamano {{len(data)}} != {{COMMAND_SIZE}} bytes")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC_COMMAND:
        raise ICDFrameError(f"magic 0x{{magic:08X}} != 0x{{MAGIC_COMMAND:08X}}")

    ({", ".join(command_unpack_names)}) = struct.unpack(COMMAND_FMT, data)

    return CommandPacket(
{command_ctor_kwargs}
    )


def pack_states({states_pack_params}) -> bytes:
    """Empaqueta un KUNTUR_STATES (lado planta -> KSE). pos_ned viaja
    CRUDO (passthrough) -- la proyeccion WGS-84 es responsabilidad
    exclusiva de geodesy.SACOLocalFrame del lado Python de Kuntur-Bridge,
    nunca de quien genera este paquete (misma regla que hoy respeta
    kuntur_ic.cpp). accel_body debe venir ya convertido a fuerza
    especifica (f_body = A_bb - g_body) por quien llama a esta funcion --
    ver icd_helpers.specific_force_body() en el paquete cadac_plant para
    la misma transformacion que hace kuntur_ic.cpp."""
    return struct.pack(STATES_FMT, {states_pack_args_str})
'''


# ---------------------------------------------------------------------------
# Generador Python genérico -- canales auxiliares (fuera de Kuntur-IC v1.3)
# ---------------------------------------------------------------------------
#
# gen_python() arriba está deliberadamente hardcodeado a los dos paquetes
# del contrato principal (KUNTUR_STATES/KUNTUR_COMMAND, con nombres y
# direcciones fijas) -- no se toca, para no arriesgar una regresión en el
# contrato que sí genera Rust/C++. Un canal auxiliar (ver
# icd_schema/kuntur_ic_aux_v1.json) declara UN paquete arbitrario y solo
# necesita Python; esta función genera un módulo autocontenido
# (pack_<x>()/unpack_<x>() + dataclass) a partir de cualquier paquete de
# cualquier schema, sin asumir sus campos.

def gen_python_generic(schema: dict, packet_name: str) -> str:
    pkt = schema["packets"][packet_name]
    fields = pkt["fields"]
    short = (
        packet_name.split("_", 1)[1].lower()
        if packet_name.startswith("KUNTUR_")
        else packet_name.lower()
    )
    SHORT = short.upper()
    class_name = short.capitalize() + "Packet"
    magic_name = f"MAGIC_{SHORT}"
    fmt_name = f"{SHORT}_FMT"
    size_name = f"{SHORT}_SIZE"

    def struct_fmt(fields: list[dict]) -> str:
        parts = []
        for f in fields:
            ch = PY_STRUCT_CHAR[f["type"]]
            n = f.get("count", 1)
            parts.append(ch * n if f["kind"] == "array" else ch)
        return "<" + "".join(parts)

    fmt = struct_fmt(fields)

    # Arreglos "a granel": un campo "kind": "array" SIN "elements" (a
    # diferencia de los arreglos nombrados ya existentes, p.ej. SBEL ->
    # [north,east,south]) se expone como list[float]/list[int] en vez de
    # `count` campos sueltos -- necesario para arreglos grandes (p.ej. 64
    # waypoints de KUNTUR_ROUTE_REPLACE), donde nombrar cada slot sería
    # inmanejable. struct_fmt() arriba ya los cuenta bien por `count`, sin
    # cambios; lo que sigue solo ajusta las partes que nombran cada slot.
    def _is_bulk_array(f: dict) -> bool:
        return f["kind"] == "array" and "elements" not in f

    def _expand_names_generic(fields: list[dict], skip: tuple[str, ...] = ()) -> list[str]:
        names = []
        for f in fields:
            if f["name"] in skip:
                continue
            if f["kind"] == "array" and not _is_bulk_array(f):
                names.extend(f["elements"])
            else:
                names.append(f["name"])  # escalar O arreglo a granel -> un solo nombre
        return names

    unpack_names = []
    bulk_temp_names: dict[str, list[str]] = {}
    for f in fields:
        if f["name"] == "magic_header":
            unpack_names.append("_magic")
        elif f["name"] == "_padding":
            unpack_names.append("_pad")
        elif f["kind"] == "array":
            if _is_bulk_array(f):
                temps = [f"_{f['name']}_{i}" for i in range(f.get("count", 1))]
                bulk_temp_names[f["name"]] = temps
                unpack_names.extend(temps)
            else:
                unpack_names.extend(f["elements"])
        else:
            unpack_names.append(f["name"])

    dataclass_fields = _expand_names_generic(fields, skip=("magic_header", "_padding"))
    int_types = {"u16", "u32", "u64"}
    field_pytype: dict[str, str] = {}
    for f in fields:
        pt = "int" if f["type"] in int_types else "float"
        if f["kind"] == "array":
            if _is_bulk_array(f):
                field_pytype[f["name"]] = f"list[{pt}]"
            else:
                for e in f["elements"]:
                    field_pytype[e] = pt
        else:
            field_pytype[f["name"]] = pt

    dataclass_body = "\n".join(f"    {n}: {field_pytype[n]}" for n in dataclass_fields)
    ctor_kwargs = "\n".join(
        f"        {n}=[{', '.join(bulk_temp_names[n])}]," if n in bulk_temp_names
        else f"        {n}={n},"
        for n in dataclass_fields
    )
    pack_params = ", ".join(dataclass_fields)

    pack_arg_names = _expand_names_generic(fields, skip=("magic_header",))
    pack_args = [magic_name]
    pre_pack_lines: list[str] = []
    for n in pack_arg_names:
        if n == "_padding":
            pack_args.append("0")
        elif n in bulk_temp_names:
            count = len(bulk_temp_names[n])
            pad_val = "0" if field_pytype[n] == "list[int]" else "0.0"
            pre_pack_lines.append(f"    if len({n}) > {count}:")
            pre_pack_lines.append(
                f'        raise ValueError(f"{n}: maximo {count} elementos, recibidos {{len({n})}}")'
            )
            pre_pack_lines.append(f"    {n} = list({n}) + [{pad_val}] * ({count} - len({n}))")
            pack_args.append(f"*{n}")
        else:
            pack_args.append(n)
    pack_args_str = ", ".join(pack_args)

    lines: list[str] = []
    lines.append('"""')
    lines.append(f"icd_{short}.py -- Canal auxiliar {packet_name} (schema v{schema.get('version', '?')}).")
    lines.append("")
    lines.append("*** ARCHIVO GENERADO -- NO EDITAR A MANO ***")
    lines.append("Generado por tools/gen_icd.py. NO es parte del contrato Kuntur-IC v1.3")
    lines.append("(kuntur_bridge/icd.py, icd_schema/kuntur_ic_v1_3.json) -- ver el schema")
    lines.append("de origen de este archivo para transporte/decimación/propósito.")
    lines.append('"""')
    lines.append("from __future__ import annotations")
    lines.append("")
    lines.append("import struct")
    lines.append("from dataclasses import dataclass")
    lines.append("")
    lines.append(f'{magic_name} = {pkt["magic_hex"]}  # "{pkt["magic_str"]}"')
    lines.append("")
    lines.append(f'{fmt_name} = "{fmt}"')
    lines.append(f"{size_name} = struct.calcsize({fmt_name})")
    lines.append("")
    lines.append(
        f'assert {size_name} == {pkt["expected_size"]}, '
        f'f"{fmt_name} error: {{{size_name}}} bytes (esperados {pkt["expected_size"]})"'
    )
    lines.append("")
    lines.append("")
    lines.append("class ICDFrameError(Exception):")
    lines.append('    """Paquete recibido con tamaño o magic header inválido."""')
    lines.append("")
    lines.append("")
    lines.append("@dataclass(frozen=True)")
    lines.append(f"class {class_name}:")
    lines.append(f'    """Representación tipada de un {packet_name} ya validado."""')
    lines.append(dataclass_body)
    lines.append("")
    lines.append("")
    lines.append(f"def unpack_{short}(data: bytes) -> {class_name}:")
    lines.append(f'    """Valida tamaño + magic y decodifica un {packet_name}."""')
    lines.append(f"    if len(data) != {size_name}:")
    lines.append(f'        raise ICDFrameError(f"tamaño {{len(data)}} != {{{size_name}}} bytes")')
    lines.append("")
    lines.append('    magic = struct.unpack_from("<I", data, 0)[0]')
    lines.append(f"    if magic != {magic_name}:")
    lines.append(f'        raise ICDFrameError(f"magic 0x{{magic:08X}} != 0x{{{magic_name}:08X}}")')
    lines.append("")
    lines.append(f"    ({', '.join(unpack_names)}) = struct.unpack({fmt_name}, data)")
    lines.append("")
    lines.append(f"    return {class_name}(")
    lines.append(ctor_kwargs)
    lines.append("    )")
    lines.append("")
    lines.append("")
    lines.append(f"def pack_{short}({pack_params}) -> bytes:")
    lines.append(f'    """Empaqueta un {packet_name}."""')
    lines.extend(pre_pack_lines)
    lines.append(f"    return struct.pack({fmt_name}, {pack_args_str})")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Generador Rust
# ---------------------------------------------------------------------------

def gen_rust(schema: dict) -> str:
    states = schema["packets"]["KUNTUR_STATES"]
    command = schema["packets"]["KUNTUR_COMMAND"]

    struct_fields, unpack_body = [], []
    off = 0
    for f in states["fields"]:
        sz = field_size(f)
        if f["name"] == "magic_header":
            off += sz
            continue
        if f["name"] == "t_sim_us":
            struct_fields.append("    pub t_sim_us: u64,")
            unpack_body.append(f"    let t_sim_us = u64::from_le_bytes(data[{off}..{off+sz}].try_into().unwrap());")
        elif f["name"] == "seq_num":
            struct_fields.append("    pub seq_num: u32,")
            unpack_body.append(f"    let seq_num = u32::from_le_bytes(data[{off}..{off+sz}].try_into().unwrap());")
        elif f["name"] == "_padding":
            pass
        elif f["kind"] == "array":
            elem_sz = TYPE_SIZES[f["type"]]
            for i, elem in enumerate(f["elements"]):
                e_off = off + i * elem_sz
                struct_fields.append(f"    pub {elem}: {RUST_TYPE[f['type']]},")
                unpack_body.append(f"    let {elem} = f32_at({e_off}, {e_off+elem_sz});")
        elif f["name"] == "status":
            struct_fields.append("    pub status: u16,")
            unpack_body.append(f"    let status = u16::from_le_bytes(data[{off}..{off+sz}].try_into().unwrap());")
        else:
            struct_fields.append(f"    pub {f['name']}: f32,")
            unpack_body.append(f"    let {f['name']} = f32_at({off}, {off+sz});")
        off += sz

    ctor_body = "\n".join(f"        {n}," for n in _expand_names(states["fields"], skip=("magic_header", "_padding")))

    command_fields = [f for f in command["fields"] if f["name"] != "magic_header"]
    command_pack, off2 = [], 4
    for f in command_fields:
        sz = field_size(f)
        command_pack.append(f"    buf[{off2}..{off2+sz}].copy_from_slice(&{f['name']}.to_le_bytes());")
        off2 += sz
    command_args = ", ".join(f"{f['name']}: {RUST_TYPE[f['type']]}" for f in command_fields)

    seen, index_enums = set(), []
    for f in states["fields"]:
        ie = f.get("index_enum")
        if ie and ie not in seen:
            seen.add(ie)
            letters = ", ".join(f"IDX_{e.upper()}" for e in f["elements"])
            index_enums.append(f"pub enum {ie} {{ {letters} }}  // {f['name']}[3] ({f.get('unit') or '-'})")

    offsets = compute_offsets(states["fields"])
    pack_lines, assert_lines, val = [], [], 1.0
    for f, foff, fsz in offsets:
        if f["name"] in ("magic_header", "_padding"):
            continue
        if f["name"] == "t_sim_us":
            pack_lines.append(f"    buf[{foff}..{foff+fsz}].copy_from_slice(&123_456u64.to_le_bytes());")
            assert_lines.append("    assert_eq!(sp.t_sim_us, 123_456);")
        elif f["name"] == "seq_num":
            pack_lines.append(f"    buf[{foff}..{foff+fsz}].copy_from_slice(&7u32.to_le_bytes());")
            assert_lines.append("    assert_eq!(sp.seq_num, 7);")
        elif f["name"] == "status":
            pack_lines.append(f"    buf[{foff}..{foff+fsz}].copy_from_slice(&0u16.to_le_bytes());")
        elif f["kind"] == "array":
            elem_sz = TYPE_SIZES[f["type"]]
            for i, elem in enumerate(f["elements"]):
                v = round(val, 3); val += 1.1
                e_off = foff + i * elem_sz
                pack_lines.append(f"    buf[{e_off}..{e_off+elem_sz}].copy_from_slice(&{v}f32.to_le_bytes());")
                assert_lines.append(f"    assert!((sp.{elem} - {v}f32).abs() < 1e-4);")
        else:
            v = round(val, 3); val += 1.1
            pack_lines.append(f"    buf[{foff}..{foff+fsz}].copy_from_slice(&{v}f32.to_le_bytes());")
            assert_lines.append(f"    assert!((sp.{f['name']} - {v}f32).abs() < 1e-4);")

    states_test = f'''
    #[test]
    fn states_round_trip() {{
        let mut buf = [0u8; STATES_SIZE];
        buf[0..4].copy_from_slice(&MAGIC_STATES.to_le_bytes());
{chr(10).join(pack_lines)}
        let sp = unpack_states(&buf).expect("debe decodificar");
{chr(10).join(assert_lines)}
    }}'''

    return f'''//! icd.rs -- Contrato Kuntur-IC v{schema["version"]} en Rust.
//!
//! *** ARCHIVO GENERADO -- NO EDITAR A MANO ***
//! Generado por tools/gen_icd.py desde icd_schema/kuntur_ic_v1_3.json.
//! Sin `#[repr(C, packed)]` ni transmute -- empaquetado explícito campo a
//! campo, igual que icd.py con struct.pack/unpack.

pub const MAGIC_STATES: u32 = {states["magic_hex"]};
pub const MAGIC_COMMAND: u32 = {command["magic_hex"]};

pub const STATES_SIZE: usize = {states["expected_size"]};
pub const COMMAND_SIZE: usize = {command["expected_size"]};

const _: () = assert!(STATES_SIZE == {states["expected_size"]});
const _: () = assert!(COMMAND_SIZE == {command["expected_size"]});

#[derive(Debug, Clone, Copy, Default)]
pub struct StatesPacket {{
{chr(10).join(struct_fields)}
}}

#[derive(Debug)]
pub enum IcdError {{
    BadSize(usize),
    BadMagic(u32),
}}

impl std::fmt::Display for IcdError {{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {{
        match self {{
            IcdError::BadSize(n) => write!(f, "tamaño {{n}} != {{STATES_SIZE}} bytes"),
            IcdError::BadMagic(m) => write!(f, "magic 0x{{m:08X}} != 0x{{MAGIC_STATES:08X}}"),
        }}
    }}
}}

pub fn unpack_states(data: &[u8]) -> Result<StatesPacket, IcdError> {{
    if data.len() != STATES_SIZE {{
        return Err(IcdError::BadSize(data.len()));
    }}
    let magic = u32::from_le_bytes(data[0..4].try_into().unwrap());
    if magic != MAGIC_STATES {{
        return Err(IcdError::BadMagic(magic));
    }}

    let f32_at = |a: usize, b: usize| f32::from_le_bytes(data[a..b].try_into().unwrap());

{chr(10).join(unpack_body)}

    Ok(StatesPacket {{
{ctor_body}
    }})
}}

#[allow(clippy::too_many_arguments)]
pub fn pack_command({command_args}) -> [u8; COMMAND_SIZE] {{
    let mut buf = [0u8; COMMAND_SIZE];
    buf[0..4].copy_from_slice(&MAGIC_COMMAND.to_le_bytes());
{chr(10).join(command_pack)}
    buf
}}

#[cfg(test)]
mod tests {{
    use super::*;

    #[test]
    fn command_round_trip_size() {{
        let buf = pack_command(123_456, 0.1, -0.2, 0.0, 0.5, 589.0, 0);
        assert_eq!(buf.len(), COMMAND_SIZE);
    }}

    #[test]
    fn rejects_bad_size() {{
        let buf = [0u8; 10];
        assert!(matches!(unpack_states(&buf), Err(IcdError::BadSize(10))));
    }}

    #[test]
    fn rejects_bad_magic() {{
        let buf = [0u8; STATES_SIZE];
        assert!(matches!(unpack_states(&buf), Err(IcdError::BadMagic(0))));
    }}
{states_test}
}}
'''


# ---------------------------------------------------------------------------
# Generador C++
# ---------------------------------------------------------------------------

def gen_cpp(schema: dict) -> str:
    states = schema["packets"]["KUNTUR_STATES"]
    command = schema["packets"]["KUNTUR_COMMAND"]

    def struct_body(fields: list[dict]) -> str:
        lines = []
        for f in fields:
            ctype = C_TYPE[f["type"]]
            if f["kind"] == "array":
                lines.append(f"    {ctype} {f['name']}[{f['count']}];  // [{', '.join(f['elements'])}] ({f.get('unit') or '-'})")
            else:
                note = f.get("note", "")
                lines.append(f"    {ctype} {f['name']};  // {f.get('unit') or '-'} {('-- ' + note) if note else ''}".rstrip())
        return "\n".join(lines)

    seen, index_enums = set(), []
    for f in states["fields"]:
        ie = f.get("index_enum")
        if ie and ie not in seen:
            seen.add(ie)
            letters = ", ".join(f"ICD_{e.upper()}" for e in f["elements"])
            index_enums.append(f"enum {ie} {{ {letters} }};  // {f['name']}[3] ({f.get('unit') or '-'})")

    return f'''// ============================================================================
// kuntur_ic_generated.h -- Contrato Kuntur-IC v{schema["version"]}, structs binarios C++
// *** ARCHIVO GENERADO -- NO EDITAR A MANO ***
// Generado por tools/gen_icd.py desde icd_schema/kuntur_ic_v1_3.json.
// ============================================================================
#pragma once

#include <cstdint>

#pragma pack(push, 1)

struct KunturStatesV13 {{
{struct_body(states["fields"])}
}}; // {states["expected_size"]} bytes

struct KunturCommandV12 {{
{struct_body(command["fields"])}
}}; // {command["expected_size"]} bytes

#pragma pack(pop)

static_assert(sizeof(KunturStatesV13) == {states["expected_size"]}, "KunturStatesV13 debe pesar {states["expected_size"]} bytes");
static_assert(sizeof(KunturCommandV12) == {command["expected_size"]}, "KunturCommandV12 debe pesar {command["expected_size"]} bytes");

#define MAGIC_STATES_V13  {states["magic_hex"]}u
#define MAGIC_COMMAND_V12 {command["magic_hex"]}u

// Índices de las señales vectoriales -- evita literales mágicos [0][1][2]
{chr(10).join(index_enums)}
'''


def _packet_short_name(packet_name: str) -> str:
    return (
        packet_name.split("_", 1)[1].lower()
        if packet_name.startswith("KUNTUR_")
        else packet_name.lower()
    )


def main() -> None:
    schema_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SCHEMA_PATH
    schema = json.loads(schema_path.read_text())
    validate(schema)

    is_main_contract = schema_path.resolve() == DEFAULT_SCHEMA_PATH.resolve()
    codegen = schema.get("codegen")

    if not is_main_contract or codegen is not None:
        # Canal(es) auxiliar(es), fuera del contrato Kuntur-IC v1.3 -- esta
        # rama NUNCA toca icd.rs/kuntur_ic_generated.h (esos se regeneran
        # exclusivamente desde kuntur_ic_v1_3.json, arriba). Por default,
        # cualquier schema que no sea el contrato principal se asume
        # "solo Python"; el bloque opcional 'codegen' del schema puede
        # hacerlo explícito.
        want_python = codegen.get("python", True) if codegen else True
        want_rust = codegen.get("rust", False) if codegen else False
        if want_rust:
            raise SystemExit(
                f"{schema_path}: codegen.rust=true, pero la generación Rust está "
                f"reservada al contrato principal ({DEFAULT_SCHEMA_PATH.name}). "
                "Un canal auxiliar no genera binding Rust -- mantiene a kse_server "
                "acotado a su rol de broker de Kuntur-IC v1.3."
            )
        if not want_python:
            raise SystemExit(f"{schema_path}: codegen.python=false -- nada que generar.")

        for packet_name in schema["packets"]:
            short = _packet_short_name(packet_name)
            out_path = ROOT / "kuntur_bridge" / f"icd_{short}.py"
            out_path.write_text(gen_python_generic(schema, packet_name))
            print(f"generado: {out_path}")

        print("Validación de tamaños OK:",
              {name: pkt["expected_size"] for name, pkt in schema["packets"].items()})
        return

    # --- Contrato principal Kuntur-IC v1.3: Python + Rust + C++ ---
    py_out = ROOT / "kuntur_bridge" / "icd.py"
    rs_out = ROOT / "kse_server" / "src" / "icd.rs"
    cpp_out = ROOT / "kuntur_ic_generated.h"

    py_out.write_text(gen_python(schema))
    print(f"generado: {py_out}")
    rs_out.write_text(gen_rust(schema))
    print(f"generado: {rs_out}")
    cpp_out.write_text(gen_cpp(schema))
    print(f"generado: {cpp_out}")

    print("Validación de tamaños OK:",
          {name: pkt["expected_size"] for name, pkt in schema["packets"].items()})


if __name__ == "__main__":
    main()

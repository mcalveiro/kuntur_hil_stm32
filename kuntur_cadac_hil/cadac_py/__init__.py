"""cadac_py -- Python port of Peter H. Zipfel's CADAC++ simulation framework.

Ported from https://github.com/missiondesignsolutions/CADAC (C++), starting
with the SRAAM6 (air-to-air 6-DoF short range missile) simulation.

Package layout mirrors the original source files as closely as possible:

    matrix.py     <- utility_header.hpp (class Matrix) + free functions
    integrate.py  <- utility_header.hpp (integrate(), atmosphere76())
    core.py       <- global_header.hpp (Module, Variable, Event, Packet, Document)
    tables.py     <- class_hierarchy.hpp/utility_functions.cpp (Table, Datadeck)
    cadac.py      <- class_hierarchy.hpp (Cadac, Flat6, Flat3 base classes)
    ... (vehicle-specific modules follow, one file per original .cpp)
"""

"""
aux_instrument_worker.py -- Hilo receptor del canal ICD auxiliar de
instrumentación KUNTUR_AUX (ver icd_schema/kuntur_ic_aux_v1.json).

Este canal NO pasa por KSE/kse_server -- plant_driver.py lo envía
DIRECTO por UDP a este listener (puerto 7000 por default), decimado a
25 Hz desde su paso base de 1 kHz. No tiene relación con el rendezvous
KSE<->FCS ni con el snapshot que consume flightgear_worker.py: es un
segundo flujo, independiente, que solo escribe al Property Tree de
FlightGear vía Telnet (ver telnet_client.apply_aux_instrumentation()).

La sesión Telnet que usa este hilo es DEDICADA (ver
telnet_client.connect_property_tree()), NO la que ya usa
flightgear_worker.py para calibración de origen/polling de terreno --
FGPropertyTree no es thread-safe (un único socket síncrono, sin locks),
así que compartir esa conexión entre dos hilos podría entrelazar
respuestas de get()/set() de ambos.
"""
from __future__ import annotations

import socket
import threading

from .config import FG_IP, FG_TELNET_PORT, KUNTUR_AUX_PORT
from .icd_aux import ICDFrameError, AUX_SIZE, unpack_aux
from . import logging_setup
from .telnet_client import apply_aux_instrumentation, connect_property_tree

log = logging_setup.get_logger("aux_instrument")


def aux_instrument_worker(
    fg_ip: str = FG_IP,
    fg_telnet_port: int = FG_TELNET_PORT,
    aux_port: int = KUNTUR_AUX_PORT,
    stop_event: threading.Event | None = None,
) -> None:
    if stop_event is None:
        stop_event = threading.Event()

    prop_tree = connect_property_tree(fg_ip, fg_telnet_port)
    if prop_tree is None:
        log.warning(
            "sin conexión Telnet dedicada -- canal ICD auxiliar de instrumentación "
            "deshabilitado (sonido de motor e instrumentos básicos no se van a actualizar)"
        )
        return

    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("0.0.0.0", aux_port))
    rx_sock.settimeout(0.5)  # despierta periódicamente para revisar stop_event

    log.info(
        f"escuchando canal ICD auxiliar KUNTUR_AUX en UDP:{aux_port}",
        extra={"aux_port": aux_port},
    )

    n_ok = 0
    n_err = 0
    try:
        while not stop_event.is_set():
            try:
                data, _ = rx_sock.recvfrom(AUX_SIZE + 32)
            except socket.timeout:
                continue
            except OSError as e:
                log.warning(
                    f"error de socket en canal ICD auxiliar ({e}) -- terminando el hilo",
                    extra={"error": str(e)},
                )
                return

            try:
                aux = unpack_aux(data)
            except ICDFrameError as e:
                n_err += 1
                if n_err % 100 == 1:
                    log.warning(
                        f"paquete KUNTUR_AUX inválido descartado: {e} (van {n_err})",
                        extra={"error": str(e), "n_err": n_err},
                    )
                continue

            try:
                apply_aux_instrumentation(prop_tree, aux)
                n_ok += 1
                if n_ok % 250 == 0:  # ~cada 10s a 25 Hz
                    log.debug(
                        f"seq_num={aux.seq_num} power={aux.power:.1f}% dvbe={aux.dvbe:.1f}m/s "
                        f"hbe={aux.hbe:.1f}m anx={aux.anx:.2f}g",
                        extra={
                            "seq_num": aux.seq_num, "power": aux.power,
                            "dvbe": aux.dvbe, "hbe": aux.hbe, "anx": aux.anx,
                        },
                    )
            except Exception as e:
                log.error(
                    f"error aplicando instrumentación auxiliar: {e}",
                    extra={"error": str(e)},
                )
    finally:
        rx_sock.close()
        log.info(
            "señal de apagado recibida -- hilo de instrumentación auxiliar terminado limpiamente "
            f"({n_ok} paquetes aplicados, {n_err} descartados)",
            extra={"n_ok": n_ok, "n_err": n_err},
        )

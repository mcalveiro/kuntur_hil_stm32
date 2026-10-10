"""
flightgear_worker.py -- Hilo de visualización FlightGear (~30 Hz).

FIX CRÍTICO: ICD v1.3 entrega phi/theta/psi en RADIANES -- este módulo
YA NO llama math.radians() sobre ellos (antes compensaba la convención
empírica "ago_18", ahora derogada). Depende exclusivamente de KSEClient.
"""
from __future__ import annotations

import math
import socket
import threading
import time

from .config import FG_IP, FG_UDP_PORT, FG_FDM_RATE_HZ, FG_TERRAIN_POLL_EVERY_N
from .fgnetfdm import pack_fg_net_fdm
from .geodesy import SACOLocalFrame
from .kse_client import KSEClient
from . import logging_setup
from .telnet_client import FGPropertyTree

log = logging_setup.get_logger("flightgear")


def flightgear_worker(
    runtime: KSEClient,
    local_frame: SACOLocalFrame,
    prop_tree: FGPropertyTree | None,
    fg_ip: str = FG_IP,
    fg_udp_port: int = FG_UDP_PORT,
    fdm_rate_hz: float = FG_FDM_RATE_HZ,
    terrain_poll_every_n: int = FG_TERRAIN_POLL_EVERY_N,
    stop_event: threading.Event | None = None,
) -> None:
    if stop_event is None:
        stop_event = threading.Event()

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    terrain_enabled = prop_tree is not None

    log.info("aeronave posicionada, esperando Executive")
    ready = False
    while not stop_event.is_set():
        try:
            if runtime.ready_event().wait(timeout=0.5):
                ready = True
                break
        except OSError:
            log.warning("conexión con KSE perdida antes de estar listo -- terminando el hilo")
            return
    if not ready:
        log.info("señal de apagado recibida antes de que KSE esté listo -- terminando el hilo")
        return

    if prop_tree:
        ok = prop_tree.set("/sim/freeze/master", 0)
        if not ok:
            log.warning("no se pudo despausar por Telnet -- presionar 'p' en FlightGear manualmente")
    else:
        log.warning("sin Telnet -- presionar 'p' en FlightGear manualmente para despausar")

    log.info(f"transmitiendo FDM a UDP:{fg_udp_port} ({fdm_rate_hz:.0f} Hz)", extra={"fg_udp_port": fg_udp_port, "fdm_rate_hz": fdm_rate_hz})
    interval = 1.0 / fdm_rate_hz
    counter = 0

    while not stop_event.is_set():
        try:
            t0 = time.time()

            snap = runtime.snapshot()
            if snap is None:
                stop_event.wait(timeout=interval)
                continue

            # phi/theta/psi YA en radianes (ICD v1.3) -- sin conversión.
            phi_rad, theta_rad, psi_rad = snap.phi, snap.theta, snap.psi

            lat_rad, lon_rad, alt_m = local_frame.ned_to_geodetic(snap.north, snap.east, snap.down)

            counter += 1
            ground_elev_m = 0.0
            if counter % terrain_poll_every_n == 0 and terrain_enabled:
                ground_elev_m = prop_tree.get("/position/ground-elev-ft") * 0.3048
                runtime.set_ground_elevation(ground_elev_m)

            agl_m = alt_m - ground_elev_m

            packet = pack_fg_net_fdm(
                lon_rad, lat_rad, alt_m, phi_rad, theta_rad, psi_rad,
                elev=0.0, ail=0.0, rud=0.0, throt=0.0,
            )
            udp_sock.sendto(packet, (fg_ip, fg_udp_port))

            if counter % 50 == 0:
                log.debug(
                    f"phi={math.degrees(snap.phi):.1f} theta={math.degrees(snap.theta):.1f} "
                    f"psi={math.degrees(snap.psi):.1f} MSL={alt_m:.1f}m AGL={agl_m:.1f}m",
                    extra={
                        "seq_num": snap.seq_num, "phi_deg": math.degrees(snap.phi),
                        "theta_deg": math.degrees(snap.theta), "psi_deg": math.degrees(snap.psi),
                        "alt_msl_m": alt_m, "alt_agl_m": agl_m,
                        "north": snap.north, "down": snap.down,
                    },
                )

            elapsed = time.time() - t0
            if elapsed < interval:
                stop_event.wait(timeout=interval - elapsed)

        except OSError as e:
            log.warning(f"conexión con KSE perdida ({e}) -- terminando el hilo", extra={"error": str(e)})
            return
        except Exception as e:
            log.error(f"error en loop: {e}", extra={"error": str(e)})
            stop_event.wait(timeout=1.0)

    log.info("señal de apagado recibida -- hilo terminado limpiamente")

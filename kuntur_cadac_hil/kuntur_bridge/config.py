"""
config.py -- Configuración de red y parámetros fijos del Kuntur-Bridge.

Fork HIL (kuntur_cadac_hil): sin KUNTUR_CTL_PORT -- el canal de control
Kuntur-CTL (retasking/reemplazo de ruta) se retira del backend en esta
versión (ver plan-hil-stm32-portado-autopiloto.md §9/§14). En su lugar,
KUNTUR_MODE_PORT es un canal nuevo y deliberadamente mínimo: SOLO lleva
el flag de modo Automático/Manual desde gcs_api_worker.py (kuntur_bridge)
hasta plant_driver.py (ModeLink, cadac_plant/protocol.py) -- no retasking,
no waypoints. Ver icd_schema/kuntur_ic_mode_v1.json.
"""
from enum import IntEnum


class ExecMode(IntEnum):
    """Modo de acoplamiento temporal entre KSE y Simulink (DA-14)."""
    ENGINE_DRIVEN = 0
    AUTHORITY_DRIVEN = 1


# --- Red: FlightGear ---
FG_IP = "127.0.0.1"
FG_UDP_PORT = 5501
FG_TELNET_PORT = 5000
FG_FDM_RATE_HZ = 30.0
FG_TERRAIN_POLL_EVERY_N = 10

# --- Red: Simulink (Kuntur-IC v1.3) ---
SIMULINK_IP = "127.0.0.1"
SIMULINK_TELEMETRY_PORT = 6000   # RX KUNTUR_STATES (KSE escucha)
SIMULINK_COMMAND_PORT = 6001     # TX KUNTUR_COMMAND (KSE envía)

# --- Red: canal ICD auxiliar de instrumentación (Kuntur-AUX v1) ---
# NO es parte de Kuntur-IC v1.3 -- no pasa por kse_server. Transporte
# directo plant_driver.py -> kuntur_bridge (ver
# icd_schema/kuntur_ic_aux_v1.json y aux_instrument_worker.py).
KUNTUR_AUX_PORT = 7000

# --- Red: canal de modo Kuntur-MODE v1 (Automático/Manual) ---
# NO es parte de Kuntur-IC v1.3 -- no pasa por kse_server. TCP, loopback:
# gcs_api_worker.py (cliente) <-> plant_driver.py (servidor, ModeLink en
# cadac_plant/protocol.py). Ver icd_schema/kuntur_ic_mode_v1.json.
KUNTUR_MODE_PORT = 6200

# --- Origen de fallback (SACO) ---
FALLBACK_LAT = -31.31
FALLBACK_LON = -64.21
FALLBACK_ALT = 589.0

# --- Modo de ejecución por defecto de KSE ---
DEFAULT_EXEC_MODE = ExecMode.ENGINE_DRIVEN

# --- Rendezvous KSE <-> FCS/PX4-Bridge (legado -- sin componente activo
# que lo ejercite en este fork, se conserva porque PlantLink/kse_client
# siguen exponiendo el parámetro) ---
FCS_RENDEZVOUS_TIMEOUT_S = 0.003
FCS_STALE_WARNING_EVERY_N_CYCLES = 10

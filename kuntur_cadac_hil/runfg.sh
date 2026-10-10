#!/bin/bash
# ==============================================================================
# runfg.sh - Arranque de FlightGear para KUNTUR-Bridge V1.2
#
# Selector de aeronave: KUNTUR_AIRCRAFT=757|c172p|rascal|f16|mq9 (default: mq9)
#
# mq9 confirmado como el perfil final para este fork (prueba de humo
# real, ver claude/plan-instrumentacion-canal-auxiliar-icd.md): sonido
# de motor funcionando via KUNTUR_AUX, sin el crash estructural de Nasal
# que descarto al paquete f16 de FGAddon, y ademas es tematicamente el
# correcto -- Kuntur-SIM es GNC para UAVs de ala fija, no cazas
# tripulados. f16 queda documentado pero no recomendado (ver el caso
# f16 mas abajo).
# Elegí según qué planta vas a alimentar por KUNTUR_STATES -- ninguna de
# las cuatro afecta la física (--fdm=null: la dinámica siempre viene de
# quien mande KUNTUR_STATES -- Simulink, o el fork CADAC-SIL/FALCON6).
# Es puramente el modelo 3D que se renderiza.
#
# f16 (agregado para el fork CADAC-SIL, cadac_plant/plane_adapter.py::
# RealFalcon6Adapter): es el único perfil con relación temática real con
# la planta -- FALCON6 (cadac_py) modela un F-16 (f16_aero_deck.asc/
# f16_prop_deck.asc). Los otros tres siguen siendo puramente decorativos.
#
# Compatibilidad con el ecosistema V1.2 (verificado contra
# kuntur_bridge/adapters/flightgear_adapter.py y contra la wiki oficial de
# FlightGear -- ver conversación anterior para el detalle de cada fix):
#   - FDM_PORT/tasa alineados a fdm_udp_port/update_rate_hz (5501 @ 50 Hz)
#   - --telnet agregado (puerto 5000, telnet_port) -- requerido por RF-01
#   - --enable-freeze (NO "--freeze", que no existe como flag) -- RF-01
# ==============================================================================

# --- 1. Limpiar instancias previas (evita el error de lock.pid) ---
killall -9 fgfs 2>/dev/null

# --- 2. Selector de aeronave ---
KUNTUR_AIRCRAFT="${KUNTUR_AIRCRAFT:-mq9}"   # 757 | c172p | rascal | f16 | mq9

# Propiedades extra a precargar vía --prop: (por perfil -- ver caso "f16"
# más abajo). Default vacío; cada case la puede sobrescribir.
FGFS_EXTRA_PROPS=()

export FG_ROOT="${FG_ROOT:-/usr/share/games/flightgear}"
FGADDON_DIR="$HOME/.fgfs/Aircraft/org.flightgear.fgaddon.stable_2020"

case "$KUNTUR_AIRCRAFT" in
  757)
    # Confirmado funcional en este equipo.
    AIRCRAFT="757-200-RB211"
    FG_AIRCRAFT_DIR="${FG_AIRCRAFT_DIR:-$FGADDON_DIR}"
    ;;
  c172p)
    # Viene con la instalación base de FlightGear ($FG_ROOT/Aircraft) --
    # no necesita --fg-aircraft propio. Se deja FG_AIRCRAFT_DIR apuntando
    # al catálogo FGAddon igual, es inofensivo: fgfs busca en ambos.
    AIRCRAFT="c172p"
    FG_AIRCRAFT_DIR="${FG_AIRCRAFT_DIR:-$FGADDON_DIR}"
    ;;
  rascal)
    # Todavía sin instalar (aviso tuyo). Tres fuentes reales verificadas
    # por búsqueda -- ninguna probada de punta a punta por mí:
    #   1) FGAddon oficial (puede que YA esté en tu $FGADDON_DIR --
    #      revisar antes de clonar nada: find "$FGADDON_DIR" -iname "*rascal*")
    #      https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/Rascal/
    #   2) ThunderFly-aerospace/FlightGear-Rascal (fork mantenido,
    #      optimizado para UAV -- sin cockpit/peso de piloto):
    #      https://github.com/ThunderFly-aerospace/FlightGear-Rascal
    #   3) FGMEMBERS/Rascal (mirror):
    #      https://github.com/FGMEMBERS/Rascal
    # Regla general para --fg-aircraft: el path debe ser el directorio
    # PADRE de la carpeta que contiene Rascal110-JSBSim-set.xml -- verificar
    # la estructura real después de clonar, varía según la fuente.
    AIRCRAFT="Rascal110-JSBSim"
    FG_AIRCRAFT_DIR="${FG_AIRCRAFT_DIR:-$HOME/Documentos/eng-projects/kuntur-project/Aircraft-Models}"
    ;;
  f16)
    # Confirmado disponible en tu instalación (FlightGear 2020.3.6):
    # es el paquete Zaretto/f16 (github.com/Zaretto/f16, v3.55,
    # documentado como compatible justo con FG 2020.3.x). Mapeo real
    # variante -> id (verificado contra los -set.xml del repo, no
    # adivinado):
    #   YF-16 Prototype        -> yf16
    #   F-16A Block 10         -> f16b10
    #   F-16AM Block 20 (MLU)  -> f16b20
    #   F-16C Block 30         -> f16b30
    #   F-16C Block 32         -> f16b32
    #   F-16CM Block 40 (CCIP) -> f16b40
    #   F-16CM Block 42 (CCIP) -> f16b42
    #   F-16CJ Block 50        -> f16b50
    #   F-16CJ Block 52        -> f16b52   (default del paquete)
    #   F-16E Block 60         -> f16b60
    #   F-16 (Simplified FDM)  -> f16-simplified
    #
    # FALCON6 (cadac_py/falcon6.py) documenta su propio dataset como
    # "F16-like" genérico -- sin número de bloque en ningún lado del
    # código fuente ni de los *_deck.asc -- así que ningún bloque "matchea"
    # la física mejor que otro; la elección acá es puramente visual/
    # práctica. Default recomendado: f16-simplified, no por parecido
    # aerodinámico (ninguno lo tiene) sino porque un FDM interno más
    # simple probablemente arrastra menos efectos Nasal (tren/flaps/motor)
    # atados a variables que --fdm=null deja de actualizar -- razonamiento,
    # no verificado en la práctica. Si preferís el clásico F-16 monoplaza
    # más detallado, f16b32 es la pinta "genérica" más reconocible.
    #
    # OJO con FG_AIRCRAFT_DIR: Zaretto/f16 NO es del catálogo FGAddon
    # (no va a estar necesariamente bajo $FGADDON_DIR aunque el launcher
    # de FlightGear lo ofrezca instalar desde la misma pantalla) -- el
    # launcher lo baja a SU PROPIO catálogo, cuyo nombre de carpeta no
    # conozco de antemano. Antes de correr esto, instalá la variante
    # elegida desde el launcher y ubicá dónde quedó:
    #   find "$HOME/.fgfs/Aircraft" -iname "f16-simplified-set.xml"
    # y pasá ese directorio PADRE (no el que contiene el -set.xml, sino
    # su padre) como FG_AIRCRAFT_DIR, p.ej.:
    #   FG_AIRCRAFT_DIR=$HOME/.fgfs/Aircraft/<lo-que-encuentre-el-find> \
    #     KUNTUR_AIRCRAFT=f16 ./runfg.sh
    AIRCRAFT="${F16_AIRCRAFT_ID:-f16-simplified}"
    FG_AIRCRAFT_DIR="${FG_AIRCRAFT_DIR:-$FGADDON_DIR}"   # placeholder -- ver arriba, probablemente hay que sobrescribirlo

    # WORKAROUND (a verificar, no garantizado -- ver
    # claude/plan-instrumentacion-canal-auxiliar-icd.md, sección "Prueba
    # de humo (equipo real)"): bajo --fdm=null JSBSim nunca se instancia,
    # así que el arbol /fdm/jsbsim/* y sus alias derivados (como
    # gear/gear/position-norm, SIN indice) no existen como nodo --
    # getprop() devuelve nil, no 0. El Nasal de sistemas de este paquete
    # (f16.nas/fuel.nas, compartido por todas las variantes de bloque y
    # por f16-simplified) no chequea nil antes de usarlos en una
    # comparacion numerica y crashea en la inicializacion (confirmado:
    # mismo crash exacto, misma linea, en f16-simplified Y f16-block-52).
    # Acá se precargan los puntos de falla conocidos hasta ahora -- si
    # aparece un próximo "nil used in numeric context" o "non-objects
    # have no members" en la consola de fgfs, hay que agregar esa
    # property a esta lista y volver a probar. No hay garantía de que
    # esto alcance por sí solo (ver plan, "Prueba de humo" -- workaround
    # no exhaustivamente auditado).
    #
    # 2) fuel.nas linea 80: props.globals.getNode("fdm/jsbsim/propulsion/
    #    fuel_dump").clearValue() -- mismo problema, nodo JSBSim ausente.
    # 3) f16.nas linea 824 (cockpit_temperature_control): formula de ram
    #    rise sobre fdm/jsbsim/velocities/vtrue-kts.
    # 4) f16.nas linea 830, misma funcion, unas lineas mas abajo -- se
    #    agrega preventivamente antes de que crashee ahi (ya visible en
    #    el fragmento revisado, no hace falta esperar el proximo run).
    #
    # ABANDONADO tras esta sesión: con estas 4 propiedades precargadas
    # se destrabaron 3 puntos de falla distintos en cascada (cada uno
    # revelaba el siguiente paso de la secuencia de init() del paquete),
    # pero el motor seguía sin sonar al llegar a la prueba completa con
    # plant_driver.py -- señal de que hay más puntos de falla más
    # adelante en el árbol de inicialización (probablemente en la propia
    # cadena de arranque de motor/sonido, no solo en AC/combustible/tren
    # como los 4 ya encontrados). Se decidió no seguir auditando este
    # Nasal de terceros -- ver "Qué traer a la próxima sesión" en
    # claude/plan-instrumentacion-canal-auxiliar-icd.md. Perfil en uso:
    # 757 (ver caso "757" arriba).
    FGFS_EXTRA_PROPS=(
      "--prop:/gear/gear/position-norm=0"
      "--prop:/fdm/jsbsim/propulsion/fuel_dump=false"
      "--prop:/fdm/jsbsim/velocities/vtrue-kts=0"
      "--prop:/fdm/jsbsim/elec/bus/emergency-dc-1=0"
    )
    ;;
  mq9)
    # General Atomics MQ-9 Reaper ("bombable"), instalado en el equipo --
    # confirmado bajo el mismo catalogo FGAddon que 757/c172p (no hace
    # falta FG_AIRCRAFT_DIR propio, a diferencia del f16):
    #   find "$HOME/.fgfs/Aircraft" -iname "MQ-9*-set.xml"
    #   -> .../org.flightgear.fgaddon.stable_2020/Aircraft/MQ-9/MQ-9-JSBSim-set.xml
    #   -> .../org.flightgear.fgaddon.stable_2020/Aircraft/MQ-9/MQ-9-JSBSim2-set.xml
    #
    # Es JSBSim + framework "Bombable" (danio/armamento), igual que el
    # paquete f16 -- pero a diferencia de aquel, CONFIRMADO en prueba de
    # humo real que NO tiene el crash estructural de Nasal (ningun "nil
    # used in numeric context"/"non-objects have no members" durante el
    # arranque). Motor y sonido funcionando via KUNTUR_AUX. Perfil final
    # elegido para este fork.
    #
    # Warnings cosmeticos observados, no bloqueantes (pendientes si se
    # quiere pulir mas adelante, no urgente):
    #   - Sounds/Rumble.wav no encontrado (falta un efecto de sonido puntual)
    #   - Animacion 'BladesBlur' no encontrada en Models/MQ-9.xml (rotor/helice)
    #   - 'radardist' indefinido en Instruments-3d/rwr/rwr.nas y radar2.nas
    #     (instrumentos 3D genericos de FlightGear, no del paquete MQ-9)
    AIRCRAFT="${MQ9_AIRCRAFT_ID:-MQ-9-JSBSim}"
    FG_AIRCRAFT_DIR="${FG_AIRCRAFT_DIR:-$FGADDON_DIR}"
    FGFS_EXTRA_PROPS=()
    ;;
  *)
    echo "ERROR: KUNTUR_AIRCRAFT='$KUNTUR_AIRCRAFT' no reconocido (usar 757 | c172p | rascal | f16 | mq9)"
    exit 1
    ;;
esac

# --- 3. Puertos -- deben coincidir con FlightGearAdapterConfig ---
FDM_PORT="${FDM_PORT:-5501}"        # fdm_udp_port
FDM_RATE_HZ="${FDM_RATE_HZ:-30}"    # update_rate_hz
TELNET_PORT="${TELNET_PORT:-5000}"  # telnet_port

# --- 4. Posición inicial ---
LAT="${LAT_DEG:--31.31}"
LON="${LON_DEG:--64.21}"
ALT_FT="${ALT_FT:-2500}"
HEADING="${HEADING_DEG:-0}"

if [ ! -d "$FG_AIRCRAFT_DIR" ]; then
    echo "ADVERTENCIA: FG_AIRCRAFT_DIR ($FG_AIRCRAFT_DIR) no existe en este equipo."
    echo "  FlightGear puede fallar al no encontrar '$AIRCRAFT'."
fi

echo "=================================================================="
echo "Iniciando FlightGear (KUNTUR-Bridge V1.2)"
echo "  Perfil:   KUNTUR_AIRCRAFT=$KUNTUR_AIRCRAFT"
echo "  Aeronave: $AIRCRAFT  (catalogo: $FG_AIRCRAFT_DIR)"
echo "  FDM UDP (Python->FlightGear): puerto $FDM_PORT @ ${FDM_RATE_HZ} Hz"
echo "  Telnet (Property Tree):       puerto $TELNET_PORT"
echo "  Origen: lat=$LAT lon=$LON alt=${ALT_FT}ft heading=$HEADING (--in-air)"
echo "  Arranca en PAUSA (--enable-freeze) -- despausa automatica al primer"
echo "  paquete KUNTUR_STATES valido (spec.md, RF-01)"
echo "=================================================================="

fgfs \
  --fg-root="$FG_ROOT" \
  --fg-aircraft="$FG_AIRCRAFT_DIR" \
  --aircraft="$AIRCRAFT" \
  --fdm=null \
  --native-fdm=socket,in,${FDM_RATE_HZ},localhost,${FDM_PORT},udp \
  --telnet=socket,bi,50,127.0.0.1,${TELNET_PORT},tcp \
  --httpd=8080 \
  --enable-freeze \
  --lat=${LAT} \
  --lon=${LON} \
  --altitude=${ALT_FT} \
  --heading=${HEADING} \
  --roll=0 \
  --pitch=0 \
  --enable-terrasync \
  --prop:/sim/rendering/shaders/quality-level=0 \
  --fog-fastest \
  --disable-clouds \
  --in-air \
  "${FGFS_EXTRA_PROPS[@]}"


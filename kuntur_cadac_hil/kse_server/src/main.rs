//! main.rs -- kse_server: KSE (Kuntur Simulation Executive) como proceso
//! completamente separado del proceso Python de kuntur_bridge. Es la
//! única implementación de KSE en el proyecto.
//!
//! Manejo de señales (SIGINT/SIGTERM): sin esto, la disposición por
//! defecto de estas señales termina el proceso sin limpiar el socket
//! Unix -- bug real, reproducido y corregido en una sesión anterior.
//! `unlink()` y `_exit()` son ambas async-signal-safe según POSIX.
//!
//! Protocolo de control (texto plano, una línea por comando) sobre un
//! socket Unix. La primera línea de cada conexión selecciona el canal:
//!   CHANNEL CMD | CHANNEL READY | CHANNEL NEW_STATE | CHANNEL ACTUATORS_READY
//!
//! Canal CMD:
//!   SNAPSHOT                              -> "NONE" | "OK <21 campos>"
//!   SET_ACTUATORS <e> <a> <r> <t>          -> "OK"
//!   SET_GROUND_ELEV <v>                    -> "OK"
//!   SET_EXEC_MODE <m>                      -> "OK"
//!   STALENESS                              -> "OK <n>"
//!   RESET                                  -> "OK"
//!   STOP                                   -> "OK", el proceso termina

mod event;
mod icd;
mod logging_setup;

use event::EventState;
use icd::{pack_command, unpack_states, StatesPacket, STATES_SIZE};

use std::env;
use std::ffi::CString;
use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::net::UdpSocket;
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::Duration;

static SOCKET_PATH_C: OnceLock<CString> = OnceLock::new();
static SOCKET_PATH: OnceLock<String> = OnceLock::new();

extern "C" fn handle_shutdown_signal(_sig: libc::c_int) {
    if let Some(path) = SOCKET_PATH_C.get() {
        unsafe {
            libc::unlink(path.as_ptr());
        }
    }
    unsafe {
        libc::_exit(0);
    }
}

fn install_signal_handlers(socket_path: &str) {
    let c_path = CString::new(socket_path).expect("socket_path no puede contener un byte nulo");
    let _ = SOCKET_PATH_C.set(c_path);
    let _ = SOCKET_PATH.set(socket_path.to_string());
    unsafe {
        libc::signal(libc::SIGINT, handle_shutdown_signal as usize);
        libc::signal(libc::SIGTERM, handle_shutdown_signal as usize);
    }
}

#[derive(Clone, Copy, Default)]
struct Actuators {
    elevator: f32,
    aileron: f32,
    rudder: f32,
    throttle: f32,
}

struct Shared {
    snapshot: Mutex<Option<StatesPacket>>,
    actuators: Mutex<Actuators>,
    ground_elev_m: Mutex<f32>,
    exec_mode: AtomicU16,
    stale_cycles: AtomicU32,
    stop: AtomicBool,
    ready: EventState,
    new_state: EventState,
    actuators_ready: EventState,
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let socket_path = args.get(1).cloned().unwrap_or_else(|| "/tmp/kse.sock".to_string());

    install_signal_handlers(&socket_path);

    let (session_id, log_path, _guard) = logging_setup::init();
    let _span = tracing::info_span!("session", session_id = %session_id).entered();
    tracing::info!(log_path = %log_path, "logging inicializado");

    let simulink_ip = env::var("KSE_SIMULINK_IP").unwrap_or_else(|_| "127.0.0.1".to_string());
    let telemetry_port: u16 = env::var("KSE_TELEMETRY_PORT").ok().and_then(|s| s.parse().ok()).unwrap_or(6000);
    let command_port: u16 = env::var("KSE_COMMAND_PORT").ok().and_then(|s| s.parse().ok()).unwrap_or(6001);
    let rendezvous_ms: u64 = env::var("KSE_RENDEZVOUS_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(3);
    let stale_warning_every_n: u32 = env::var("KSE_STALE_WARNING_EVERY_N").ok().and_then(|s| s.parse().ok()).unwrap_or(10);
    // Fork CADAC-SIL: en modo "SIL puro" (componentes 'fcs'/'px4' deshabilitados
    // -- default de este fork, ver ComponentsSettings en config_file.py) nadie
    // se conecta jamás al canal ACTUATORS_READY, así que el rendezvous de abajo
    // SIEMPRE vence y el aviso de staleness se repetiría por consola para
    // siempre pese a ser el comportamiento esperado, no un error. kuntur_bridge
    // (el único lanzador de este proceso) pone esta variable en "0" cuando
    // ninguno de esos dos componentes está habilitado; por default queda en
    // "1" para no cambiar el comportamiento de quien invoque kse_server a mano.
    let expect_actuator_client: bool =
        env::var("KSE_EXPECT_ACTUATOR_CLIENT").map(|s| s != "0").unwrap_or(true);
    if !expect_actuator_client {
        tracing::info!(
            "sin componente fcs/px4 habilitado (SIL puro) -- no se esperará \
             confirmación de actuadores; el aviso periódico de staleness queda \
             deshabilitado"
        );
    }

    let shared = Arc::new(Shared {
        snapshot: Mutex::new(None),
        actuators: Mutex::new(Actuators::default()),
        ground_elev_m: Mutex::new(0.0),
        exec_mode: AtomicU16::new(0),
        stale_cycles: AtomicU32::new(0),
        stop: AtomicBool::new(false),
        ready: EventState::new(),
        new_state: EventState::new(),
        actuators_ready: EventState::new(),
    });

    {
        let shared = Arc::clone(&shared);
        let span = tracing::Span::current();
        thread::Builder::new()
            .name("kse-grant-step".into())
            .spawn(move || {
                let _guard = span.enter();
                run_loop(
                    shared, simulink_ip, telemetry_port, command_port,
                    Duration::from_millis(rendezvous_ms), stale_warning_every_n,
                    expect_actuator_client,
                )
            })
            .expect("no se pudo crear el hilo del lazo Grant/STEP");
    }

    let _ = fs::remove_file(&socket_path);
    let listener = UnixListener::bind(&socket_path)
        .unwrap_or_else(|e| panic!("kse_server: no se pudo bindear {socket_path}: {e}"));
    tracing::info!(socket_path = %socket_path, pid = std::process::id(), "kse_server escuchando");

    for conn in listener.incoming() {
        match conn {
            Ok(stream) => {
                let shared = Arc::clone(&shared);
                let span = tracing::Span::current();
                thread::spawn(move || {
                    let _guard = span.enter();
                    handle_connection(stream, shared)
                });
            }
            Err(e) => tracing::warn!(error = %e, "error aceptando conexión"),
        }
    }
}

fn run_loop(
    shared: Arc<Shared>,
    simulink_ip: String,
    telemetry_port: u16,
    command_port: u16,
    rendezvous_timeout: Duration,
    stale_warning_every_n: u32,
    expect_actuator_client: bool,
) {
    let rx_sock = match UdpSocket::bind(("0.0.0.0", telemetry_port)) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!(port = telemetry_port, error = %e, "no se pudo bindear el puerto RX");
            std::process::exit(1);
        }
    };
    rx_sock.set_read_timeout(Some(Duration::from_secs(1))).ok();

    let tx_sock = UdpSocket::bind(("0.0.0.0", 0)).expect("no se pudo crear el socket TX");
    let target = (simulink_ip.as_str(), command_port);

    tracing::info!(simulink_ip = %simulink_ip, command_port = command_port, icd_version = "1.3", "lazo Grant/STEP iniciado");

    let mut sim_time_us: u64 = 0;
    let mut buf = [0u8; STATES_SIZE + 32];
    let mut ready_sent = false;

    while !shared.stop.load(Ordering::SeqCst) {
        let (elev, ail, rud, thr) = {
            let a = shared.actuators.lock().unwrap();
            (a.elevator, a.aileron, a.rudder, a.throttle)
        };
        let ground_elev = *shared.ground_elev_m.lock().unwrap();
        let exec_mode = shared.exec_mode.load(Ordering::SeqCst);

        let cmd = pack_command(sim_time_us, elev, ail, rud, thr, ground_elev, exec_mode);
        if let Err(e) = tx_sock.send_to(&cmd, target) {
            tracing::warn!(error = %e, "error enviando KUNTUR_COMMAND");
        }

        let n = match rx_sock.recv(&mut buf) {
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => continue,
        };

        let states = match unpack_states(&buf[..n]) {
            Ok(s) => s,
            Err(_) => continue,
        };

        *shared.snapshot.lock().unwrap() = Some(states);

        shared.actuators_ready.clear();
        shared.new_state.set();
        let acked = shared.actuators_ready.wait(rendezvous_timeout);

        if acked {
            shared.stale_cycles.store(0, Ordering::SeqCst);
        } else {
            let n = shared.stale_cycles.fetch_add(1, Ordering::SeqCst) + 1;
            if expect_actuator_client && n % stale_warning_every_n == 0 {
                tracing::warn!(
                    stale_cycles = n,
                    rendezvous_timeout_ms = rendezvous_timeout.as_secs_f64() * 1000.0,
                    "FCS/PX4-Bridge sin confirmar hace {n} ciclos consecutivos"
                );
            }
        }

        if !ready_sent {
            shared.ready.set();
            ready_sent = true;
            tracing::info!("primer KUNTUR_STATES válido recibido, KSE listo");
        }

        sim_time_us = states.t_sim_us;
    }

    tracing::info!("lazo Grant/STEP terminado");
}

fn handle_connection(stream: UnixStream, shared: Arc<Shared>) {
    let mut reader = BufReader::new(match stream.try_clone() {
        Ok(s) => s,
        Err(_) => return,
    });
    let mut writer = stream;

    let mut first_line = String::new();
    if reader.read_line(&mut first_line).unwrap_or(0) == 0 {
        return;
    }

    match first_line.trim() {
        "CHANNEL CMD" => handle_cmd_channel(reader, writer, &shared),
        "CHANNEL READY" => handle_event_channel(reader, writer, &shared.ready),
        "CHANNEL NEW_STATE" => handle_event_channel(reader, writer, &shared.new_state),
        "CHANNEL ACTUATORS_READY" => handle_event_channel(reader, writer, &shared.actuators_ready),
        other => {
            let _ = writeln!(writer, "ERR canal desconocido: {other}");
        }
    }
}

fn handle_event_channel(mut reader: BufReader<UnixStream>, mut writer: UnixStream, ev: &EventState) {
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line).unwrap_or(0) == 0 {
            return;
        }
        let mut parts = line.trim().split_whitespace();
        let resp = match parts.next() {
            Some("WAIT") => {
                let timeout_s: f64 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
                if ev.wait(Duration::from_secs_f64(timeout_s)) { "SET" } else { "TIMEOUT" }
            }
            Some("CLEAR") => { ev.clear(); "OK" }
            Some("SET") => { ev.set(); "OK" }
            Some("IS_SET") => if ev.is_set() { "1" } else { "0" },
            _ => "ERR comando desconocido",
        };
        if writeln!(writer, "{resp}").is_err() {
            return;
        }
    }
}

fn handle_cmd_channel(mut reader: BufReader<UnixStream>, mut writer: UnixStream, shared: &Arc<Shared>) {
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line).unwrap_or(0) == 0 {
            return;
        }
        let trimmed = line.trim();
        let mut parts = trimmed.split_whitespace();
        let cmd = parts.next().unwrap_or("");

        let resp = match cmd {
            "SNAPSHOT" => {
                let guard = shared.snapshot.lock().unwrap();
                match &*guard {
                    None => "NONE".to_string(),
                    Some(sp) => format!(
                        "OK {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
                        sp.t_sim_us, sp.seq_num,
                        sp.north, sp.east, sp.down,
                        sp.phi, sp.theta, sp.psi,
                        sp.p, sp.q, sp.r,
                        sp.ax, sp.ay, sp.az,
                        sp.vn, sp.ve, sp.vd,
                        sp.ias, sp.alpha, sp.beta,
                        sp.status,
                    ),
                }
            }
            "SET_ACTUATORS" => {
                let vals: Vec<f32> = parts.filter_map(|s| s.parse().ok()).collect();
                if vals.len() == 4 {
                    let mut a = shared.actuators.lock().unwrap();
                    *a = Actuators { elevator: vals[0], aileron: vals[1], rudder: vals[2], throttle: vals[3] };
                    "OK".to_string()
                } else {
                    "ERR argumentos".to_string()
                }
            }
            "SET_GROUND_ELEV" => {
                if let Some(v) = parts.next().and_then(|s| s.parse::<f32>().ok()) {
                    *shared.ground_elev_m.lock().unwrap() = v;
                    "OK".to_string()
                } else {
                    "ERR argumentos".to_string()
                }
            }
            "SET_EXEC_MODE" => {
                if let Some(m) = parts.next().and_then(|s| s.parse::<u16>().ok()) {
                    shared.exec_mode.store(m, Ordering::SeqCst);
                    "OK".to_string()
                } else {
                    "ERR argumentos".to_string()
                }
            }
            "STALENESS" => format!("OK {}", shared.stale_cycles.load(Ordering::SeqCst)),
            "RESET" => {
                *shared.snapshot.lock().unwrap() = None;
                *shared.actuators.lock().unwrap() = Actuators::default();
                shared.stale_cycles.store(0, Ordering::SeqCst);
                shared.ready.clear();
                shared.new_state.clear();
                shared.actuators_ready.clear();
                tracing::info!("RESET -- estado lógico reinicializado, proceso sigue corriendo");
                "OK".to_string()
            }
            "STOP" => {
                let _ = writeln!(writer, "OK");
                let _ = writer.flush();
                if let Some(path) = SOCKET_PATH.get() {
                    let _ = std::fs::remove_file(path);
                }
                std::process::exit(0);
            }
            _ => "ERR comando desconocido".to_string(),
        };

        if writeln!(writer, "{resp}").is_err() {
            return;
        }
    }
}

//! logging_setup.rs -- Configuración de logging de kse_server.
//!
//! Mismo principio que logging_setup.py del lado Python (ver
//! sistema_de_logs_kuntur_sim.md, NF1-NF7): el escritor a archivo es
//! no bloqueante (tracing_appender::non_blocking) -- el hilo del lazo
//! Grant/STEP nunca hace I/O síncrono al loggear, solo entrega el
//! evento a un canal interno que un hilo de tracing-appender drena por
//! su cuenta.
//!
//! Esquema JSON: se usa el formateador json() estándar de
//! tracing-subscriber (no uno custom) -- produce un esquema distinto al
//! de logging_setup.py del lado Python (timestamp/level/fields/span vs.
//! ts/level/session_id/seq_num planos). run_sim.py logs normaliza los
//! dos esquemas al leer, en vez de forzar a Rust a imitar el esquema de
//! Python a mano.
//!
//! session_id viaja como campo de un span raíz que envuelve todo main()
//! -- con with_current_span(true) en el layer JSON, cada línea de
//! evento incluye los campos del span activo, así que session_id queda
//! en cada línea sin tener que pasarlo a mano en cada log::info!.

use std::fs::OpenOptions;
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::prelude::*;

/// Devuelve (session_id, log_path, guard). Cada proceso escribe su
/// PROPIO archivo (`{session_id}.kse_server.jsonl`) -- no comparte un
/// único archivo con kuntur_bridge (que escribe
/// `{session_id}.kuntur_bridge.jsonl`), evitando así tener que
/// garantizar escritura atómica cruzada entre dos runtimes distintos.
/// `run_sim.py logs` combina ambos archivos de la sesión al leer.
///
/// El guard debe mantenerse vivo (no dropearse) durante toda la
/// ejecución de main(): al dropearse fuerza el flush final del buffer
/// no bloqueante (importante para no perder los últimos eventos del
/// apagado -- NF7).
pub fn init() -> (String, String, WorkerGuard) {
    let session_id = std::env::var("KUNTUR_SESSION_ID").unwrap_or_else(|_| generate_session_id());
    let dir = "/tmp/kuntur_sim_logs";
    let _ = std::fs::create_dir_all(dir);
    let log_path = format!("{dir}/{session_id}.kse_server.jsonl");

    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
        .unwrap_or_else(|e| panic!("kse_server: no se pudo abrir el log {log_path}: {e}"));

    let (non_blocking, guard) = tracing_appender::non_blocking(file);

    let json_layer = tracing_subscriber::fmt::layer()
        .json()
        .with_writer(non_blocking)
        .with_current_span(true)
        .with_span_list(false);

    let console_layer = tracing_subscriber::fmt::layer()
        .with_target(false)
        .without_time()
        .with_writer(std::io::stdout);

    tracing_subscriber::registry()
        .with(json_layer)
        .with(console_layer)
        .init();

    (session_id, log_path, guard)
}

fn generate_session_id() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
    format!("kse-{secs}")
}

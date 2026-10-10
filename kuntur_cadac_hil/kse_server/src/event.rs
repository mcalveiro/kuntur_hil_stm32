//! event.rs -- Primitivo de sincronización estilo `threading.Event` de
//! Python, en Rust puro (Mutex + Condvar de la std). kse_server es un
//! proceso completamente separado del intérprete Python de
//! kuntur_bridge -- no hay forma de sostener un Event real de Python
//! acá, así que se reimplementa la misma semántica de forma nativa.

use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

pub struct EventState {
    flag: Mutex<bool>,
    cv: Condvar,
}

impl EventState {
    pub fn new() -> Self {
        Self { flag: Mutex::new(false), cv: Condvar::new() }
    }

    pub fn set(&self) {
        let mut f = self.flag.lock().unwrap();
        *f = true;
        self.cv.notify_all();
    }

    pub fn clear(&self) {
        *self.flag.lock().unwrap() = false;
    }

    pub fn is_set(&self) -> bool {
        *self.flag.lock().unwrap()
    }

    pub fn wait(&self, timeout: Duration) -> bool {
        let mut f = self.flag.lock().unwrap();
        if *f {
            return true;
        }
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return *f;
            }
            let (guard, result) = self.cv.wait_timeout(f, deadline - now).unwrap();
            f = guard;
            if *f || result.timed_out() {
                return *f;
            }
        }
    }
}

impl Default for EventState {
    fn default() -> Self {
        Self::new()
    }
}

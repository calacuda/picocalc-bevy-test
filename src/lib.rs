#![no_main]
#![no_std]

pub mod env;
pub mod lfo;
pub mod osc;
pub mod synth;

pub const SAMPLE_RATE: u32 = 44_100;

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

// Alias for our HAL crate
// use rp2040_hal as _; // memory layout
use rp_hal as _; // memory layout

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#![no_main]
#![no_std]

use pico_synth as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = rp2040_hal::pac,
    dispatchers = [UART1_IRQ],
)]
mod app {
    use cortex_m::interrupt::Mutex;
    use pico_synth::synth::Synth;
    // Alias for our HAL crate
    use rp2040_hal as hal;

    // A shorter alias for the Peripheral Access Crate, which provides low-level
    // register access
    use hal::pac;

    /// Shared resources go here
    #[shared]
    struct Shared {
        synth: Mutex<Synth>,
    }

    /// Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);

        let mut resets = cx.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(cx.device.WATCHDOG);

        // Configure the clocks - The default is to generate a 125 MHz system clock
        let _clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // midi_input::spawn().ok();

        // TODO: build synth store in Arc Mutex
        let synth = Mutex::new(Synth::new());

        // TODO: spawn thread (one the second core) with super loop arch that generates and sends samples over i2s

        (
            Shared {
                // Initialization of shared resources go here
                synth,
            },
            Local {
                // Initialization of local resources go here
            },
        )
    }

    //
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            // TODO: look for chagnes in the knobs
            continue;
        }
    }

    /// modify synth when midi input is recieved
    #[task(priority = 1)]
    async fn midi_input(_cx: midi_input::Context) {
        defmt::info!("Hello from task1!");
    }
}

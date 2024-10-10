use crate::osc::{Oscillator, Overtone};

pub const WAVE_TABLE_SIZE: usize = 128;
pub const VOICES: usize = 10;

pub struct Synth {
    osc_s: [Oscillator; VOICES],
    wave_table: [f32; WAVE_TABLE_SIZE],
}

impl Synth {
    pub fn new() -> Self {
        let overtones = [
            Some(Overtone {
                overtone: 0.25,
                volume: 1.0,
            }),
            Some(Overtone {
                overtone: 0.5,
                volume: 1.0,
            }),
            Some(Overtone {
                overtone: 1.0,
                volume: 1.0,
            }),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        ];
        let wave_table = Self::build_wave_table(overtones);

        Self {
            osc_s: [Oscillator::new(); VOICES],
            wave_table,
        }
    }

    fn build_wave_table(overtones: [Option<Overtone>; 10]) -> [f32; WAVE_TABLE_SIZE] {
        let mut wave_table = [0.0; WAVE_TABLE_SIZE];

        let mut n_overtones = 0;

        for ot in overtones {
            if ot.is_some() {
                n_overtones += 1;
            }
        }

        let bias = 1.0 / n_overtones as f32;

        for i in 0..WAVE_TABLE_SIZE {
            for ot in overtones {
                if let Some(ot) = ot {
                    wave_table[i] += (libm::sin(
                        2.0 * core::f64::consts::PI * i as f64 * ot.overtone
                            / WAVE_TABLE_SIZE as f64,
                    ) * ot.volume) as f32
                }
            }

            wave_table[i] *= bias;
        }

        wave_table
    }

    pub fn set_overtones(&mut self, overtones: [Option<Overtone>; 10]) {
        self.wave_table = Self::build_wave_table(overtones);
    }

    pub fn get_sample(&mut self) -> f32 {
        let mut sample = 0.0;

        for ref mut osc in self.osc_s {
            sample += osc.get_sample(&self.wave_table)
        }

        sample
    }
}

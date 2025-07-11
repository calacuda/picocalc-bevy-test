// macro_rules! wrap_input_err {
//     ($expr:expr) => {
//         $expr.map_err(|_e| DisplayError::BusReadError)
//     };
// }

macro_rules! wrap_output_err {
    ($expr:expr) => {
        $expr.map_err(|_e| DisplayError::BusWriteError)
    };
}

// macro_rules! write_bit {
//     ($output_pin:expr, $bit_set:expr) => {
//         if $bit_set {
//             wrap_output_err!($output_pin.set_high())?
//         } else {
//             wrap_output_err!($output_pin.set_low())?
//         }
//     };
// }

pub type U18 = u32;

// pub use self::gpio16::*;
// pub use self::gpio18::*;
// pub use self::gpio8::*;
//
// mod gpio16;
// mod gpio18;
// mod gpio8;

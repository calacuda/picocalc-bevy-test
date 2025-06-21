use core::fmt::Debug;

// use embedded_hal::digital::InputPin;
// use embedded_hal::digital::OutputPin;

///
/// IoPin combines [InputPin](InputPin) and [OutputPin](OutputPin) to allow for switchable I/O on the same pin.
///
pub trait IoPin {
    type InputPinError: Debug;
    type OutputPinError: Debug;
    type Input: embedded_hal::digital::InputPin<Error = Self::InputPinError>
        + IoPin<
            Input = Self::Input,
            Output = Self::Output,
            InputPinError = Self::InputPinError,
            OutputPinError = Self::OutputPinError,
        >;
    type Output: embedded_hal::digital::OutputPin<Error = Self::OutputPinError>
        + IoPin<
            Input = Self::Input,
            Output = Self::Output,
            InputPinError = Self::InputPinError,
            OutputPinError = Self::OutputPinError,
        >;

    /// Switch to input mode
    fn into_input(&mut self) -> &mut Self::Input;

    /// Switch to output mode
    fn into_output(&mut self) -> &mut Self::Output;
}

pub mod shim;

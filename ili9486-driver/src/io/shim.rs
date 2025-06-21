use crate::IoPin;
use core::fmt::Debug;
use embedded_hal::digital::ErrorKind;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
// use embedded_hal::spi::ErrorKind;

#[derive(Debug)]
pub enum Error {
    InputNotAvailableError,
}

impl embedded_hal::digital::Error for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl Into<ErrorKind> for Error {
    fn into(self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    pin: P,
}

// impl<P> InputPin for <OutputOnlyIoPin<P> as IoPin>::Input
// where
//     P: OutputPin,
//     P::Error: Debug,
// {
//     fn is_high(&mut self) -> Result<bool, Self::Error> {
//         Ok(self.pin.is_high())
//     }
//
//     fn is_low(&mut self) -> Result<bool, Self::Error> {
//         Ok(self.pin.is_low())
//     }
// }

impl<P> OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    pub fn new(pin: P) -> OutputOnlyIoPin<P> {
        OutputOnlyIoPin { pin: pin }
    }
}

impl<P> embedded_hal::digital::ErrorType for OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    type Error = Error;
}

impl<P> IoPin for OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    type InputPinError = Error;
    type OutputPinError = Error;
    type Input = Self;
    type Output = Self;

    fn into_input(&mut self) -> &mut Self::Input {
        self
    }

    fn into_output(&mut self) -> &mut Self::Output {
        self
    }
}

// impl<P> InputPir for OutputOnlyIoPin<P>
// where
//     P: OutputPin,
//     P::Error: Debug,
// {
//     // type Error = Error;
//
//     fn is_high(&self) -> Result<bool, Self::Error> {
//         Err(Error::InputNotAvailableError)
//     }
//     fn is_low(&self) -> Result<bool, Self::Error> {
//         Err(Error::InputNotAvailableError)
//     }
// }
//
// impl<P> OutputPin for OutputOnlyIoPin<P>
// where
//     P: OutputPin,
//     P::Error: Debug,
// {
//     // type Error = P::Error;
//
//     #[inline]
//     fn set_low(&mut self) -> Result<(), Self::Error> {
//         self.pin.set_low()
//     }
//     #[inline]
//     fn set_high(&mut self) -> Result<(), Self::Error> {
//         self.pin.set_high()
//     }
// }

impl<'a, P> InputPin for OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    // type Error = Error;

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Err(Error::InputNotAvailableError.into())
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Err(Error::InputNotAvailableError.into())
    }
}

impl<'a, P> OutputPin for OutputOnlyIoPin<P>
where
    P: OutputPin,
    P::Error: Debug,
{
    // type Error = P::Error;

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        _ = self.pin.set_low();
        Ok(())
    }
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        _ = self.pin.set_high();
        Ok(())
    }
}

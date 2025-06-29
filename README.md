# Picocalc Bevy Test

A test of rust the bevy game engine running on the [ClockWorkPI Picocalc](https://www.clockworkpi.com/picocalc). The Picocalc is equiped with a Raspberry PI Pico 2 (rp2350).

> Quickly set up a [`probe-rs`] + [`defmt`] + [`flip-link`] embedded project
> running on the [`RTIC`] scheduler

[`probe-rs`]: https://crates.io/crates/probe-rs
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link
[`RTIC`]: https://rtic.rs/

Based on [https://github.com/knurling-rs/app-template](https://github.com/knurling-rs/app-template)

## Whats The Point

The point of this repo is to get a bevy application running on a Raspberry Pi Pico 2 powered Picocalc.

## Test Status

This test is considered a **Success**. The only thing yet to be implemented is audio output.

- [x] 2d screen rendering
- [x] 3d screen rendering
- [x] keyboard input
- [x] SD Card reader/writer
- [ ] Audio output

## License

Licensed under

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

<!-- ### Contribution -->
<!---->
<!-- Unless you explicitly state otherwise, any contribution intentionally submitted -->
<!-- for inclusion in the work by you, as defined in the Apache-2.0 license, shall be -->
<!-- licensed as above, without any additional terms or conditions. -->
<!---->
<!-- [Knurling]: https://knurling.ferrous-systems.com -->
<!-- [Ferrous Systems]: https://ferrous-systems.com/ -->
<!-- [GitHub Sponsors]: https://github.com/sponsors/knurling-rs -->

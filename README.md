> **Warning**:
> Still in progress to make this public friendly. Please be patience. Feel free to contact me (email on github profile).

> **Warning**:
> If you decide to run this software, make sure you know what you are doing.
> Electrical fire and explosion can happen if hardware and software are not configured properly.
> I'm taking no responsibility of all your losses caused by running this software.

## DIY Force Feedback Steering Wheel
Code name: `wheel03-pico`

- Microcontroller: Raspberry Pi Pico
- Motor: Hoverboard motor
- Motor driver: 2x IBT-2 module
- Position sensor: AS5600
- Power supply: FSP 24V, but generic 12V 10A PSU will also works with voltage adjustment on code.
- Motor control software: SimpleFOC
- PC side software: Custom (public repo coming soon) and vJoy
- Pedals: currently using Thrustmaster T3PA set

## Data Communication Strategy
A bit unique because this microcontroller is not communicate directly with directX or any other game API.
This microcontroller is just communicating to a [software on the PC](https://github.com/suryavip/wheel01-pc) side via USB serial.
The software on the PC side that will communicate with vJoy API, which then communicate to game API.

`Microcontroller` ↔ `Software on PC` ↔ `vJoy` ↔ `Game`.

## More details are coming soon...

## Thanks to
- [SimpleFOC](https://simplefoc.com/)
- [Earle F. Philhower, III](https://github.com/earlephilhower) for the [arduino-pico](https://github.com/earlephilhower/arduino-pico)
- [Rob Tillaart](https://github.com/RobTillaart) for the [AS5600](https://github.com/RobTillaart/AS5600) and [MultiMap](https://github.com/RobTillaart/MultiMap) libraries
- [Bruno Azevedo Chagas](https://github.com/bachagas) for the [Kalman](https://github.com/bachagas/Kalman) library
- [propeler on simracing.su forum](https://forum.simracing.su/profile/5730-propeler/) for the mounting inspiration

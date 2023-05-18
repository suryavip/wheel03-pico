> __Note__: Still in progress to make this public friendly. Please be patience. Feel free to contact me (email on github profile).

# DIY Force Feedback Steering Wheel
Code name: `wheel03-pico`

- Microcontroller: Raspberry Pi Pico
- Motor: Hoverboard motor
- Motor driver: 2x IBT-2 module
- Position sensor: AS5600
- Power supply: FSP 24V, but generic 12V 10A PSU will also works with voltage adjustment on code.
- Motor control software: SimpleFOC
- PC side software: Custom (public repo coming soon) and vJoy

## Data Communication Strategy
A bit unique because this microcontroller is not communicate directly with directX or any other game API.
This microcontroller is just communicating to a software on the PC side via USB serial.
The software on the PC side that will communicate with vJoy API, which then communicate to game API.

`Microcontroller` ↔ `Software on PC` ↔ `vJoy` ↔ `Game`.

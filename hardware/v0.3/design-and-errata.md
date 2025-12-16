# Bed lift v0.3

**Status: Incomplete, work in progress**

## Design synopsis

The controller has the following major components:

- A [STM32C031K6T6](https://www.st.com/resource/en/datasheet/stm32c031k6.pdf) controls the actuators. It is programmed with SWD using a [Tag-Connect 2030-NL](https://www.tag-connect.com/info) connector, UART, or I2C.
- A [ISO1640](https://www.ti.com/lit/ds/symlink/iso1640.pdf) isolates the I2C bus to prevent ground loops through the `QWIIC` connector because the driver has a separate power supply.
- Four [DRV8434S](https://www.ti.com/lit/ds/symlink/drv8434s.pdf) stepper motor drivers operate the four motors that extend and retract the side spans of the bed.
- Two [DRV8874](https://www.ti.com/lit/ds/slvsf66a/slvsf66a.pdf) motor drivers operate the two linear actuators that raise and lower the bed.

The MCU receives control inputs from a momentary rocker switch attached to the `CONTROL` input and it provides feedback on its progress to an RGB LED indicator.  It also receives control inputs and provides feedback over I2C to a host controller via the `QWIIC` connector.  It receives position feedback to synchronize the motors from the lift actuator hall sensors, lift `LIMIT` switches, and stepper motor driver stall detection.

The circuit board requires a 12 V DC nominal supply protected by a 10 A external fuse. If that isn't enough current, the it should be safe to operate the board at up to 15 A with a limited duty cycle of a few minutes.

The schematics include the BOM and metadata for the JLCPCB fabrication toolkit plug-in.

## Illustrations

[Schematics PDF](bed-lift.pdf)

<details>
<summary>PCB front and back</summary>

### PCB front
![PCB front](bed-lift-front.png)

### PCB back
![PCB back](bed-lift-back.png)

</details>

## Errata

None yet...

## Changes since v0.2

Corrected the lift motor connector pins.

Shuffled GPIOs:

- Added `SPI1_NSS` signal for DRV8434S chip-select from `PA4`.
- Moved `SPAN_FAULT` signal from `PA4` to `PA3`.
- Moved `DRV_SLEEP` signal from `PA3` to `PC15`.

TODO:

- Use TIM1 for LED PWM and general purpose timing at ~1 kHz, 4 external channels + 2 internal channels
- Use TIM3 for lift motor PWM at ~25 kHz, 4 external channels
- Consider swapping lift motor to TIM1 to allow use of the BREAK function to stop the motor when the debugger fires
  - Set DBG_TIM1_STOP bit to stop the timer when the core is halted
  - Set OSSI = 1 bit to make the counter outputs inactive when the timer is stopped
  - Although TIM3 can be stopped with DBG_TIM3_STOP, it doesn't seem to support disabling the output
- Move the hall signal pairs to ports on EXTI0-1 and EXTI2-3 to allow for higher priority interrupt handling of these signals compared to all of the other signals that share the EXTI4-15 vector. Unfortunate STM32C0 only has one advanced timer otherwise the timer could decode the signal in hardware.
- Use TIM14/16/17 for span motor STEP
- Upgrade to LQFP48 to alleviate pin scarcity
- Upgrade to more flash memory, maybe 64 KB for future proofing
- Maybe add bulk capacitors for every motor driver, perhaps 47 uF aluminum polymer to distribute the switching current, although it seems to be working fine.
- Maybe add a connector for a PWM fan.
- Add the heatsinks to the BOM and model.
- Maybe remove the serial port connector and repurpose the pins.
- Maybe split the `DRV_SLEEP` signal for the two groups of motor drivers to remove a dependency between the drivers in firmware.
- Supply the STEP signal from a timer capable GPIO.
- Provide PWM capable outputs for the lift motors.
- Gate the motor driver SLEEP through a 2 pin e-stop switch connector, sense the result with a GPIO.
- Ensure these signals share a port for efficient reading
  - LIFT_LIMIT_A + LIFT_LIMIT_B (PB)
  - LIFT1_HALL1 + LIFT1_HALL2 (PB)
  - LIFT2_HALL1 + LIFT2_HALL2 (PA) --> could combine with other hall port
  - CTRL_LOWER + CTRL_RAISE (PB)
- Ensure each GPIO pin that will have an interrupt assigned has a unique pin number because there is one only EXTI line per pin number independent of the number of ports
- Replace the TagConnect pads with an STDC14 1.27 mm connector for a stronger mechanical attachment when debugging in-situ
- Add 5 V LDO (LP2985-50DBVR) for the hall sensors instead of a buffer.  Ensure there is a pull-down from ON/OFF to GND to ensure the LDO shuts off when the MCU is not driving the pin.  Should already be the case if enabled by DRV_SLEEP.

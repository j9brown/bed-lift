# Bed lift v0.2 (OBSOLETE)

**Status: Superseded by v1.0, see [errata](#errata)**

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

## Lift control and limit switches

Install two limit switches on the bed frame.  Each limit switch has a compliant arm that is depressed when it passes over a raised strip of material attached to the face of one of the bed lift columns.  Together the limit switches encode the position of the bed.

| State | Position                  | `Limit A` | `Limit B` | Wing action          |
| ----- | ------------------------- | --------- | --------- | -------------------- |
| `1`   | Maximum elevation         | `ON`      | `OFF`     | Fully retracted      |
| `2`   | Above safe zone           | `OFF`     | `OFF`     | Fully retracted      |
| `3`   | Within safe zone          | `OFF`     | `ON`      | Extending/retracting |
| `4`   | Below safe zone           | `ON`      | `ON`      | Fully extended       |

To raise the bed...

  - state `4`: raise platform
  - state `3`: retract wings then raise platform (can do both simultaneously)
  - state `2`: retract wings then raise platform
  - state `1`:
    - if full range of travel permitted (no cushions or other bulky materials stored on the bed) then raise platform until end-of-travel
    - otherwise stop

To lower the bed...

  - state `1`: lower platform
  - state `2`: lower platform
  - state `3`: extend wings then lower platform (can do both simultaneously)
  - state `4`:
    - if wings fully extended then lower platform until end-of-travel
    - otherwise: raise platform until state `3` achieved and the wings can be extended

## Additional materials

Control panel switches

  - 3 gang switch panel mount
    - Carling Technologies VM3-01
  - Up/down control
    - Carling Technologies V8D2S001-00000-000 Contura momentary on/off/on rocker switch
    - Carling Technologies VVJZZ00-000 Contura XII paddle actuator
  - Mode control
    - Carling Technologies V2D2S001-00000-000 Contura momentary on/off rocker switch
    - Carling Technologies VV6ZZ00-000 Contura XI recessed actuator
  - Arm/disarm control
    - Carling Technologies V1D2S001-00000-000 Contura maintained on/off rocker switch
    - Carling Technologies VV1ZZ00-000 Contura X raised actuator
  - Generic small signal diodes to connect the mode switch to both up/down inputs

Control panel indicators

  - 3 V RGB panel indicator
    - Dialight 6251121307F or 6201121117F

## Hall sensor noise

Each lift actuator provides a pair of digital hall sensor signals that provide feedback for the motor rotational direction and speed with quadrature encoding.  The actuator datasheet states that it provides feedback at 660 pulses per linear inch of travel.

These signals were observed with a logic analyzer.

At maximum speed, the signals had a period of approximately 3.7 ms (270 Hz) corresponding to 0.4" per second of linear travel.  At slower speeds, the signals had a longer period as expected.  The low pulses tended to be shorter than the high pulses, sometimes as short as 1 ms out of a total 4 ms period.  The shortest pulses observed were about 950 us.  The exact duty cycle seemed to vary among actuators.

The signals also showed jitter where one of the sensor in a pair briefly reverts to a prior state, typically for less than 50 us and sometimes after around 500 us.  Sometimes one or more of the hall sensors didn't respond at all (particularly when it was cold outside).

It seems that the malfunction was caused by supplying the hall sensors with 3.3 V instead of 5 V as indicated in the actuator datasheet.  Prior testing had shown that the sensors would tolerate 3.3 V but they may have been borderine.  Providing 5 V seems to have resolved the issue.

## Errata

Be advised that this version of the circuit board contains errors that must be corrected for correct operation.

The lift actuator connector pins 2 and 3 are swapped which reverses the polarity of the power supply to the hall sensors.

  - Cut a section out of the lead for pin 3 where it connects to GND.
  - Cut the VHALL trace for pin 3. Expose part of the trace under the solder mask.
  - Solder VHALL to the stub of the pin 3 lead.
  - On the back side of the board, form a solder bridge between the pin 2 and pin 3 terminals thereby connecting pin 2 to GND.

The DRV8434S must observe a high-to-low transition of nSCS to being an SPI transaction.  Unfortunately, nSCS was tied to ground because it was believed no chip select was needed due to the absence of other devices on the SPI bus.

  - Detach nSCS (pin 18) from each DRV8434S, bend it away from the PCB, solder jumpers to PC15 (pin 3) of the microcontroller.
  - Note that the pad is tied to ground underneath the chip so cutting the trace is not practical (unless the whole chip is removed).

The DRV8434S stall detection logic requires STEP pulses to arrive at regular intervals so as to not false trigger during acceleration.  Unfortunately, sending pulses over SPI introduces too much jitter (at least with Zephyr's SPI drivers) so stall detection doesn't work.  The STEP pulses must be generated by a hardware timer instead.

  - Detach STEP (pin 23) from each DRV8434S, bend it away from the PCB, solder jumpers to PC14 (pin 2) of the microcontroller.
  - Note that the pad is tied to ground underneath the chip so cutting the trace is not practical (unless the whole chip is removed).

The DRV8434S VREF pin is directly connected to DVDD without a voltage divider which results it in receiving 5 V instead of the recommended 3.3 V.  It still works but is out of spec.  When designing the circuit, I assumed incorrectly that DVDD supplied 3.3 V.

Having both groups of motor drivers share the same DRV_SLEEP signal makes it harder to control them independently.  Conversely, it would be nice to gate the sleep signals through a physical emergency stop switch connection.

The lift actuators are synchronized by observing pulses from the hall sensors.  The original plan was to bit-bang the motor drivers to stall whichever motor is falling behind as was done in the v0.1 prototype but we can get smoother operation of the motor and also support a slow-start acceleration ramp using PWM.  Unfortunately, one of the motor driver inputs must be relocated to use the timer function.

  - Cut the traces for LIFT_1_IN2 and LIFT_FAULT where they exit their vias near R16.
  - Solder a wire from MCU PB3 (pin 27) to LIFT1 DRV8874 IN2 (pin 2).  This will be the new LIFT1_IN2.
  - Solder a wire from MCU PA15 (pin 26) to LIFT1 DRV8874 FAULT (pin 4).  This will be the new LIFT_FAULT.
  - Effectively MCU pin 26 and 27 are being swapped so that LIFT_IN2 can be driven by TIM3_CH2.  The other inputs can already be bound to the other TIM3 channels.

Although the lift is only intended to move while the user is actively pressing the rocker switch, a software malfunction could cause it to move in an uncoordinated manner. To disarm the system, we can disconnect the DRV_SLEEP pin and connect it through a switch on the control panel or an emergency stop button.  Decided not to make this modification and to add a jumper to the next board revision instead.

  - Cut the trace for DRV_SLEEP between the MCU and R16.
  - Solder a wire from MCU pin 10 to a wire-to-wire connector.
  - Solder a wire from R16's DRV_SLEEP pad to a wire-to-wire connector.
  - Connect an on/off switch to the wire-to-wire connector.

The lift actuator hall sensors sometimes do not work.  They are intended to be driven at 5 V and while they seem to tolerate 3.3 V most of the time, sometimes they produce inconsistent pulses or no pulses at all.  It seems to be more of a problem in cold conditions.  Resolved this issue by removing the VHALL buffer (U4) and powering the hall sensors with a 5 V LDO.

Apply epoxy to the bodges to keep them secure.

Attach adhesive heatsinks to the bottom of the board.  It worked well to cut up two generic 70 mm x 25 mm x 10 mm heatsinks to fit the spaces below both groups of motor drivers.

## Changes since v0.1

Designed a feature-complete circuit board.

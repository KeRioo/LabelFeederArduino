# LabelFeederArduino

An Arduino-based project for controlling a label feeder using an Arduino Mega, TMC2209 stepper motor driver, and L293D motor driver.

## Hardware Components

- Arduino Mega
- TMC2209 Stepper Motor Driver
- L293D Motor Driver
- [Arduino Hat Board](https://oshwlab.com/mateusz.przybyl.mp/controlmodule_labelfeeder_rev2)

## Error List

ERROR<00>: Unknown command

ERROR<01>: Limit switch was triggered

ERROR<02>: Stepper motor software limit

ERROR<03>: Low Vacuum during swing movement

ERROR<04>: Swing Blocked

ERROR<05>: No labels in the feeder

ERROR<06>: No communication with stepper motor

## State List

STATE<00>: ERROR

STATE<01>: IDLE

## Commands

RESET

NEXT_LABEL

## Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/KeRioo/LabelFeederArduino.git
   ```
2. Open the project with PlatformIO.

3. Upload the code to your Arduino Mega.

## Usage

1. Connect the hardware components as per the schematic.
2. Power on the system.
3. Use the commands listed above to control the label feeder.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>

HardwareSerial & serial_stream = Serial3;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;

const uint8_t STEPS_PER_MM = 320;
const uint8_t MAX_SPEED = (STEPS_PER_MM * 5);
const uint8_t HOMING_SPEED = (STEPS_PER_MM * 1);
const uint8_t MAX_ACCELERATION = (STEPS_PER_MM * 15);



// Instantiate TMC2209
TMC2209 stepper_driver;

AccelStepper stepper(AccelStepper::DRIVER, 2, 3);


void setup()
{
  Serial.begin(115200);

  //Stepper Driver config

  stepper_driver.setup(serial_stream);

  stepper_driver.setMicrostepsPerStepPowerOfTwo(5); // 32 microsteps per step
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);

  stepper_driver.setCoolStepCurrentIncrement(TMC2209::CURRENT_INCREMENT_8);
  stepper_driver.setCoolStepMeasurementCount(TMC2209::MEASUREMENT_COUNT_1);
  stepper_driver.setCoolStepDurationThreshold(2000);
  stepper_driver.enableCoolStep(1,1);

  stepper_driver.enable();

  //Stepper Accelaration Config

  stepper.setEnablePin(4);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION)



}

void loop()
{
  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

}

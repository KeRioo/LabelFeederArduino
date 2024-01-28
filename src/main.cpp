#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>
#include <L293D.h>

HardwareSerial & serial_stream = Serial3;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;

const uint32_t STEPS_PER_MM = 320;
const uint32_t MAX_TRAVEL = (STEPS_PER_MM * 120);
const uint32_t MAX_SPEED = (STEPS_PER_MM * 5);
const uint32_t HOMING_SPEED = (STEPS_PER_MM * 1);
const uint32_t MAX_ACCELERATION = (STEPS_PER_MM * 15);



// Pinout
const uint8_t pin_ValveVaccum       = 42; 
const uint8_t pin_ValveProbe        = 44; 
const uint8_t pin_ValveClampSmall   = 46; 
const uint8_t pin_ValveClampBig     = 48;

const uint8_t pin_EndstopR_Max      = 30; 
const uint8_t pin_EndstopR_Min      = 32; 
const uint8_t pin_EndstopZ_Max      = 34; 
const uint8_t pin_EndstopZ_Min      = 36; 

const uint8_t pin_HomingSensor      = 28; 

const uint8_t pin_VacuumSensor      = A0; 

enum StateENUM{
  RESET,
  ERROR,
  NEXT_LABEL,
  IDLE
} state;

TMC2209 stepper_driver;

AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

L293D swingDriver(7, 8, 9);


void deployProbe();
void retractProbe();


enum HomingENUM{ Probe, Z_Max, Z_Min };

// sensor -> { Probe, Z_Max, Z_Min }
// return : 0 -> clear, 1 -> sensor, 2 -> Z_MIN/MAX
uint8_t checkEndstops(HomingENUM sensor);

// sensor -> { Probe, Z_Max, Z_Min }
// return : false -> OK, true -> NG
bool stepperHomeTo(HomingENUM sensor);

// return : false -> OK, true -> NG
bool rotateSwing(bool toRight, bool checkVacuum = false);

void deployClamp();
void retractClamp();



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


  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }

  //Stepper Accelaration Config
  stepper.setEnablePin(4);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION);

  swingDriver.begin(true);

  pinMode(pin_ValveVaccum,      OUTPUT);
  pinMode(pin_ValveProbe,       OUTPUT);
  pinMode(pin_ValveClampSmall,  OUTPUT);
  pinMode(pin_ValveClampBig,    OUTPUT);

  pinMode(pin_EndstopR_Max,     INPUT);
  pinMode(pin_EndstopR_Min,     INPUT);
  pinMode(pin_EndstopZ_Max,     INPUT);
  pinMode(pin_EndstopZ_Min,     INPUT);
  pinMode(pin_HomingSensor,     INPUT); 
}

void loop()
{


switch (state) {
case RESET:
  retractClamp();
  digitalWrite(pin_ValveVaccum, HIGH);
  if (rotateSwing(true) == 1) {
    state = ERROR;
    Serial.println("ERROR: Swing Blocked");
    return;
  }
  if (stepperHomeTo(HomingENUM::Z_Min) == 1) {
    state = ERROR;
    Serial.println("ERROR: Homing Failed");
    return;
  }

  state = IDLE;

  break;
case NEXT_LABEL:
  
  
  
  
  break;
case ERROR:
  
  
  
  
  break;
case IDLE:
  Serial.print("STATE: IDLE");
  break;
default:
  break;
}

}



void deployProbe() {
  digitalWrite(pin_ValveProbe, HIGH);
  delay(300);
}

void retractProbe() {
  digitalWrite(pin_ValveProbe, LOW);
  delay(300);
}

uint8_t checkEndstops(HomingENUM sensor){
  uint8_t pin;
  switch (sensor) {
  case Z_Max:
    pin = pin_EndstopZ_Max; break;
  case Probe:
    pin = pin_HomingSensor; break;
  case Z_Min:
    pin = pin_EndstopZ_Min; break;
    break;
  default:
    break;
  }

  if (digitalRead(pin) == LOW) { return 1; }
  if (digitalRead(pin_EndstopZ_Max) == LOW || digitalRead(pin_EndstopZ_Min) == LOW) { return 2; }
  return 0;
}

// HomingENUM{ Probe, Z_Max, Z_Min }
bool stepperHomeTo(HomingENUM sensor) {
  uint8_t pin;
  bool dir;
  switch (sensor) {
  case Z_Max:
    pin = pin_EndstopZ_Max; dir = true; break;
  case Probe:
    pin = pin_HomingSensor; dir = true; break;
  case Z_Min:
    pin = pin_EndstopZ_Min; dir = false; break;
    break;
  default:
    break;
  }

  stepper.move(dir ? STEPS_PER_MM * MAX_TRAVEL : -STEPS_PER_MM * MAX_TRAVEL);
  while(checkEndstops(sensor) == 0 && stepper.currentPosition() == stepper.targetPosition()) {
    stepper.run();
  };

  if (stepper.currentPosition() == stepper.targetPosition() || checkEndstops(sensor) == 2) return true;
  stepper.moveTo(stepper.currentPosition());
  stepper.runToPosition();

  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.move(!dir ? STEPS_PER_MM * 2 : -STEPS_PER_MM * 2);

  while(digitalRead(pin) == LOW && stepper.currentPosition() == stepper.targetPosition()) {
    stepper.run();
  };

  stepper.setMaxSpeed(MAX_TRAVEL);

  if (stepper.currentPosition() == stepper.targetPosition()) return true;

  stepper.setCurrentPosition(0);
  return false;
}

bool rotateSwing(bool toRight, bool checkVacuum = false) {
  uint32_t time = millis();

  swingDriver.SetMotorSpeed(toRight ? 50 : -50);
  while(millis() < time + 750) {
    if (digitalRead(toRight ? pin_EndstopR_Max : pin_EndstopR_Min) == LOW) {
        delay(100);
        swingDriver.Stop();
        return false;
    };
  }
  return true;



}

void deployClamp() {
  digitalWrite(pin_ValveClampSmall, HIGH);
  delay(200);
  digitalWrite(pin_ValveClampBig, HIGH);
  delay(200);
}

void retractClamp(){
  digitalWrite(pin_ValveClampSmall, LOW);
  delay(200);
  digitalWrite(pin_ValveClampBig, LOW);
  delay(200);
}
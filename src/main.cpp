#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>
#include <L293D.h>
#include <ArduinoJson.h>
#include "EEPROM-Storage.h"


HardwareSerial & serial_stream = Serial3;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
EEPROMStorage<uint8_t> RUN_CURRENT_PERCENT(0, 100);

EEPROMStorage<uint32_t> STEPS_PER_MM(2,320);
EEPROMStorage<uint32_t> MAX_TRAVEL(7, STEPS_PER_MM * 120);
EEPROMStorage<uint32_t> MAX_SPEED(12, STEPS_PER_MM * 5);
EEPROMStorage<uint32_t> HOMING_SPEED(17, STEPS_PER_MM * 1);
EEPROMStorage<uint32_t> MAX_ACCELERATION(22, STEPS_PER_MM * 300);
EEPROMStorage<uint16_t> VACUUM_DELAY(27, 400);
EEPROMStorage<uint16_t> PROBE_DELAY(30, 500);
EEPROMStorage<uint16_t> CLAMP_DELAY(33, 300);



// Pinout
const uint8_t pin_ValveVacuum       = 42; 
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
  FIRST_LABEL,
  NEXT_LABEL,
  IDLE
} state;

TMC2209 stepper_driver;

AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

L293D swingDriver(7, 8, 9);

String readBuf;
bool firstAfterRestart = true;


void deployProbe();
void retractProbe();


enum HomingENUM{ Probe, Z_Max, Z_Min };

// sensor -> { Probe, Z_Max, Z_Min }
// return : 0 -> clear, 1 -> sensor, 2 -> Z_MIN/MAX
uint8_t checkEndstops(HomingENUM sensor);

// sensor -> { Probe, Z_Max, Z_Min }
// return : false -> NG, true -> OK
bool stepperHomeTo(HomingENUM sensor);

// return : false -> NG, true -> OK
bool rotateSwing(bool toRight, bool checkVacuum = false);

void deployClamp();
void retractClamp();

void parseJSON();



void setup()
{

  Serial.begin(115200);
  Serial.setTimeout(60000);
  while (Serial.availableForWrite() == 0) {}

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
  }

  //Stepper Acceleration Config
  stepper.setEnablePin(4);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION);

  swingDriver.begin(true);

  pinMode(pin_ValveVacuum,      OUTPUT);
  pinMode(pin_ValveProbe,       OUTPUT);
  pinMode(pin_ValveClampSmall,  OUTPUT);
  pinMode(pin_ValveClampBig,    OUTPUT);

  pinMode(pin_EndstopR_Max,     INPUT);
  pinMode(pin_EndstopR_Min,     INPUT);
  pinMode(pin_EndstopZ_Max,     INPUT);
  pinMode(pin_EndstopZ_Min,     INPUT);
  pinMode(pin_HomingSensor,     INPUT); 

  state = ERROR;
}

void loop()
{


switch (state) {
case RESET:
  retractClamp();
  retractProbe();
  digitalWrite(pin_ValveVacuum, LOW);
  if (!rotateSwing(true)) {
    state = ERROR;
    Serial.println("ERROR: Swing Blocked");
    break;
  }
  if (!stepperHomeTo(HomingENUM::Z_Min)) {
    state = ERROR;
    Serial.println("ERROR: Homing Failed");
    break;
  }
  firstAfterRestart = true;
  state = IDLE;

  break;
case NEXT_LABEL:
  retractClamp();
  if (!rotateSwing(false)) {
    state = ERROR;
    Serial.println("ERROR: Swing Blocked");
    break;
  }
  digitalWrite(pin_ValveVacuum, HIGH);
  delay(VACUUM_DELAY);
  if (!rotateSwing(true)) {
    state = ERROR;
    Serial.println("ERROR: Swing Blocked");
    break;
  }
  digitalWrite(pin_ValveVacuum, LOW);
  delay(VACUUM_DELAY);
  
case FIRST_LABEL:
  deployProbe();
  if (!stepperHomeTo(HomingENUM::Probe)) {
    state = ERROR;
    Serial.println("ERROR: Homing to Probe Failed");
    return;
  }
  deployClamp();
  retractProbe();
  Serial.println("READY");
  
  firstAfterRestart = false;
  state = IDLE;
  break;

case ERROR:
  retractClamp();
  retractProbe();
  digitalWrite(pin_ValveVacuum, LOW);

  firstAfterRestart = true;

  while(state != RESET){
    Serial.println("STATE: ERROR | send 'RESET' to continue");
    Serial.print("CMD> ");
    while (Serial.available() == 0) {}
    readBuf = Serial.readStringUntil('\n');
    Serial.println();
    
    readBuf.trim();

    if(readBuf.equals("RESET")) state = RESET;
  }


  break;
case IDLE:
  Serial.println("STATE: IDLE");
  Serial.print("CMD> ");
  while (Serial.available() == 0) {}
  readBuf = Serial.readStringUntil('\n');
  Serial.println();
  readBuf.trim();

  if(readBuf.equals("RESET")) {
    state = RESET;
    break;
  } else 
  if (readBuf.equals("NEXT_LABEL"))
  {
    state = firstAfterRestart ? FIRST_LABEL : NEXT_LABEL;
    break;
  } else if (readBuf.equals("JSON")) {
    parseJSON();
    break;
  }else {
    Serial.println("ERROR: Unknown command");
    state = ERROR;
    break;
  }
default:
  break;
}

}



void deployProbe() {
  digitalWrite(pin_ValveProbe, HIGH);
  delay(PROBE_DELAY);
}

void retractProbe() {
  digitalWrite(pin_ValveProbe, LOW);
  delay(PROBE_DELAY);
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

  if (stepper.currentPosition() == stepper.targetPosition() || checkEndstops(sensor) == 2) return false;
  stepper.moveTo(stepper.currentPosition());
  stepper.runToPosition();

  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.move(!dir ? STEPS_PER_MM * 2 : -STEPS_PER_MM * 2);

  while(digitalRead(pin) == LOW && stepper.currentPosition() == stepper.targetPosition()) {
    stepper.run();
  };

  stepper.setMaxSpeed(MAX_TRAVEL);

  if (stepper.currentPosition() == stepper.targetPosition()) return false;

  stepper.setCurrentPosition(0);
  return true;
}

bool rotateSwing(bool toRight, bool checkVacuum) {
  uint32_t time = millis();

  swingDriver.SetMotorSpeed(toRight ? 50 : -50);
  while(millis() < time + 750) {
    if (digitalRead(toRight ? pin_EndstopR_Max : pin_EndstopR_Min) == LOW) {
        delay(100);
        swingDriver.Stop();
        return true;
    };
  }
  return false;

}

void deployClamp() {
  digitalWrite(pin_ValveClampSmall, HIGH);
  delay(CLAMP_DELAY);
  digitalWrite(pin_ValveClampBig, HIGH);
  delay(CLAMP_DELAY);
}

void retractClamp(){
  digitalWrite(pin_ValveClampBig, LOW);
  delay(CLAMP_DELAY);
  digitalWrite(pin_ValveClampSmall, LOW);
  delay(CLAMP_DELAY);
}

void parseJSON() {
JsonDocument doc;

doc["RUN_CURRENT_PERCENT"] = RUN_CURRENT_PERCENT.get();
doc["STEPS_PER_MM"] = STEPS_PER_MM.get();
doc["MAX_TRAVEL"] = MAX_TRAVEL.get();
doc["MAX_SPEED"] = MAX_SPEED.get();
doc["HOMING_SPEED"] = HOMING_SPEED.get();
doc["MAX_ACCELERATION"] = MAX_ACCELERATION.get();
doc["VACUUM_DELAY"] = VACUUM_DELAY.get();
doc["PROBE_DELAY"] = PROBE_DELAY.get();
doc["CLAMP_DELAY"] = CLAMP_DELAY.get();

serializeJson(doc, Serial);
Serial.println();

DeserializationError error = deserializeJson(doc, Serial);

if (error) {
  Serial.print(F("deserializeJson() failed: "));
  Serial.println(error.f_str());
  return;
} else {
  Serial.println("OK.");
}

RUN_CURRENT_PERCENT.set(doc["RUN_CURRENT_PERCENT"].as<uint8_t>());
STEPS_PER_MM.set(doc["STEPS_PER_MM"].as<uint32_t>());
MAX_TRAVEL.set(doc["MAX_TRAVEL"].as<uint32_t>());
MAX_SPEED.set(doc["MAX_SPEED"].as<uint32_t>());
HOMING_SPEED.set(doc["HOMING_SPEED"].as<uint32_t>());
MAX_ACCELERATION.set(doc["MAX_ACCELERATION"].as<uint32_t>());
VACUUM_DELAY.set(doc["VACUUM_DELAY"].as<uint16_t>());
PROBE_DELAY.set(doc["PROBE_DELAY"].as<uint16_t>());
CLAMP_DELAY.set(doc["CLAMP_DELAY"].as<uint16_t>());

}
#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>
#include <L293D.h>
#include <ArduinoJson.h>
#include "EEPROM-Storage.h"

HardwareSerial &serial_stream = Serial3;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
EEPROMStorage<uint8_t> RUN_CURRENT_PERCENT(0, 100);

EEPROMStorage<uint32_t> STEPS_PER_MM(2, 160);
EEPROMStorage<uint32_t> MAX_TRAVEL(7, STEPS_PER_MM * 120);
EEPROMStorage<uint32_t> MAX_SPEED(12, STEPS_PER_MM * 15);
EEPROMStorage<uint32_t> HOMING_SPEED(17, STEPS_PER_MM * 3);
EEPROMStorage<uint32_t> MAX_ACCELERATION(22, STEPS_PER_MM * 2500);
EEPROMStorage<uint16_t> VACUUM_DELAY(27, 400);
EEPROMStorage<uint16_t> PROBE_DELAY(30, 500);
EEPROMStorage<uint16_t> CLAMP_DELAY(33, 300);

// Pinout
const uint8_t pin_ValveVacuum = 48;
const uint8_t pin_ValveProbe = 42;
const uint8_t pin_ValveClampSmall = 46;
const uint8_t pin_ValveClampBig = 44;

const uint8_t pin_EndstopR_Max = 30;
const uint8_t pin_EndstopR_Min = 32;
const uint8_t pin_EndstopZ_Max = 34;
const uint8_t pin_EndstopZ_Min = 36;

const uint8_t pin_HomingSensor = 28;

const uint8_t pin_VacuumSensor = A0;

enum StateENUM
{
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

enum HomingENUM
{
  Probe,
  Z_Max,
  Z_Min
};

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
  while (Serial.availableForWrite() == 0)
  {
  }

  // Stepper Driver config
  stepper_driver.setup(serial_stream);

  stepper_driver.setMicrostepsPerStepPowerOfTwo(4); // 16 microsteps per step
  // stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setRunCurrent(100);

  // stepper_driver.setCoolStepCurrentIncrement(TMC2209::CURRENT_INCREMENT_8);
  // stepper_driver.setCoolStepMeasurementCount(TMC2209::MEASUREMENT_COUNT_1);
  // stepper_driver.setCoolStepDurationThreshold(2000);
  // stepper_driver.enableCoolStep(1, 1);

  stepper_driver.enable();

  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating!");
  }

  // Stepper Acceleration Config
  stepper.setEnablePin(4);
  stepper.setPinsInverted(false, false, true);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION);

  swingDriver.begin(true);

  pinMode(pin_ValveVacuum, OUTPUT);
  pinMode(pin_ValveProbe, OUTPUT);
  pinMode(pin_ValveClampSmall, OUTPUT);
  pinMode(pin_ValveClampBig, OUTPUT);

  pinMode(pin_EndstopR_Max, INPUT);
  pinMode(pin_EndstopR_Min, INPUT);
  pinMode(pin_EndstopZ_Max, INPUT_PULLUP);
  pinMode(pin_EndstopZ_Min, INPUT_PULLUP);
  pinMode(pin_HomingSensor, INPUT_PULLUP);

  state = ERROR;
  stepper.enableOutputs();

  // while (true)
  // {
  //   Serial.println("Homing Z Max...");
  //   if (!stepperHomeTo(HomingENUM::Z_Max))
  //   {
  //     state = ERROR;
  //     Serial.println("ERROR: Homing Failed");
  //     break;
  //   }

  //   delay(500);

  //   Serial.println("Homing Z Min...");
  //   if (!stepperHomeTo(HomingENUM::Z_Min))
  //   {
  //     state = ERROR;
  //     Serial.println("ERROR: Homing Failed");
  //     break;
  //   }

  //   delay(500);
  // }

  // Serial.println("End...");
  // while (true)
  // {
  //   Serial.print(digitalRead(pin_EndstopZ_Max));
  //   Serial.print(digitalRead(pin_EndstopZ_Min));
  //   Serial.println(digitalRead(pin_HomingSensor));
  //   delay(100);
  // }

  // bool running = true;
  // bool side = true;
  // uint16_t counter = 0;
  // while (running)
  // {

  //   digitalWrite(pin_ValveVacuum, !side);
  //   delay(VACUUM_DELAY);
  //   if (!rotateSwing(side))
  //   {
  //     running = false;
  //     digitalWrite(pin_ValveVacuum, LOW);
  //     Serial.println("ERROR: Swing Blocked");
  //   }
  //   Serial.print("Counter: ");
  //   Serial.println(counter++);
  //   delay(1000);
  //   side = !side;
  // }
}

void loop()
{

  switch (state)
  {
  case RESET:
    retractClamp();
    retractProbe();
    digitalWrite(pin_ValveVacuum, LOW);
    if (!rotateSwing(true))
    {
      state = ERROR;
      Serial.println("ERROR: Swing Blocked");
      break;
    }
    if (!stepperHomeTo(HomingENUM::Z_Min))
    {
      state = ERROR;
      Serial.println("ERROR: Homing Failed");
      break;
    }
    firstAfterRestart = true;
    state = IDLE;

    break;
  case NEXT_LABEL:
    retractClamp();
    if (!rotateSwing(false))
    {
      state = ERROR;
      Serial.println("ERROR: Swing Blocked");
      break;
    }
    digitalWrite(pin_ValveVacuum, HIGH);
    delay(VACUUM_DELAY);
    stepper.move(-STEPS_PER_MM * 2);
    stepper.runToPosition();
    delay(500);
    stepper.move(STEPS_PER_MM * 2);
    stepper.runToPosition();

    if (!rotateSwing(true))
    {
      state = ERROR;
      Serial.println("ERROR: Swing Blocked");
      break;
    }
    digitalWrite(pin_ValveVacuum, LOW);
    delay(VACUUM_DELAY);

  case FIRST_LABEL:
    deployProbe();
    if (!stepperHomeTo(HomingENUM::Probe))
    {
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

    while (state != RESET)
    {
      Serial.println("STATE: ERROR | send 'RESET' to continue");
      Serial.print("CMD> ");
      while (Serial.available() == 0)
      {
      }
      readBuf = Serial.readStringUntil('\n');
      Serial.println();

      readBuf.trim();

      if (readBuf.equals("RESET"))
        state = RESET;
    }

    break;
  case IDLE:
    Serial.println("STATE: IDLE");
    Serial.print("CMD> ");
    while (Serial.available() == 0)
    {
    }
    readBuf = Serial.readStringUntil('\n');
    Serial.println();
    readBuf.trim();

    if (readBuf.equals("RESET"))
    {
      state = RESET;
      break;
    }
    else if (readBuf.equals("NEXT_LABEL"))
    {
      state = firstAfterRestart ? FIRST_LABEL : NEXT_LABEL;
      break;
    }
    else if (readBuf.equals("JSON"))
    {
      parseJSON();
      break;
    }
    else
    {
      Serial.println("ERROR: Unknown command");
      state = ERROR;
      break;
    }
  default:
    break;
  }
}

void deployProbe()
{
  digitalWrite(pin_ValveProbe, HIGH);
  delay(PROBE_DELAY);
}

void retractProbe()
{
  digitalWrite(pin_ValveProbe, LOW);
  delay(PROBE_DELAY);
}

uint8_t checkEndstops(HomingENUM sensor)
{
  uint8_t pin;
  switch (sensor)
  {
  case Z_Max:
    pin = pin_EndstopZ_Max;
    break;
  case Probe:
    pin = pin_HomingSensor;
    break;
  case Z_Min:
    pin = pin_EndstopZ_Min;
    break;
  default:
    break;
  }

  if (digitalRead(pin))
  {
    return 1;
  }
  if (digitalRead(pin_EndstopZ_Max) == HIGH || digitalRead(pin_EndstopZ_Min) == HIGH)
  {
    return 2;
  }
  return 0;
}

bool stepperHomeTo(HomingENUM sensor)
{
  bool dir;
  bool homing_result = true;
  int32_t current = 0;
  switch (sensor)
  {
  case Z_Max:

    dir = true;
    break;
  case Probe:

    dir = true;
    break;
  case Z_Min:

    dir = false;
    break;
  default:
    break;
  }
  if (dir)
  {
    stepper.move(-STEPS_PER_MM * MAX_TRAVEL);
  }
  else
  {
    stepper.move(STEPS_PER_MM * MAX_TRAVEL);
  }

  while (homing_result)
  {
    if (checkEndstops(sensor) == 1)
    {
      break;
    }
    if (checkEndstops(sensor) == 2)
    {
      Serial.println("STEPPER: Endstop triggered");
      homing_result = false;
      break;
    }
    if (stepper.currentPosition() == stepper.targetPosition())
    {
      Serial.println("STEPPER: Software limit trigger");
      homing_result = false;
      break;
    }
    stepper.run();
  };

  current = stepper.currentPosition();
  stepper.stop();
  stepper.runToPosition();
  stepper.runToNewPosition(dir ? current + (STEPS_PER_MM * 2) : current - (STEPS_PER_MM * 2));

  stepper.setCurrentPosition(0);

  return homing_result;
}

bool rotateSwing(bool toRight, bool checkVacuum)
{
  uint32_t time = millis();
  swingDriver.SetMotorSpeed(toRight ? 40 : -40);

  while (millis() < time + 1500)
  {
    if (digitalRead(toRight ? pin_EndstopR_Max : pin_EndstopR_Min) == LOW)
    {
      delay(100);
      swingDriver.Stop();
      return true;
    }
  }
  swingDriver.Stop();
  return false;
}

void deployClamp()
{
  digitalWrite(pin_ValveClampSmall, HIGH);
  delay(CLAMP_DELAY);
  digitalWrite(pin_ValveClampBig, HIGH);
  delay(CLAMP_DELAY);
}

void retractClamp()
{
  digitalWrite(pin_ValveClampBig, LOW);
  delay(CLAMP_DELAY);
  digitalWrite(pin_ValveClampSmall, LOW);
  delay(CLAMP_DELAY);
}

void parseJSON()
{
  JsonDocument doc;

  doc["RUN_CURRENT_PERCENT"] = RUN_CURRENT_PERCENT.get();
  doc["STEPS_PER_MM"] = STEPS_PER_MM.get();
  doc["MAX_TRAVEL"] = MAX_TRAVEL.get() / STEPS_PER_MM;
  doc["MAX_SPEED"] = MAX_SPEED.get() / STEPS_PER_MM;
  doc["HOMING_SPEED"] = HOMING_SPEED.get() / STEPS_PER_MM;
  doc["MAX_ACCELERATION"] = MAX_ACCELERATION.get() / STEPS_PER_MM;
  doc["VACUUM_DELAY"] = VACUUM_DELAY.get();
  doc["PROBE_DELAY"] = PROBE_DELAY.get();
  doc["CLAMP_DELAY"] = CLAMP_DELAY.get();

  serializeJson(doc, Serial);
  Serial.println();

  DeserializationError error = deserializeJson(doc, Serial);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  else
  {
    Serial.println("OK.");
  }

  RUN_CURRENT_PERCENT.set(doc["RUN_CURRENT_PERCENT"].as<uint8_t>());
  STEPS_PER_MM.set(doc["STEPS_PER_MM"].as<uint32_t>());
  MAX_TRAVEL.set(doc["MAX_TRAVEL"].as<uint32_t>() * STEPS_PER_MM);
  MAX_SPEED.set(doc["MAX_SPEED"].as<uint32_t>() * STEPS_PER_MM);
  HOMING_SPEED.set(doc["HOMING_SPEED"].as<uint32_t>() * STEPS_PER_MM);
  MAX_ACCELERATION.set(doc["MAX_ACCELERATION"].as<uint32_t>() * STEPS_PER_MM);
  VACUUM_DELAY.set(doc["VACUUM_DELAY"].as<uint16_t>());
  PROBE_DELAY.set(doc["PROBE_DELAY"].as<uint16_t>());
  CLAMP_DELAY.set(doc["CLAMP_DELAY"].as<uint16_t>());

  // default: {"RUN_CURRENT_PERCENT":100,"STEPS_PER_MM":320,"MAX_TRAVEL":38400,"MAX_SPEED":1600,"HOMING_SPEED":320,"MAX_ACCELERATION":96000,"VACUUM_DELAY":400,"PROBE_DELAY":500,"CLAMP_DELAY":300}
}
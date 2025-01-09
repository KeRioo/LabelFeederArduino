#include <Arduino.h>

#include "Swing.h"
#include "Stepper.h"
#include "Pneumatic.h"

#ifdef __AVR_ATmega2560__
#include "config_mega.h"
#else
#include "config_minima.h"
#endif

enum StateENUM
{
  RESET,
  ERROR,
  FIRST_LABEL,
  NEXT_LABEL,
  IDLE
} state;

Stepper stepper;
Swing swing;

String readBuf;
bool firstAfterRestart = true;
uint8_t pickCounter = 0;

/**
 * Handles errors by setting the state to ERROR and printing a message.
 * @param isError Indicates whether an error has occurred.
 * @param message The error message to print (optional).
 * @return True if no error, false otherwise.
 */
bool errorHandler(bool isError, String message = "")
{
  if (!isError)
  {
    state = ERROR;
    if (message.length() > 0)
    {
      Serial.println(message);
    }
  }
  return !isError;
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(4294967295);
  while (Serial.availableForWrite() == 0)
  {
  }
  delay(1000);

  while (!stepper.setup())
  {
    Serial.println("ERROR<06>: No communication with stepper motor");
    delay(1000);
  }

  pinMode(pin_ValveVacuum, OUTPUT);
  pinMode(pin_ValveProbe, OUTPUT);
  pinMode(pin_ValveClampSmall, OUTPUT);
  pinMode(pin_ValveClampBig, OUTPUT);

  pinMode(pin_EndstopR_Max, INPUT_PULLUP);
  pinMode(pin_EndstopR_Min, INPUT_PULLUP);
  pinMode(pin_EndstopZ_Max, INPUT_PULLUP);
  pinMode(pin_EndstopZ_Min, INPUT_PULLUP);
  pinMode(pin_HomingSensor, INPUT_PULLUP);
  pinMode(pin_VacuumSensor, INPUT_PULLUP);

  state = ERROR;
}

void loop()
{
  switch (state)
  {
  case RESET:
    retractClamp();
    retractProbe();
    vacuumOFF();

    if (errorHandler(swing.rotate(Right)))
      break;

    if (errorHandler(stepper.homeTo(HomingENUM::Z_Min)))
      break;

    firstAfterRestart = true;
    state = IDLE;

    break;
  case NEXT_LABEL:
    retractClamp();

    if (errorHandler(swing.rotate(Left)))
      break;

    pickCounter = 0;

    if (vacuumON() == false)
    {
      while (digitalRead(pin_VacuumSensor))
      {
        stepper.z_hop(2);
        if (errorHandler(pickCounter++ < 2), "ERROR<05>: No labels in the feeder")
        {
          vacuumOFF();
          return;
        }
      }
    }

    if (errorHandler(swing.rotate(Right, true)))
      break;
    delay(300);
    vacuumOFF();

  case FIRST_LABEL:
    deployProbe();

    if (errorHandler(stepper.homeTo(HomingENUM::Probe)))
      return;

    deployClamp();
    retractProbe();
    Serial.println("READY");

    firstAfterRestart = false;
    state = IDLE;
    break;

  case ERROR:
    retractClamp();
    retractProbe();
    vacuumOFF();

    firstAfterRestart = true;

    while (state != RESET)
    {
      Serial.println("STATE<00>: ERROR | send 'RESET' to continue");

      readBuf = Serial.readStringUntil('\n');
      Serial.println();

      readBuf.trim();

      if (readBuf.equals("RESET") || readBuf.equals("***"))
        state = RESET;
    }

    break;
  case IDLE:
    Serial.println("STATE<01>: IDLE");

    readBuf = Serial.readStringUntil('\n');
    Serial.println();
    readBuf.trim();

    if (readBuf.equals("RESET") || readBuf.equals("***"))
    {
      state = RESET;
      break;
    }
    else if (readBuf.equals("NEXT_LABEL") || readBuf.equals("+++"))
    {
      state = firstAfterRestart ? FIRST_LABEL : NEXT_LABEL;
      break;
    }
    else
    {
      errorHandler(false, "ERROR<00>: Unknown command");
      break;
    }
  default:
    break;
  }
}

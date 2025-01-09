#ifndef _Pneumatic_h
#define _Pneumatic_h

#include <Arduino.h>

#ifdef __AVR_ATmega2560__
#include "config_mega.h"
#else
#include "config_minima.h"
#endif

/**
 * Deploys the probe by activating the corresponding valve.
 */
void deployProbe()
{
    digitalWrite(pin_ValveProbe, HIGH);
    delay(PROBE_DELAY);
}

/**
 * Retracts the probe by deactivating the corresponding valve.
 */
void retractProbe()
{
    digitalWrite(pin_ValveProbe, LOW);
    delay(PROBE_DELAY);
}

/**
 * Deploys the clamp by activating the corresponding valves.
 */
void deployClamp()
{
    digitalWrite(pin_ValveClampSmall, HIGH);
    delay(CLAMP_DELAY);
    digitalWrite(pin_ValveClampBig, HIGH);
    delay(CLAMP_DELAY);
}

/**
 * Retracts the clamp by deactivating the corresponding valves.
 */
void retractClamp()
{
    digitalWrite(pin_ValveClampBig, LOW);
    delay(CLAMP_DELAY);
    digitalWrite(pin_ValveClampSmall, LOW);
    delay(CLAMP_DELAY);
}

/**
 * Turns on the vacuum by activating the corresponding valve.
 * @return True if the vacuum sensor detects low pressure, false otherwise.
 */
bool vacuumON()
{
    digitalWrite(pin_ValveVacuum, HIGH);

    unsigned long time = millis() + VACUUM_ERROR_DELAY;
    while (millis() < time)
    {
        if (digitalRead(pin_VacuumSensor) == LOW)
        {
            return true;
        }
    }
    return false;
}

/**
 * Turns off the vacuum by deactivating the corresponding valve.
 */
void vacuumOFF()
{
    digitalWrite(pin_ValveVacuum, LOW);

    unsigned long time = millis() + VACUUM_ERROR_DELAY;
    while (millis() < time && digitalRead(pin_VacuumSensor) == LOW)
    {
    }
}

#endif

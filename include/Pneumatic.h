#ifndef _Pneumatic_h
#define _Pneumatic_h

#include <Arduino.h>

#ifdef __AVR_ATmega2560__
#include "config_mega.h"
#else
#include "config_minima.h"
#endif

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

void vacuumOFF()
{
    digitalWrite(pin_ValveVacuum, LOW);

    unsigned long time = millis() + VACUUM_ERROR_DELAY;
    while (millis() < time && digitalRead(pin_VacuumSensor) == LOW)
    {
    }
}

#endif

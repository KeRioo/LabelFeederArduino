#ifndef SWING_H
#define SWING_H

#include <Arduino.h>
#include <L293D.h>

#ifdef __AVR_ATmega2560__
#include "config_mega.h"
#else
#include "config_minima.h"
#endif

// left = false, right = true
enum SwingSideENUM
{
    Left,
    Right
};

class Swing
{
private:
    L293D motor;

public:
    Swing();
    /**
     * Rotates the swing to the specified side.
     * @param side The side to rotate to (Left or Right).
     * @param checkVacuum Whether to check the vacuum sensor during rotation.
     * @return True if the rotation was successful, false otherwise.
     */
    bool rotate(SwingSideENUM side, bool checkVacuum = false);
};

Swing::Swing() : motor(pin_L293_A, pin_L293_B, pin_L293_Ena)
{
    motor.begin(true);
}

bool Swing::rotate(SwingSideENUM side, bool checkVacuum)
{
    uint32_t time = millis();
    motor.SetMotorSpeed(side == Right ? 50 : -50);

    while (millis() < time + 2000)
    {
        if (digitalRead(side == Right ? pin_EndstopR_Max : pin_EndstopR_Min) == LOW)
        {
            delay(100);
            motor.Stop();
            return true;
        }
        if (checkVacuum && digitalRead(pin_VacuumSensor))
        {
            Serial.println("ERROR<03>: Low Vacuum during swing movement");
            motor.Stop();
            return false;
        }
    }
    Serial.println("ERROR<04>: Swing Blocked");
    motor.Stop();
    return false;
}

#endif
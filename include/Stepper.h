#ifndef _Stepper_h
#define _Stepper_h

#include <Arduino.h>
#include <TMC2209.h>
#include <MobaTools.h>

#ifdef __AVR_ATmega2560__
#include "config_mega.h"
#else
#include "config_minima.h"
#endif

enum HomingENUM
{
    Probe,
    Z_Max,
    Z_Min
};

class Stepper
{
private:
    TMC2209 driver;

    MoToStepper stepper; //

public:
    Stepper(/* args */);

    /**
     * Setup the stepper driver and motor.
     * @return True if setup is successful, false otherwise.
     */
    bool setup();

    /**
     * Check the endstop status for a given sensor.
     * @param sensor The sensor to check (Probe, Z_Max, Z_Min).
     * @return 0 if clear, 1 if sensor triggered, 2 if Z_MIN/MAX triggered.
     */
    uint8_t checkEndstop(HomingENUM sensor);

    /**
     * Home the stepper motor to a given sensor.
     * @param sensor The sensor to home to (Probe, Z_Max, Z_Min).
     * @return True if homing is successful, false otherwise.
     */
    bool homeTo(HomingENUM sensor);

    /**
     * Perform a Z-hop movement.
     * @param mm The distance in millimeters to hop.
     */
    void z_hop(uint32_t mm);
};

Stepper::Stepper(/* args */) : driver(), stepper(3200, STEPDIR)
{
}

bool Stepper::setup()
{
    // Stepper Driver config
    driver.setup(serial_stream);

    driver.setMicrostepsPerStepPowerOfTwo(4); // 16 microsteps per step
    driver.setRunCurrent(100);
    driver.setHoldCurrent(10);

    driver.setHardwareEnablePin(4);
    driver.enableCoolStep();

    driver.enable();

    if (not driver.isSetupAndCommunicating())
    {
        Serial.println("Stepper driver not setup and communicating!");
        return false;
    }

    // Stepper Acceleration Config
    stepper.attach(pin_TMC_Step, pin_TMC_Dir);          // assign step/dir pins
    stepper.setSpeedSteps(MAX_SPEED, MAX_ACCELERATION); // initial value of speed

    return true;
}

uint8_t Stepper::checkEndstop(HomingENUM sensor)
{
    uint8_t pin;
    bool NOpen = true;
    switch (sensor)
    {
    case Z_Max:
        pin = pin_EndstopZ_Max;
        break;
    case Probe:
        pin = pin_HomingSensor;
        NOpen = false;
        break;
    case Z_Min:
        pin = pin_EndstopZ_Min;
        break;
    default:
        break;
    }

    if (NOpen ? digitalRead(pin) : !digitalRead(pin))
    {
        return 1;
    }
    if (digitalRead(pin_EndstopZ_Max) == HIGH || digitalRead(pin_EndstopZ_Min) == HIGH)
    {
        return 2;
    }
    return 0;
}

bool Stepper::homeTo(HomingENUM sensor)
{
    bool dir;
    bool homing_result = true;
    stepper.setZero();
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
        stepper.doSteps(-STEPS_PER_MM * MAX_TRAVEL);
    }
    else
    {
        stepper.doSteps(STEPS_PER_MM * MAX_TRAVEL);
    }

    while (homing_result)
    {
        if (checkEndstop(sensor) == 1)
        {
            break;
        }
        if (checkEndstop(sensor) == 2 && sensor == Probe)
        {
            Serial.println("ERROR<01>: Limit switch was triggered");
            homing_result = false;
            break;
        }
        if (stepper.stepsToDo() == 0 && stepper.moving() == 0)
        {
            Serial.println("ERROR<02>: Stepper motor software limit");
            homing_result = false;
            break;
        }
    };
    stepper.stop();
    stepper.setZero();

    stepper.doSteps(dir ? (STEPS_PER_MM * 1) : (-STEPS_PER_MM * 1));
    while (stepper.moving())
    {
    }
    stepper.setZero(0);

    delay(200);

    return homing_result;
}

void Stepper::z_hop(uint32_t mm)
{
    stepper.setZero(0);
    stepper.doSteps(-STEPS_PER_MM * mm);
    while (stepper.moving())
    {
    }
    delay(200);

    stepper.doSteps(STEPS_PER_MM * mm);
    while (stepper.moving())
    {
    }
    stepper.setZero(0);
}

#endif
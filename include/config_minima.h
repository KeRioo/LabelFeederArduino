#ifndef CONFIG_MINIMA_H
#define CONFIG_MINIMA_H

#include "Arduino.h"

HardwareSerial &serial_stream = Serial1;

const uint8_t pin_ValveProbe = 11;
const uint8_t pin_ValveClampSmall = 10;
const uint8_t pin_ValveClampBig = 9;
const uint8_t pin_ValveVacuum = 8;

uint8_t pin_EndstopR_Max = A3;
uint8_t pin_EndstopR_Min = A2;
uint8_t pin_EndstopZ_Max = A5;
uint8_t pin_EndstopZ_Min = A4;

const uint8_t pin_HomingSensor = 12;

const uint8_t pin_VacuumSensor = A0;

const uint8_t pin_TMC_Step = 3;
const uint8_t pin_TMC_Dir = 2;
const uint8_t pin_TMC_Ena = 4;

uint8_t pin_L293_A = 5;
uint8_t pin_L293_B = 6;
const uint8_t pin_L293_Ena = 7;

const uint32_t STEPS_PER_MM = 160;
const uint32_t MAX_TRAVEL = STEPS_PER_MM * 120;
const uint32_t MAX_SPEED = STEPS_PER_MM * 80;
const uint32_t MAX_ACCELERATION = 10;
const uint16_t VACUUM_ERROR_DELAY = 500;
const uint16_t PROBE_DELAY = 500;
const uint16_t CLAMP_DELAY = 200;

bool stepper_direction = false;
bool swing_direction = true;
bool endstops_R_reversed = true;
bool endstops_Z_reversed = true;

void config_handler()
{
    if (swing_direction)
    {
        pin_L293_A = 6;
        pin_L293_B = 5;
    }
    if (endstops_R_reversed)
    {
        pin_EndstopR_Max = A2;
        pin_EndstopR_Min = A3;
    }
    if (endstops_Z_reversed)
    {
        pin_EndstopZ_Max = A4;
        pin_EndstopZ_Min = A5;
    }
}

#endif

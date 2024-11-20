#ifndef CONFIG_MEGA_H
#define CONFIG_MEGA_H

#include "Arduino.h"

HardwareSerial &serial_stream = Serial3;

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

const uint8_t pin_TMC_Step = 2;
const uint8_t pin_TMC_Dir = 3;
const uint8_t pin_TMC_Ena = 4;

const uint8_t pin_L293_A = 7;
const uint8_t pin_L293_B = 8;
const uint8_t pin_L293_Ena = 9;

uint32_t STEPS_PER_MM = 160;
uint32_t MAX_TRAVEL = STEPS_PER_MM * 120;
uint32_t MAX_SPEED = STEPS_PER_MM * 8;
uint32_t HOMING_SPEED = STEPS_PER_MM * 3;
uint32_t MAX_ACCELERATION = 10;
uint16_t VACUUM_ERROR_DELAY = 500;
uint16_t PROBE_DELAY = 500;
uint16_t CLAMP_DELAY = 300;
int VACUUM_THRESHOLD = 250;

#endif
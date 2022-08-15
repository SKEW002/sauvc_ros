#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include "Arduino.h"
#include <Servo.h>
#include "config.h"


void register_motor();
void stop_operation();
void test_motor();
void horizontal_movement(int *pwm);
void vertical_movement(int *pwm);

#endif

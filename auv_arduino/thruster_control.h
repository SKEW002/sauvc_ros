#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include "Arduino.h"
#include <Servo.h>
#include "config.h"


void register_motor();
void stop_operation();
void all_run_slow();
void horizontal_movement();
void vertical_movement();

#endif

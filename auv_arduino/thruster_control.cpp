#include "thruster_control.h"

Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;


/**
 * Register pins to each thrusters.
 */
void register_motor(){
  thruster1.attach(MOTOR_NO_1_PIN);
  thruster2.attach(MOTOR_NO_2_PIN);
  thruster3.attach(MOTOR_NO_3_PIN);
  thruster4.attach(MOTOR_NO_4_PIN);
  thruster5.attach(MOTOR_NO_5_PIN);
  thruster6.attach(MOTOR_NO_6_PIN);
  thruster7.attach(MOTOR_NO_7_PIN);
  thruster8.attach(MOTOR_NO_8_PIN);
}


/**
 * Stop motors.
 */
void stop_operation(){
    thruster1.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster2.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster3.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster4.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster5.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster6.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster7.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
    thruster8.writeMicroseconds(ESC_INPUT_FOR_STOP_SIGNAL);
}


/**
 * Motor test.
 */
void test_motor(){
    thruster1.writeMicroseconds(1550);
    thruster2.writeMicroseconds(1550);
    thruster3.writeMicroseconds(1550);
    thruster4.writeMicroseconds(1550);
    thruster5.writeMicroseconds(1550);
    thruster6.writeMicroseconds(1550);
    thruster7.writeMicroseconds(1550);
    thruster8.writeMicroseconds(1550);
}


/**
 * Forward, backward, translate and z axis rotation.
 */
void horizontal_movement(int speed1, int speed2, int speed3, int speed4){
    thruster1.writeMicroseconds(speed1);
    thruster2.writeMicroseconds(speed2);
    thruster3.writeMicroseconds(speed3);
    thruster4.writeMicroseconds(speed4);
}


/**
 * Surface or submerge.
 */
void vertical_movement(int speed5, int speed6, int speed7, int speed8){
    thruster5.writeMicroseconds(speed5);
    thruster6.writeMicroseconds(speed6);
    thruster7.writeMicroseconds(speed7);
    thruster8.writeMicroseconds(speed8);
}

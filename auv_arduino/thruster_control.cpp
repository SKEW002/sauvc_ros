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
    thruster1.writeMicroseconds(STOP_SIGNAL);
    thruster2.writeMicroseconds(STOP_SIGNAL);
    thruster3.writeMicroseconds(STOP_SIGNAL);
    thruster4.writeMicroseconds(STOP_SIGNAL);
    thruster5.writeMicroseconds(STOP_SIGNAL);
    thruster6.writeMicroseconds(STOP_SIGNAL);
    thruster7.writeMicroseconds(STOP_SIGNAL);
    thruster8.writeMicroseconds(STOP_SIGNAL);
}


/**
 * Motor test.
 */
void test_motor(){
    thruster1.writeMicroseconds(1550);
    delay(1000);
    thruster1.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster2.writeMicroseconds(1550);
    delay(1000);
    thruster2.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster3.writeMicroseconds(1550);
    delay(1000);
    thruster3.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster4.writeMicroseconds(1550);
    delay(1000);
    thruster4.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster5.writeMicroseconds(1550);
    delay(1000);
    thruster5.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster6.writeMicroseconds(1550);
    delay(1000);
    thruster6.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster7.writeMicroseconds(1550);
    delay(1000);
    thruster7.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    thruster8.writeMicroseconds(1550);
    delay(1000);
    thruster8.writeMicroseconds(STOP_SIGNAL);
    delay(500);
    
//    thruster2.writeMicroseconds(1550);
//    thruster3.writeMicroseconds(1550);
//    thruster4.writeMicroseconds(1550);
//    thruster5.writeMicroseconds(1550);
//    thruster6.writeMicroseconds(1550);
//    thruster7.writeMicroseconds(1550);
//    thruster8.writeMicroseconds(1550);
}


/**
 * Forward, backward, translate and z axis rotation.
 */
void horizontal_movement(int *pwm){
    thruster1.writeMicroseconds(pwm[0]);
    thruster2.writeMicroseconds(pwm[1]);
    thruster3.writeMicroseconds(pwm[2]);
    thruster4.writeMicroseconds(pwm[3]);
}


/**
 * Surface or submerge.
 */
void vertical_movement(int *pwm){
    thruster5.writeMicroseconds(pwm[0]);
    thruster6.writeMicroseconds(pwm[1]);
    thruster7.writeMicroseconds(pwm[2]);
    thruster8.writeMicroseconds(pwm[3]);
}

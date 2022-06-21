//ROS lib
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <Servo.h>
#include "config.h"

volatile byte state = HIGH;
byte state_count = 0;
double oz,ow;
std_msgs::Float64 test_msg;

Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;

ros::NodeHandle nh;

void imuCallback(const sensor_msgs::Imu& msg)
{
  oz = msg.orientation.z;
  ow = msg.orientation.w;
  nh.loginfo("Program info");
}
ros::Subscriber<sensor_msgs::Imu> imu_sub("/zedm/zed_node/imu/data", &imuCallback);

ros::Publisher test_pub("test_data", &test_msg);

void stop_operation(){
    thruster1.writeMicroseconds(1500);
    thruster2.writeMicroseconds(1500);
    thruster3.writeMicroseconds(1500);
    thruster4.writeMicroseconds(1500);
    thruster5.writeMicroseconds(1500);
    thruster6.writeMicroseconds(1500);
    thruster7.writeMicroseconds(1500);
    thruster8.writeMicroseconds(1500);
}

void all_run_slow(){
    thruster1.writeMicroseconds(1550);
    thruster2.writeMicroseconds(1550);
    thruster3.writeMicroseconds(1550);
    thruster4.writeMicroseconds(1550);
    thruster5.writeMicroseconds(1550);
    thruster6.writeMicroseconds(1550);
    thruster7.writeMicroseconds(1550);
    thruster8.writeMicroseconds(1550);
}

void forward(){
    thruster1.writeMicroseconds(1500);
    thruster2.writeMicroseconds(1500);
    thruster3.writeMicroseconds(1600);
    thruster4.writeMicroseconds(1600);
}

void down(){
    thruster5.writeMicroseconds(1400);
    thruster6.writeMicroseconds(1600);
    thruster7.writeMicroseconds(1600);
    thruster8.writeMicroseconds(1400);
}

void up(){
    thruster5.writeMicroseconds(1650);
    thruster6.writeMicroseconds(1350);
    thruster7.writeMicroseconds(1350);
    thruster8.writeMicroseconds(1650);
}

void transit_state () {
  if(state_count >= 100){
    state = !state;
    state_count = 0;
  }
  else{
    state_count++;
  }
}

void setup() {
  cli();
//  while (!ping.initialize()) {
//    Serial.println("\nPing device failed to initialize!");
//    Serial.println("Are the Ping rx/tx wired correctly?");
//    Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
//    Serial.print(arduinoTxPin);
//    Serial.println(" (Arduino tx)");
//    Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
//    Serial.print(arduinoRxPin);
//    Serial.println(" (Arduino rx)");
//    delay(2000);
//  }
  Serial.begin(57600);
  
  thruster1.attach(MOTOR_NO_1_PIN);
  thruster2.attach(MOTOR_NO_2_PIN);
  thruster3.attach(MOTOR_NO_3_PIN);
  thruster4.attach(MOTOR_NO_4_PIN);
  thruster5.attach(MOTOR_NO_5_PIN);
  thruster6.attach(MOTOR_NO_6_PIN);
  thruster7.attach(MOTOR_NO_7_PIN);
  thruster8.attach(MOTOR_NO_8_PIN);

  Serial.println("Ready...");
  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);
  thruster3.writeMicroseconds(1500);
  thruster4.writeMicroseconds(1500);
  thruster5.writeMicroseconds(1500);
  thruster6.writeMicroseconds(1500);
  thruster7.writeMicroseconds(1500);
  thruster8.writeMicroseconds(1500);
  delay(1000*5);

  pinMode(E_STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), transit_state, HIGH);

  nh.initNode();
//  nh.subscribe(cmd_sub);
//  nh.subscribe(angvel_sub);
  nh.subscribe(imu_sub);
  nh.advertise(test_pub);
  
  sei();
}

int get_depth(){
  if (ping.update()) {
    return ping.distance(); //in mm
  } 
}

void loop() {

  nh.spinOnce();
  
  test_msg.data = ow;
  nh.loginfo("start");

  test_pub.publish(&test_msg);
  
  delay(100);

//  if (state == HIGH) {
//    stop_operation();
//    Serial.println("STOP!");
//  }
//  
//  else if (state == LOW){
//    Serial.println("GO!");
//    forward();
//    up();
//  }
  
}

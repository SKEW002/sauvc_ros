//ROS lib
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#include "thruster_control.h"
#include "ping_sonar.h"

volatile byte state = HIGH;
byte state_count = 0;
float gx;
std_msgs::Int16 depth_msg;

ros::NodeHandle nh;

void controlCallback(const std_msgs::Float32& msg)
{
  gx = msg.data;
  nh.loginfo("Program info");
}
ros::Subscriber<std_msgs::Float32> control_sub("/cmd_out/gx", &controlCallback);

ros::Publisher depth_pub("depth", &depth_msg);


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

  Serial.begin(57600);
  Serial.println("Ready...");
  
  register_motor();
  stop_operation();
  delay(1000*5);

  pinMode(E_STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), transit_state, HIGH);

  nh.initNode();
  nh.subscribe(control_sub);
  nh.advertise(depth_pub);
}



void loop() {

  nh.spinOnce();
  
  nh.loginfo("start");

  depth_pub.publish(&depth_msg);
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

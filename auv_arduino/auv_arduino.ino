//ROS lib
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "thruster_control.h"
#include "ping_sonar.h"

volatile byte state = HIGH;
byte state_count = 0;
float gx;
int hori_pwm[4];
int vert_pwm[4];

std_msgs::Int16 depth_msg;

ros::NodeHandle nh;

void testCallback(const std_msgs::Float32& msg)
{
  gx = msg.data;
  nh.loginfo("Program info");
}
ros::Subscriber<std_msgs::Float32> test_sub("/cmd_out/gx", &testCallback);


void controlCallback(const std_msgs::Int16MultiArray& msg)
{
  for(int i=0; i<4; i++){
    hori_pwm[i] = msg.data[i];
  }
  for(int i=0; i<4; i++){
    vert_pwm[i] = msg.data[i+4];
  }

  if(hori_pwm[0] == 1){
    nh.loginfo("Success");
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> control_sub("/cmd_out/pwm", &controlCallback);



ros::Publisher depth_pub("/cmd_out/depth", &depth_msg);


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
  //initialize_ping_sonar();
  delay(1000*5);

  pinMode(E_STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), transit_state, HIGH);

  nh.initNode();
  nh.subscribe(test_sub);
  nh.subscribe(control_sub);
  nh.advertise(depth_pub);
}



void loop() {

  nh.spinOnce();
    
  horizontal_movement(hori_pwm);
  vertical_movement(vert_pwm);
  depth_msg.data = get_depth();
  depth_pub.publish(&depth_msg);
  //nh.loginfo("Start");
  delay(100);

  
}

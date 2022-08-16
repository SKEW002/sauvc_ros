//ROS lib
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "thruster_control.h"
#include "ping_sonar.h"

volatile byte state = LOW;
byte state_count = 0;
float gx;
int hori_pwm[4];
int vert_pwm[4];
int stop_pwm[4] = {1500,1500,1500,1500};

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


bool transit_state () {
  state = !state;
  return true;

}



void setup() {
  
  Serial.begin(57600);
  Serial.println("Ready...");
  
  register_motor();
  stop_operation();
  //initialize_ping_sonar();
  delay(1000*5);

  pinMode(E_STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), transit_state, FALLING);

  nh.initNode();
  nh.subscribe(test_sub);
  nh.subscribe(control_sub);
  nh.advertise(depth_pub);
}



void loop() {
  nh.spinOnce();
  Serial.println(digitalRead(2));

  depth_msg.data = get_depth();
  depth_pub.publish(&depth_msg);
  Serial.println(get_depth());
  
  char log_msg[100];
  
  sprintf(log_msg, "Hori: %d %d %d %d", (int)(hori_pwm[0]), (int)(hori_pwm[1]),(int)(hori_pwm[2]), (int)(hori_pwm[3]));
  nh.loginfo(log_msg);
  //Serial.println(state);
  if(state == LOW){
    
    horizontal_movement(hori_pwm);
    
    vertical_movement(stop_pwm);
    nh.loginfo("run");
    Serial.println("run");
  }
  else if(state == HIGH){
    nh.loginfo("stop");
    Serial.println("stop");
    horizontal_movement(stop_pwm);
    vertical_movement(stop_pwm);
  }
  delay(100);

  
}

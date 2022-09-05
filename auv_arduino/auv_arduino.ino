//ROS lib
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>

#include "thruster_control.h"
#include "ping_sonar.h"
#include "MS5837.h"

MS5837 depth_sensor;
volatile byte state = HIGH;
uint16_t state_count = 0;
int hori_pwm[4];
int vert_pwm[4];
int stop_pwm[4] = {STOP_SIGNAL,STOP_SIGNAL,STOP_SIGNAL,STOP_SIGNAL};

std_msgs::UInt16 depth_msg;

ros::NodeHandle nh;



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
  if (state_count >5){
    state = !state;    
    state_count = 0;
  }
   return true;

}



void setup() {
  
  Serial.begin(57600);
  Serial.println("Ready...");
  
  register_motor();
  stop_operation();
  //initialize_ping_sonar();
  delay(1000*5);
  Wire.begin();

  while (!depth_sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar02: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  depth_sensor.setModel(DEPTH_SENSOR_MODEL);
  depth_sensor.setFluidDensity(FLUID_DENSITY); 

  pinMode(E_STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), transit_state, FALLING);

  nh.initNode();
  nh.subscribe(control_sub);
  nh.advertise(depth_pub);
}



void loop() {
  nh.spinOnce();
  Serial.println(digitalRead(E_STOP_PIN));
  
  depth_sensor.read();
  depth_msg.data = abs((int)(depth_sensor.depth() * 100)); //cm
  depth_pub.publish(&depth_msg);

  // For pwm debugging
  /*
  char log_msg[100];
  sprintf(log_msg, "Hori: %d %d %d %d", (int)(hori_pwm[0]), (int)(hori_pwm[1]),(int)(hori_pwm[2]), (int)(hori_pwm[3]));
  nh.loginfo(log_msg);
  Serial.println(state);
  */
  
  if(state == LOW){
    //test_motor();

    //horizontal_movement(hori_pwm);
    vertical_movement(vert_pwm);
    nh.loginfo("run");
    Serial.println("run");
  }
  else if(state == HIGH){
    nh.loginfo("stop");
    Serial.println("stop");
    horizontal_movement(stop_pwm);
    vertical_movement(stop_pwm);
  }
  
  if(state_count < 255){ //prevent bounce back
    state_count++;
    Serial.println(state_count);
  }
  delay(100);

  
}

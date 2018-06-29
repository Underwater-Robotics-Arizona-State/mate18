
#include <Wire.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#define FULL_STOP          1500
#define FRONT_RIGHT_PIN    7
#define FRONT_LEFT_PIN     3
#define BACK_RIGHT_PIN     5 
#define BACK_LEFT_PIN      2
#define FRONT_VERT_PIN     8
#define BACK_VERT_PIN      4
#define CLAW_GRAB_PIN      6
#define CLAW_BOTTOM_PIN    10
#define LIGHT_PIN          11

Servo frontR;
Servo frontL;
Servo backR;
Servo backL;
Servo frontV;
Servo backV;
Servo clawFront;
Servo clawBottom;
Servo light;


//init ros node need to run ros
ros::NodeHandle  nh;

std_msgs::Int16MultiArray motors_back;
ros::Publisher pub_back("motor_back", &motors_back);


int16_t back[9];

/* 

0   +   1
/-------\
|   4   |
|       |
|   5   |
\-------/
2   +   3

data value array locations for the motors


*/


//light true = 1900, ligth false = 1100;

void motor_value_pull( const std_msgs::Int16MultiArray& values){
  
  frontL.writeMicroseconds(values.data[0]);
  frontR.writeMicroseconds(values.data[1]);

  backL.writeMicroseconds(values.data[2]);
  backR.writeMicroseconds(values.data[3]);

  frontV.writeMicroseconds(values.data[4]);
  backV.writeMicroseconds(values.data[5]);

  clawFront.writeMicroseconds(values.data[6]);
  clawBottom.writeMicroseconds(values.data[8]);
  
  for(int i = 0; i < 9; i++) back[i] = values.data[i];
  
   
}

void light_pull(const std_msgs::Bool& value){

  if(value.data) light.writeMicroseconds(1900);
  else light.writeMicroseconds(1100);
  
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_motor("motorValues", motor_value_pull);
ros::Subscriber<std_msgs::Bool> sub_light("light", light_pull);

void setup(){

  nh.initNode();
  
  nh.subscribe(sub_motor);
  nh.subscribe(sub_light);
  
  motors_back.data_length = 9;
  
  nh.advertise(pub_back);
  
  Serial.begin(57600);

  frontR.attach(FRONT_RIGHT_PIN);
  frontL.attach(FRONT_LEFT_PIN);

  backR.attach(BACK_RIGHT_PIN);
  backL.attach(BACK_LEFT_PIN);

  frontV.attach(FRONT_RIGHT_PIN);
  backV.attach(BACK_LEFT_PIN);

  clawFront.attach(CLAW_GRAB_PIN);
  clawBottom.attach(CLAW_BOTTOM_PIN);
  light.attach(LIGHT_PIN);


  
  frontR.writeMicroseconds(FULL_STOP);
  frontL.writeMicroseconds(FULL_STOP);

  backR.writeMicroseconds(FULL_STOP);
  backL.writeMicroseconds(FULL_STOP);

  frontV.writeMicroseconds(FULL_STOP);
  backV.writeMicroseconds(FULL_STOP);

  clawFront.writeMicroseconds(FULL_STOP);
  clawBottom.writeMicroseconds(FULL_STOP);


  delay(3000);
  
}

void loop(){
 
  motors_back.data = back;
  pub_back.publish(&motors_back);

  nh.spinOnce();

  delay(10);
}

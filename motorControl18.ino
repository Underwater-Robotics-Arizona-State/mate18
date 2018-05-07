
#include <Wire.h>
#include "HCPCA9685.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#include <printf.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#define I2CAdd             0x40
#define FULL_STOP          210
#define FRONT_RIGHT_PIN    11
#define FRONT_LEFT_PIN     10
#define BACK_RIGHT_PIN     13
#define BACK_LEFT_PIN      1
#define FRONT_VERT_PIN     2
#define BACK_VERT_PIN      3
#define CLAW_GRAB_PIN      0
#define CLAW_ROTATION_PIN  100    //dont worry about that
#define CLAW_BOTTOM_PIN    7
#define LIGHT_PIN          15

const byte rxAddr[6] = "00001";

HCPCA9685 HCPCA9685(I2CAdd);
RF24 radio(7, 8);


//init ros node need to run ros
ros::NodeHandle  nh;

std_msgs::Int16MultiArray motors_back;
ros::Publisher pub_back("motor_back", &motors_back);


int16_t back[9];

/* IGNORE ATM

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
  

  HCPCA9685.Servo(FRONT_RIGHT_PIN, values.data[0]);
  HCPCA9685.Servo(FRONT_LEFT_PIN, values.data[1]);

  HCPCA9685.Servo(BACK_RIGHT_PIN, values.data[2]);
  HCPCA9685.Servo(BACK_LEFT_PIN, values.data[3]);

  HCPCA9685.Servo(FRONT_VERT_PIN, values.data[4]);
  HCPCA9685.Servo(BACK_VERT_PIN, values.data[5]);

  HCPCA9685.Servo(CLAW_GRAB_PIN, values.data[6]);
  HCPCA9685.Servo(CLAW_ROTATION_PIN, values.data[7]);
  HCPCA9685.Servo(CLAW_BOTTOM_PIN, values.data[8]);

  for(int i = 0; i < 9; i++) back[i] = values.data[i];
  
   
}

void light_pull(const std_msgs::Bool& value){

  if(value.data) HCPCA9685.Servo(LIGHT_PIN, 369);
  else HCPCA9685.Servo(LIGHT_PIN, 82);
  
}

void wifi_send(boolean flag){

      if(flag){
      const char text[] = "Hello World";
      radio.write(&text, sizeof(text));
    }
}

void wifi_pull(const std_msgs::Bool& value){

  wifi_send(value.data);
 
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_motor("motorValues", motor_value_pull);
ros::Subscriber<std_msgs::Bool> sub_light("light", light_pull);
ros::Subscriber<std_msgs::Bool> sub_wifi("wifi", wifi_pull);

void setup(){

  nh.initNode();
  
  nh.subscribe(sub_motor);
  nh.subscribe(sub_light);
  
  motors_back.data_length = 9;
  
  nh.advertise(pub_back);

  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  Wire.begin();
  
  Serial.begin(57600);

  HCPCA9685.Servo(FRONT_RIGHT_PIN, FULL_STOP);
  HCPCA9685.Servo(FRONT_LEFT_PIN, FULL_STOP);

  HCPCA9685.Servo(BACK_RIGHT_PIN, FULL_STOP);
  HCPCA9685.Servo(BACK_LEFT_PIN, FULL_STOP);


  HCPCA9685.Servo(FRONT_VERT_PIN, FULL_STOP);
  HCPCA9685.Servo(BACK_VERT_PIN, FULL_STOP);

  HCPCA9685.Servo(CLAW_GRAB_PIN, FULL_STOP);
  HCPCA9685.Servo(CLAW_ROTATION_PIN, FULL_STOP);
  
  HCPCA9685.Servo(CLAW_BOTTOM_PIN, FULL_STOP);

  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  

  delay(3000);
  
}

void loop(){
 
  motors_back.data = back;
  pub_back.publish(&motors_back);

  
  
  nh.spinOnce();

  delay(10);
}

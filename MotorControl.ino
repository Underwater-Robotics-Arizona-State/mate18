
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#define FULL_STOP          1500
#define FRONT_RIGHT_PIN    0
#define FRONT_LEFT_PIN     1
#define BACK_RIGHT_PIN     2
#define BACK_LEFT_PIN      3
#define FRONT_VERT_PIN     4
#define BACK_VERT_PIN      5
#define CLAW_GRAB_PIN      6
#define CLAW_ROTATION_PIN  7
#define CLAW_BOTTOM_PIN    8
#define LIGHT_PIN          9


//creating i2c PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
  

  pwm.setPWM(FRONT_RIGHT_PIN, 0, values.data[0]);
  pwm.setPWM(FRONT_LEFT_PIN, 0, values.data[1]);

  pwm.setPWM(FRONT_RIGHT_PIN, 0, values.data[2]);
  pwm.setPWM(BACK_LEFT_PIN, 0, values.data[3]);

  pwm.setPWM(FRONT_VERT_PIN, 0, values.data[4]);
  pwm.setPWM(BACK_VERT_PIN, 0, values.data[5]);

  pwm.setPWM(CLAW_GRAB_PIN, 0, values.data[6]);
  pwm.setPWM(CLAW_ROTATION_PIN, 0, values.data[7]);
  pwm.setPWM(CLAW_BOTTOM_PIN, 0, values.data[8]);

  back[0] = values.data[0];
  back[1] = values.data[1];
  back[2] = values.data[2];
  back[3] = values.data[3];
  back[4] = values.data[4];
  back[5] = values.data[5];
  back[6] = values.data[6];
  back[7] = values.data[7];
  back[8] = values.data[8];
  
   
}

void light_pull(const std_msgs::Bool& value){

  if(value.data) pwm.setPWM(LIGHT_PIN, 0, 1900);
  else pwm.setPWM(LIGHT_PIN, 0, 1100);
  
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_motor("motorValues", motor_value_pull);
ros::Subscriber<std_msgs::Bool> sub_light("light", light_pull);

void setup(){

  nh.initNode();
  
  nh.subscribe(sub_motor);
  nh.subscribe(sub_light);
  
  motors_back.data_length = 9;
  
  nh.advertise(pub_back);
 
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(57600);
  
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

  pwm.setPWM(FRONT_RIGHT_PIN, 0, FULL_STOP);
  pwm.setPWM(FRONT_LEFT_PIN, 0, FULL_STOP);

  pwm.setPWM(FRONT_RIGHT_PIN, 0, FULL_STOP);
  pwm.setPWM(BACK_LEFT_PIN, 0, FULL_STOP);


  pwm.setPWM(FRONT_VERT_PIN, 0, FULL_STOP);
  pwm.setPWM(BACK_VERT_PIN, 0, FULL_STOP);

  pwm.setPWM(CLAW_GRAB_PIN, 0, FULL_STOP);
  pwm.setPWM(CLAW_ROTATION_PIN, 0, FULL_STOP);
  
  pwm.setPWM(CLAW_BOTTOM_PIN, 0, FULL_STOP);


  delay(2000);
  
}

void loop(){
  

  motors_back.data = back;
  pub_back.publish(&motors_back);
  
  nh.spinOnce();

  delay(10);
}

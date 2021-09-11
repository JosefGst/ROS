/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (right_ticks and left_ticks topics) using the built-in encoder 
 * (forward = positive; reverse = negative) 
 */
 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

#define EN_L 9
#define IN1_L 10
#define IN2_L 11

#define EN_R 6    //should be PWM PIN!
#define IN1_R 12
#define IN2_R 13


double w_r=0, w_l=0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.064, wheel_sep = 0.145;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 5
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;


double speed_ang=0, speed_lin=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = true; // Reverse
  }
  else {
    Direction_right = false; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
void setup() {
  Motors_init();
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}
 
void loop() {
  MotorL(w_l*10);
  MotorR(w_r*10);
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
  }
}

void Motors_init(){
 pinMode(EN_L, OUTPUT);
 pinMode(EN_R, OUTPUT);
 pinMode(IN1_L, OUTPUT);
 pinMode(IN2_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(EN_L, LOW);
 digitalWrite(EN_R, LOW);
 digitalWrite(IN1_L, LOW);
 digitalWrite(IN2_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(EN_L, constrain(Pulse_Width1,100,255));
     digitalWrite(IN1_L, HIGH);
     digitalWrite(IN2_L, LOW);
 }

 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(constrain(Pulse_Width1,-255,-100));
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);
 }

 if (Pulse_Width1 == 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, LOW);
 }
}


void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(EN_R, constrain(Pulse_Width2,100,255));
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);
 }

 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(constrain(Pulse_Width2,-255,-100));
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);
 }

 if (Pulse_Width2 == 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, LOW);
 }
}

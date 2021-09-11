//Code Authors:
//* Ahmed A. Radwan (author)
//* Maisa Jazba

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>



#define EN_L 9
#define IN1_L 10
#define IN2_L 11

#define EN_R 6    //should be PWM PIN!
#define IN1_R 12
#define IN2_R 13

unsigned long previousMillis1 = 0;  
const long period1 = 500;

double w_r = 0, w_l = 0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.064, wheel_sep = 0.145;

//Range
#define pingPin 8
const boolean CENTIMETERS = true;
const boolean INCHES = false;

ros::NodeHandle nh;

double speed_ang = 0, speed_lin = 0;

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound_range", &range_msg);
char frameid[] = "/ultrasound";

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void Motors_init();
void MotorL(int16_t Pulse_Width1);
void MotorR(int16_t Pulse_Width2);

void setup() {
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.002;  // 2 cm
  range_msg.max_range = 0.150;  // 150 cm


  //Serial.begin(9600);
}


void loop() {
  MotorL(w_l * 10);
  MotorR(w_r * 10);

  //Range Sensor
//  unsigned long currentMillis = millis();
//  if (currentMillis - previousMillis1 >= period1) {
//    previousMillis1 = currentMillis;
// 
    range_msg.range=getRange(pingPin, CENTIMETERS);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    nh.spinOnce();
//  }
  delay(50);
}

void Motors_init() {
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

void MotorL(int Pulse_Width1) {
  if (Pulse_Width1 > 0) {
analogWrite(EN_L, constrain(Pulse_Width1,60,255));
digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }

  if (Pulse_Width1 < 0) {
     Pulse_Width1=abs(constrain(Pulse_Width1,-255,-70));
     analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }

  if (Pulse_Width1 == 0) {
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
}


void MotorR(int Pulse_Width2) {

  if (Pulse_Width2 > 0) {
    analogWrite(EN_R, constrain(Pulse_Width2,70,255));
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }

  if (Pulse_Width2 < 0) {
    Pulse_Width2 = abs(constrain(Pulse_Width2,-255,-60));
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }

  if (Pulse_Width2 == 0) {
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
}

/*
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}
*/

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

long getRange(uint8_t pinNumber, boolean in_centimeters){

    // establish variables for duration of the ping, and the distance result
      // in inches and centimeters:
      long duration, distance, inches, cm;
    
      // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
      // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);
    
      // The same pin is used to read the signal from the PING))): a HIGH pulse
      // whose duration is the time (in microseconds) from the sending of the ping
      // to the reception of its echo off of an object.
      pinMode(pingPin, INPUT);
      duration = pulseIn(pingPin, HIGH);
    
      // convert the time into a distance
      //inches = microsecondsToInches(duration);
      cm = microsecondsToCentimeters(duration);

      if (in_centimeters) 
         distance = cm;
      else 
        distance = inches;
      //Serial.print(inches);
      //Serial.print("in, ");
      Serial.print(cm);
      Serial.println("cm");

      return distance;
      
}

#define EN_L 9
#define IN1_L 10
#define IN2_L 11

#define EN_R 6
#define IN1_R 12
#define IN2_R 13


void setup() {
  analogWrite(EN_L, 120);

  digitalWrite(IN1_L, HIGH);

  digitalWrite(IN2_L, LOW);

  analogWrite(EN_R, 120);

  digitalWrite(IN1_R, HIGH);

  digitalWrite(IN2_R, LOW);
  
  


}

void loop() {
  // put your main code here, to run repeatedly:

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

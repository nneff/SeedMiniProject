
//red sda
//blue scl
//grey GND


#define Enable 4
#define MotorDir 7
#define MotorSpeed 9
#define FlagIndicator 12
#define pinA 2 //yellow
#define pinB 3 //white
#include <math.h>;
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
long stateB, stateA, prevB;
int count = 0;
double Angle, diff, SpeedV, PWM;
double desiredAngle = 0;
double kp = 4; 
double ki = 0.015;
double ErrorSum = 0;
int fromrpi;
void setup() {
  Serial.begin(9600);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, HIGH);
  pinMode(MotorDir, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(FlagIndicator, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), readEncoder, CHANGE);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}

void loop() {
  // put your main code here, to run repeatedly:
    Angle =  (count / 3200.0)*(360.0)*(PI/180.0);
    Serial.println(Angle);
    diff = Angle - desiredAngle;
    ErrorSum = ErrorSum + diff;
    if (ErrorSum > 50) {
          ErrorSum = 50;
    }
    if (ErrorSum < -50) {
          ErrorSum = -50;
    }
    if (diff > 1) {
          diff = 1;
    }
    if (diff < -1) {
          diff = -1;
    }
    //Serial.println(ErrorSum);
    SpeedV = diff * kp + ErrorSum * ki;
        if (SpeedV > 0){
      digitalWrite(MotorDir, LOW);
    }
    else{
      digitalWrite(MotorDir, HIGH);
    }
    //Serial.println(SpeedV);
    SpeedV = abs(SpeedV);
    PWM = (SpeedV * 255.0)/7.2;
    //Serial.println(PWM);
    analogWrite(MotorSpeed, PWM);




      if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    desiredAngle = 0;
    }
}


void receiveData() {
  while(Wire.available()) {
    fromrpi = Wire.read();
    Serial.println(fromrpi);
    if(fromrpi == 1){
      desiredAngle = 0;
    }
    if(fromrpi == 2){
      desiredAngle = 1.57;
    }
    if(fromrpi == 3){
      desiredAngle = 3.14;
    }
    if(fromrpi == 4){
      desiredAngle = 4.71;
    }
  }
}

void readEncoder(){
  stateB = digitalRead(pinB);
  stateA = digitalRead(pinA);

  if (prevB != stateB){
    if (stateA != stateB){
      count = count -2;
    } else{
      count = count + 2;
    }
  }
  prevB = stateB;
}

#define Enable 4
#define MotorDir 7
#define MotorSpeed 9
#define FlagIndicator 12
#define pinA 2 //yellow
#define pinB 3 //white
#include <math.h>

long stateB, stateA, prevB;
int count = 0;
double Angle;
double desiredAngle = 0;
int period = 5;
unsigned long time_now = 0;
float Velocity;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, HIGH);
  pinMode(MotorDir, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(FlagIndicator, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), readEncoder, CHANGE);
  analogWrite(MotorSpeed, 0);

}

void loop() {
  time_now = millis();
  Angle =  (count / 3200.0)*(360.0)*(PI/180.0);
  Velocity = (Angle) / ((double(time_now)/1000)-1);
  
if (time_now >= 1000 and time_now <= 3000) {
  analogWrite(MotorSpeed, 35.417);
  Serial.print("Angular Position (rad): ");
  Serial.print(abs(Angle));
  Serial.print(" ");
  Serial.print(" || Angular Velocity (rad/s): ");
  Serial.print(abs(Velocity));
  Serial.println();

}
if (time_now > 3000) {
  analogWrite(MotorSpeed, 0); 
}
  while (millis() < time_now + period) {
  }
}


void readEncoder(){
  stateB = digitalRead(pinB);
  stateA = digitalRead(pinA);

  if (prevB != stateB){
    if (stateA != stateB){
      count = count +2;
    } else{
      count = count - 2;
    }
  }
  prevB = stateB;
}

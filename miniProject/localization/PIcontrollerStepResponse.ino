#define Enable 4
#define MotorDir 7
#define MotorSpeed 9
#define FlagIndicator 12
#define pinA 2 //yellow
#define pinB 3 //white
#include <math.h>

long stateB, stateA, prevB;
int count = 0;
double Angle, error, lastError, PWM, output;
double desiredAngle = 1; //angle we want the wheel to turn to in rads
double kp = 1.98; //proportional constant
double ki = .01;  //integral constant
double cumError = 0;  //cumulative error

int period = 5; //sampling time in ms
unsigned long time_now; //current time in ms

void setup() {
  Serial.begin(115200);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, HIGH);
  pinMode(MotorDir, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(FlagIndicator, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), readEncoder, CHANGE);
}

void loop() {
    time_now = millis();
    Angle = (count / 3200.0)*(360.0)*(PI/180.0);  //current wheel position in rads
    error = Angle - desiredAngle; //difference between current position and desired position
    cumError = cumError + error;
    if (cumError > 50) {
          cumError = 50;
    }
    if (cumError < -50) {
          cumError = -50;
    }
    output = kp*error + ki*cumError;
    if (output > 0) {
      digitalWrite(MotorDir, LOW);
    }
    if (output <= 0) {
      digitalWrite(MotorDir, HIGH);
    }
    
    output = abs(output);
    PWM = (output * 255.0)/7.2;
    analogWrite(MotorSpeed, PWM);
    Serial.print("Time: ")
    Serial.print(time_now);
    Serial.print(" || Current Position: ")
    Serial.print(Angle);
    Serial.println();
    while (millis() < time_now + period) {     
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

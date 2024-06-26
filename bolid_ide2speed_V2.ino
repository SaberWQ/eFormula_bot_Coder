#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*

*/

#define button 12
#define led_on 13

#define AQ1 5
#define AQ2 4
#define BQ1 7
#define BQ2 8
#define STBY 6

#define LM 9
#define RM 3

int IsStarted;

float Kp = 1.7;    //related to the proportional control term; 1.6
                    //change the value by trial-and-error (ex: 0.7).
float Ki = 0;  //related to the integral control term;
                    //change the value by trial-and-error (ex: 0.0008).
float Kd = 2.8;     //related to the derivative control term; 2.69
                    //change the value by trial-and-error (ex: 0.8).
int P = 0;
int I = 0;
int D = 0;
int lastError = 0;

int basespeed = 120;//100//120

unsigned long p1Time = 0;
unsigned long p2Time = 0;

void calibration() {
  delay(1000);
  for (uint16_t i = 0; i < 200; i++)  // 5 second
  {
    qtr.calibrate();
  }
}


void driver_control(int value) {
   digitalWrite(AQ2, value);
  digitalWrite(BQ2, value);
}

int Start() {

  int button1 = digitalRead(button);
  delay(40);
  int button2 = digitalRead(button);

  button1 = button1 | button2;

  if (button1 == 0)
    return 1;
  else
    return 0;
}

float PID_calculation(uint16_t pos) {

  int error = 3500 - pos;  //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  float delta = (P * Kp + D * Kd)*0.365;

  return delta;
}

void MotorControl(float delta, int base) {
  int motorspeeda = base + int(delta);
  int motorspeedb = base - int(delta);

  if(delta<150&&delta>-150){
    motorspeeda+=30;
    motorspeedb+=30;

  }

  if (motorspeeda > 255){
    motorspeedb = base - int(delta*2);
    motorspeeda = 255;
  } 
  if (motorspeedb > 255){
    motorspeeda = base + int(delta*2);
    motorspeedb = 255;
  } 
  if (motorspeeda < 0) motorspeeda = 0;
  if (motorspeedb < 0) motorspeedb = 0;

  analogWrite(LM, motorspeeda);
  analogWrite(RM, motorspeedb);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button, INPUT_PULLUP);

  pinMode(AQ1, OUTPUT);
  pinMode(AQ2, OUTPUT);
  pinMode(BQ1, OUTPUT);
  pinMode(BQ2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(AQ1, HIGH);
  digitalWrite(BQ1, HIGH);
  digitalWrite(STBY, HIGH);
  digitalWrite(AQ2, LOW);
  digitalWrite(BQ2, LOW);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);

  delay(500);
  pinMode(led_on, OUTPUT);
  digitalWrite(led_on, HIGH);

  calibration();
  delay(500);
  // IsStarted = false;
}

void loop() {

  digitalWrite(led_on, HIGH);
  uint16_t position = qtr.readLineBlack(sensorValues);
  digitalWrite(led_on, LOW);

  if (position<=400||position>=6600)
  {
    driver_control(HIGH);
  }
  else{
    driver_control(LOW);
    
  }

  if (IsStarted == 0) {
    p1Time = millis();
    IsStarted = Start();
  }

  if (IsStarted != 0) {
    float deltaspeed = PID_calculation(position);
    MotorControl(deltaspeed, basespeed);

    p2Time = millis() - p1Time;

    if (p2Time > 200000) IsStarted = 0;

  } else {
    analogWrite(LM, 0);
    analogWrite(RM, 0);
    P = 0;
    I = 0;
    D = 0;
    lastError = 0;    
  }
   //delay(100);

  // Serial.println(position);
}

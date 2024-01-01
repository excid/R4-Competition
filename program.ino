#include <Servo.h>

#define PWM_A D3
#define DIR_A D12
#define BRAKE_A D9
#define PWM_B D11
#define DIR_B D13
#define BRAKE_B D8

#define TRIGGER_F D1
#define ECHO_F D0

#define sen_1 2
#define sen_2 A5
#define sen_3 A4
#define sen_4 A2
#define sen_5 A3

Servo myservo;

int FS1, FS2, FS3, FS4, FS5;

int BB = 0, BW = 0, WB = 0, turningCount = 0, forwardCount = 0, servoCount = 0, backCount = 0;

double Kp = 65, Ki = 0, Kd = 0;

double PID_value;

double lineDetectStartTimer = 0, lineDetectEndTimer = 0;

long duration, distance;

bool isLifting = true;

void setup()
{
  Serial.begin(115200);

  myservo.attach(5);

  const byte pinOut[] = {DIR_A, BRAKE_A, PWM_A, DIR_B, BRAKE_B, PWM_B, TRIGGER_F};
  const byte pinIn[] = {sen_1, sen_2, sen_3, sen_4, sen_5, ECHO_F};

  for (byte i = 0; i < sizeof(pinOut); i++) {
    pinMode(pinOut[i], OUTPUT);
  }

  for (byte i = 0; i < sizeof(pinIn); i++) {
    pinMode(pinIn[i], INPUT);
  }

  myservo.write(70);
  delay(1000);
}

void loop()
{
  FS1 = digitalRead(sen_1);
  FS2 = digitalRead(sen_2);
  FS3 = digitalRead(sen_3);
  FS4 = digitalRead(sen_4);
  FS5 = digitalRead(sen_5);

  /*digitalWrite(TRIGGER_F, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIGGER_F, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIGGER_F, LOW);
    duration = pulseIn(ECHO_F, HIGH);
    distance = (duration/2) / 29.1;
    Serial.print(duration);
    Serial.print(" cm\n");*/

  incrementCheck(0.8);

  switch (BB) {

    // BB = 0 TO 4
    case 0 ... 4:
      PID(150);
      break;

    // BB = 5
    case 5:
      backRight(150, 1, 0);
      PID(150);
      break;

    case 6:
      forward(150, 0.3, 0);
      backRight(150, 1, 1);
      back(150,0.1,0);
      servoLift(0, 0); // Lift
      forward(150, 0.5, 1);
      PID(150);
      break;

    case 7:
      backRight(150, 1, 2);
      PID(150);

      break;

    case 8:
      backLeft(150, 1, 3);
      PID(150);
      break;

    case 9 ... 10:
      PID(150);
      break;

    case 11:
      backLeft(150, 1, 4);
      PID(150);
      break;

    case 12:
      backRight(150, 1, 5);
      PID(150);
      break;

    case 13:
      forward(150, 0.3, 2);
      robotStop();
      servoLift(70, 1); // Lift
      BB++;
      break;

    case 14:
      robotStop();
      break;

    /*case 14:
      turnAround(150, 1, 6);
      forward(150, 0.5, 3);
      PID(150);
      break;*/

    /*case 15:
      PID(150);
      break;

    case 16:
      backLeft(150, 1, 7);
      PID(150);
      break;

    case 17 ... 18:
      PID(150);
      break;

    case 19:
      backRight(150, 1, 8);
      PID(150);
      break;

    case 20 ... 22:
      PID(150);
      break;

    case 23:
      backLeft(150, 1, 9);
      PID(150);
      break;

    case 24:
      forward(150, 0.3, 4);
      backLeft(150, 1, 10);
      servoLift(0, 2); // Lift
      forward(150, 0.5, 5);
      PID(150);
      break;

    case 25:
      backLeft(150, 1, 11);
      PID(150);
      break;

    case 26:
      backRight(150, 1, 12);
      PID(150);
      break;

    case 27 ... 29:
      PID(150);
      break;

    case 30:
      backRight(150, 1, 12);
      PID(150);
      break;

    case 31:
      forward(150, 0.3, 2);
      robotStop();
      servoLift(70, 3); // Lift
      BB++;
      break;

    case 32:
      turnAround(150, 1, 6);
      forward(150, 0.5, 3);
      PID(150);
      break;

    case 33:
      robotStop();
      break;*/












    default:
      break;
  }
}

void incrementCheck(double detectCooldown) {
  lineDetectEndTimer = (double)millis() / 1000;
  if (!FS1 && !FS2 && !FS4 && !FS5 && (lineDetectEndTimer - lineDetectStartTimer >= detectCooldown)) {
    BB++;
    delay(200);
    robotStop();
    delay(200);
    Serial.print("BB"); Serial.println(BB);
    lineDetectStartTimer = lineDetectEndTimer;
  }

  /*if (BB == 5) {
    if (!FS1 && !FS2 && (lineDetectEndTimer - lineDetectStartTimer >= detectCooldown)) {
      BW++;
      delay(200);
      robotStop();
      delay(200);
      Serial.print("BW"); Serial.println(BW);
      lineDetectStartTimer = lineDetectEndTimer;
    }
    }
    /*if (!FS4 && !FS5 && !FS3 && FS1 && FS2 && (lineDetectEndTimer - lineDetectStartTimer >= detectCooldown)) {
    WB++;
    Serial.print("WB"); Serial.println(WB);
    lineDetectStartTimer = lineDetectEndTimer;
    }*/
}

void PID(int Speed) {
  int error, sum_error, last_error, d_error;
  FS1 = digitalRead(sen_1);
  FS2 = digitalRead(sen_2);
  FS3 = digitalRead(sen_3);
  FS4 = digitalRead(sen_4);
  FS5 = digitalRead(sen_5);
  if (FS1 == 0 && FS2 == 1 && FS3 == 1 && FS4 == 1 && FS5 == 1) // 0 1 1 1 1
  {
    error = -2 ;
  }
  else if (FS1 == 1 && FS2 == 0 && FS3 == 1 && FS4 == 1 && FS5 == 1) // 1 0 1 1 1
  {
    error = -1 ;
  }
  else if (FS1 == 1 && FS2 == 1 && FS3 == 0 && FS4 == 1 && FS5 == 1) // 1 1 0 1 1
  {
    error = 0 ;
  }
  else if (FS1 == 1 && FS2 == 1 && FS3 == 1 && FS4 == 0 && FS5 == 1) // 1 1 1 0 1
  {
    error = 1 ;
  }
  else if (FS1 == 1 && FS2 == 1 && FS3 == 1 && FS4 == 1 && FS5 == 0) // 1 1 1 1 0
  {
    error = 2 ;
  }
  else
  {
    error = 0 ;
  }
  PID_value = Kp * error + Ki * sum_error + Kd * d_error;
  sum_error = sum_error + error ;
  d_error = error - last_error;
  analogWrite(PWM_A, Speed + PID_value);
  analogWrite(PWM_B, Speed - PID_value);
  last_error = error;
}

void turnAround(int Speed, double timeInSecond, int Count) {
  while (turningCount == Count) {
    robotStop();
    delay(200);
    robotStop();
    digitalWrite(DIR_A, 1);
    digitalWrite(DIR_B, 0);
    delay(500);
    analogWrite(PWM_A, Speed);
    analogWrite(PWM_B, Speed);
    delay(timeInSecond * 1000);
    robotStop();
    delay(200);
    digitalWrite(DIR_A, 0);
    digitalWrite(DIR_B, 0);
    turningCount++;
  }
}


void back(int Speed, double timeInSecond, int Count) {
  while (backCount == Count) {
    robotStop();
    digitalWrite(DIR_A, 1);
    digitalWrite(DIR_B, 1);
    delay(200);
    analogWrite(PWM_A, Speed);
    analogWrite(PWM_B, Speed);
    delay(450);
    robotStop();
    digitalWrite(DIR_A, 0);
    digitalWrite(DIR_B, 0);
    robotStop();
    delay(200);
    backCount++;
  }
}

void backRight(int Speed, double timeInSecond, int Count) {
  while (turningCount == Count) {
    robotStop();
    digitalWrite(DIR_A, 1);
    digitalWrite(DIR_B, 1);
    delay(200);
    analogWrite(PWM_A, 150);
    analogWrite(PWM_B, 150);
    delay(450);
    robotStop();
    digitalWrite(DIR_A, 0);
    digitalWrite(DIR_B, 0);
    delay(500);
    analogWrite(PWM_A, Speed);
    analogWrite(PWM_B, 0);
    delay(timeInSecond * 1000);
    robotStop();
    delay(200);
    turningCount++;
  }
}


void right(int Speed, double timeInSecond, int Count) {
  while (turningCount == Count) {
    robotStop();
    delay(200);
    analogWrite(PWM_A, Speed);
    analogWrite(PWM_B, 0);
    delay(timeInSecond * 1000);
    robotStop();
    delay(200);
    turningCount++;
  }
}

void backLeft(int Speed, double timeInSecond, int Count) {
  while (turningCount == Count) {
    robotStop();
    digitalWrite(DIR_A, 1);
    digitalWrite(DIR_B, 1);
    delay(200);
    analogWrite(PWM_A, 150);
    analogWrite(PWM_B, 150);
    delay(450);
    robotStop();
    digitalWrite(DIR_A, 0);
    digitalWrite(DIR_B, 0);
    delay(500);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, Speed);
    delay(timeInSecond * 1000);
    robotStop();
    delay(200);
    turningCount++;
  }
}

void forward(int Speed, double timeInSecond, int Count) {
  while (forwardCount == Count) {
    analogWrite(PWM_A, Speed);
    analogWrite(PWM_B, Speed);
    delay(timeInSecond * 1000);
    robotStop();
    delay(200);
    forwardCount++;
  }
}

void robotStop() {
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void servoLift(int Degree, int Count) {
  if (servoCount == Count) {
    myservo.write(Degree);
    delay(1000);
    robotStop();
    delay(200);
    servoCount++; 
  }
}
#include <Arduino.h>
#include <MOTORS.h>
#include <ARRAY.h>
#include <PID.h>

// Global constants
#define BRAKE_DELAY 30
#define MAX_SPEED 100
#define MIN_SPEED 50
const int BASE_SPEED = (MIN_SPEED + ((MAX_SPEED - MIN_SPEED) / 2));
const unsigned int black = 0;
const unsigned int white = 1;
bool onLine = 0;
bool brakeFlag = 0;
int currentSpeed = 60;
bool brakeEnabled = 1;
int leftSpeed, rightSpeed, error, PIDvalue;
bool isBlackLine = 1;

// PID constants
float Kp = 0.01884;
float Kd = 0;
float Ki = 0;

// PID variables
float prevKP = 0;
float prevKI = 0;
float prevKD = 0;

MOTORS motors(6, 7, 8, 9, 10, 11);

ARRAY array(A0, A1, A2, A3, A4);

PID pid(Kp, Ki, Kd);

void printSensorValues(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Read sensor values
  motors.turnAround(90);
  array.calibrate();
  motors.stop();
  delay(1000);

  while (1)
  {
    if (Serial.available())
    {
      prevKD = Kd;
      prevKI = Ki;
      prevKP = Kp;

      char c = Serial.read();
      switch (c)
      {
      case 'A':
        Kp = pid.getKP() + 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;
      case 'B':
        Ki = pid.getKI() + 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;

      case 'C':
        Kd = pid.getKD() + 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;

      case 'D':
        Kp = pid.getKP() - 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;

      case 'E':
        Ki = pid.getKI() - 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;

      case 'F':
        Kd = pid.getKD() - 0.01;
        pid.setPID(Kp, Ki, Kd);
        break;

      default:
        break;
      }
    }

    if (array.IRvalues[0] > 700 && array.IRvalues[1] > 700 && array.IRvalues[2] > 700 && array.IRvalues[3] > 700 && array.IRvalues[4] > 700)
    {
      motors.turnAround(75);
    }

    array.readSensorValue();
    //
    onLine = array.isOnLine();
    if (onLine == 1)
    { //
      error = isBlackLine ? array.calcError() : (-1 * array.calcError());
      PIDvalue = pid.calcPID(error);

      leftSpeed = BASE_SPEED - PIDvalue;
      rightSpeed = BASE_SPEED + PIDvalue;
      leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

      Serial.print("KP: ");
      Serial.print(Kp);
      Serial.print(" e: ");
      Serial.print(error);
      Serial.print(" PIDval: ");
      Serial.print(PIDvalue);
      Serial.print(" LFS: ");
      Serial.print(leftSpeed);
      Serial.print(" RTS: ");
      Serial.println(rightSpeed);

      motors.moveA(leftSpeed);
      motors.moveB(rightSpeed);
      brakeFlag = 0;
    }
    else
    { // PID LINE SEARCH
      if (error > 0)
      {
        if (brakeEnabled == 1 && brakeFlag == 0)
        {
          motors.stop();
          delay(BRAKE_DELAY);
        }
        motors.turnRight(75, 75);
        brakeFlag = 1;
      }
      else
      {
        if (brakeEnabled == 1 && brakeFlag == 0)
        {
          motors.stop();
          delay(BRAKE_DELAY);
        }
        motors.turnLeft(75, 75);
        brakeFlag = 1;
      }
    }
  }
}

void printSensorValues(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
  Serial.println("------------------ Sensor Values ------------------");
  Serial.print("Sensor1: ");
  Serial.println(array.IRvalues[0]);
  Serial.print("Sensor2: ");
  Serial.println(array.IRvalues[1]);
  Serial.print("Sensor3: ");
  Serial.println(array.IRvalues[2]);
  Serial.print("Sensor4: ");
  Serial.println(array.IRvalues[3]);
  Serial.print("Sensor5: ");
  Serial.println(array.IRvalues[4]);
  Serial.println("----------------------------------------------------");
}

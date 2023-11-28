
#include <MOTORS.h>
#include <ARRAY.h>
#include <PID.h>

float kp = 0.5;
float ki = 0.5;
float kd = 0.5;

float prevKP = 0;
float prevKI = 0;
float prevKD = 0;

MOTORS motors(6, 7, 8, 9, 10, 11);

ARRAY array(A0, A1, A2, A3, A4);

PID pid(kp, ki, kd);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available())
    {
        prevKD = kd;
        prevKI = ki;
        prevKP = kp;

        char c = Serial.read();
        switch (c)
        {
        case 'A':
            kp = pid.getKP() + 0.01;
            pid.setPID(kp, ki, kd);
            break;
        case 'B':
            ki = pid.getKI() + 0.01;
            pid.setPID(kp, ki, kd);
            break;

        case 'C':
            kd = pid.getKD() + 0.01;
            pid.setPID(kp, ki, kd);
            break;

        case 'D':
            kp = pid.getKP() - 0.01;
            pid.setPID(kp, ki, kd);
            break;

        case 'E':
            ki = pid.getKI() - 0.01;
            pid.setPID(kp, ki, kd);
            break;

        case 'F':
            kd = pid.getKD() - 0.01;
            pid.setPID(kp, ki, kd);
            break;

        default:
            break;
        }
    }

    if ((prevKD != kd) || (prevKI != ki) || (prevKP != kp))
    {
        prevKD = pid.getKD();
        prevKI = pid.getKI();
        prevKP = pid.getKP();

        Serial.print("kp: ");
        Serial.print(kp);
        Serial.print(" ki: ");
        Serial.print(ki);
        Serial.print(" kd: ");
        Serial.println(kd);
    }
}
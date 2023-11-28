#include <MOTORS.h>
MOTORS motors(6, 7, 8, 9, 10, 11);

const int speed = 80;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        case 'A':
            motors.forward(speed);
            break;
        case 'B':
            motors.backward(speed);
            break;
        case 'C':
            motors.turnLeft(speed);
            break;
        case 'D':
            motors.turnRight(speed);
            break;
        case 'E':
            motors.turnAround(speed);
            break;
        case 'F':
            motors.stop();
            break;
        }
    }
}
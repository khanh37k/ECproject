#include "Motor.h"

#define ENA  25
#define IN1  26
#define IN2  27

#define ENB  14
#define IN3  12
#define IN4  13

void motorInit(){
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(0 , 5000, 8);
    ledcAttachPin(ENA, 0);

    ledcSetup(1, 5000, 8);
    ledcAttachPin(ENB, 1);
}

void setMotor(int left, int right){
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    
    digitalWrite(IN1, left >= 0);
    digitalWrite(IN2, left < 0);

    digitalWrite(IN3, right >= 0);
    digitalWrite(IN4, right < 0);

    ledcWrite(0, abs(left));
    ledcWrite(1, abs(right));
}
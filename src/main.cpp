#include <Arduino.h>
#include <Motor.h>
#include <PID.h>
#include <LS.h>
#include <encoder.h>


#define USE_LINE_PID     1
#define USE_ENCODER_PID  0
#define USE_FEEDFORWARD  0


Encoder encoderLeft(18);
Encoder encoderRight(19);

PID pidLine(20, 0, 10);
PID pidLeft(1.0, 0, 0);
PID pidRight(1.0, 0, 0);


int baseSpeed = 150;

void IRAM_ATTR isrRight() {
    encoderRight.tick();
}

void IRAM_ATTR isrLeft() {
    encoderLeft.tick();
}



void setup() {
    Serial.begin(115200);

    motorInit();
    lineInit();
    encoderLeft.begin();
    encoderRight.begin();

    attachInterrupt(18, isrLeft, RISING);
    attachInterrupt(19, isrRight, RISING);
}



void loop() {
    
   
    float error = 0, correction = 0;

    if (USE_LINE_PID) {
        error = getLineError();
        correction = pidLine.compute(error);
    }

    // ===== TARGET =====
    float left_target  = baseSpeed + correction;
    float right_target = baseSpeed - correction;

    // ===== CURRENT =====
    float left_current = 0;
    float right_current = 0;

    if (USE_ENCODER_PID) {
        left_current  = encoderLeft.getSpeed();
        right_current = encoderRight.getSpeed();
    }

    // ===== FEED FORWARD =====
    float Kff = 1.0;
    float FF_left = 0, FF_right = 0;

    if (USE_FEEDFORWARD) {
        FF_left  = Kff * left_target;
        FF_right = Kff * right_target;
    }

    // ===== PID SPEED =====
    float pidL = 0, pidR = 0;

    if (USE_ENCODER_PID) {
        pidL = pidLeft.compute(left_target - left_current);
        pidR = pidRight.compute(right_target - right_current);
    }

    // ===== OUTPUT =====
    int leftPWM, rightPWM;

    if (USE_ENCODER_PID || USE_FEEDFORWARD) {
        leftPWM  = FF_left  + pidL;
        rightPWM = FF_right + pidR;
    } else {
        leftPWM  = left_target;
        rightPWM = right_target;
    }

    setMotor(leftPWM, rightPWM);
Serial.print("L: ");
Serial.print(left_current);
Serial.print(" | R: ");
Serial.println(right_current);
delay(10); // ~100Hz
}
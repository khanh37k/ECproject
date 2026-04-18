#include <Arduino.h>

// ====== SENSOR ======
int sensorPins[5] = {32, 33, 34, 35, 4};
int weights[5] = {-2, -1, 0, 1, 2};

#define USE_LINE_PID     1
#define USE_ENCODER_PID  0
#define USE_FEEDFORWARD  0

// ====== MOTOR ======
#define ENA  25
#define IN1  26
#define IN2  27

#define ENB  14
#define IN3  12
#define IN4  13

#define ENCODER_RIGHT 19
#define ENCODER_LEFT 18

// ====== PID ======
float Kp = 20;
float Ki = 0;
float Kd = 10;

float error = 0;
float last_error = 0;
float integral = 0;

// ====== SPEED ======
int baseSpeed = 150;

float Kp_s = 1.0;
float Ki_s = 0;
float Kd_s = 0;

float lastErrL = 0, lastErrR = 0;
float intL = 0, intR = 0;

// =============================
volatile int pulseLeft = 0;


volatile int pulseRight = 0;

void IRAM_ATTR isrRight() {
    pulseRight++;
}

void IRAM_ATTR isrLeft() {
    pulseLeft++;
}

float getSpeedLeft() {
    static int lastPulse = 0;
    static unsigned long lastTime = 0;

    unsigned long now = millis();
    int delta = pulseLeft - lastPulse;

    float speed = delta / (now - lastTime + 1); // tránh chia 0

    lastPulse = pulseLeft;
    lastTime = now;

    return speed;
}

float getSpeedRight() {
    static int lastPulse = 0;
    static unsigned long lastTime = 0;

    unsigned long now = millis();
    int delta = pulseRight - lastPulse;

    float speed = delta / (now - lastTime + 1); // tránh chia 0

    lastPulse = pulseRight;
    lastTime = now;

    return speed;
}

float getLineError() {
    int sum = 0;
    int count = 0;

    for (int i = 0; i < 5; i++) {
        int val = digitalRead(sensorPins[i]);

        if (val == 0) { // 0 = line đen
            sum += weights[i];
            count++;
        }
    }

    if (count == 0) return last_error; // mất line

    return (float)sum / count;
}

// =============================

void setMotor(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    // LEFT MOTOR
    if (left >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        left = -left;
    }

    // RIGHT MOTOR
    if (right >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        right = -right;
    }

    ledcWrite(0, left);
    ledcWrite(1, right);
}

float PID_line(float error) {
    integral += error;
    float derivative = error - last_error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    last_error = error;
    return output;
}

float PID_left(float target, float current) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    float err = target - current;
    intL += err * dt;
    float d = (err - lastErrL) / (dt + 1e-6);

    float out = Kp_s * err + Ki_s * intL + Kd_s * d;

    lastErrL = err;
    lastTime = now;
    return out;
}

float PID_right(float target, float current) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    float err = target - current;
    intR += err * dt;
    float d = (err - lastErrR) / (dt + 1e-6);

    float out = Kp_s * err + Ki_s * intR + Kd_s * d;

    lastErrR = err;
    lastTime = now;
    return out;
}

// =============================

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(0, 5000, 8);
    ledcAttachPin(ENA, 0);

    ledcSetup(1, 5000, 8);
    ledcAttachPin(ENB, 1);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), isrRight, RISING);
}

// =============================

void loop() {
    
    // ===== LINE =====
    float error = 0, correction = 0;

    if (USE_LINE_PID) {
        error = getLineError();
        correction = PID_line(error);
    }

    // ===== TARGET =====
    float left_target  = baseSpeed + correction;
    float right_target = baseSpeed - correction;

    // ===== CURRENT =====
    float left_current = 0;
    float right_current = 0;

    if (USE_ENCODER_PID) {
        left_current  = getSpeedLeft();
        right_current = getSpeedRight();
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
        pidL = PID_left(left_target, left_current);
        pidR = PID_right(right_target, right_current);
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
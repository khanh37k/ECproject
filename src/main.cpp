#include <Arduino.h>

// ====== SENSOR ======
int sensorPins[5] = {32, 33, 34, 35, 25};
int weights[5] = {-2, -1, 0, 1, 2};

// ====== MOTOR ======
#define ENA  25
#define IN1  26
#define IN2  27

#define ENB  14
#define IN3  12
#define IN4  13


// ====== PID ======
float Kp = 20;
float Ki = 0;
float Kd = 10;

float error = 0;
float last_error = 0;
float integral = 0;

// ====== SPEED ======
int baseSpeed = 150;

// =============================

float getError() {
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

float PID_control(float error) {
    integral += error;
    float derivative = error - last_error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    last_error = error;
    return output;
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
}

// =============================

void loop() {
    error = getError();

    float output = PID_control(error);

    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;

    setMotor(leftSpeed, rightSpeed);

    // DEBUG
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.println(output);
}
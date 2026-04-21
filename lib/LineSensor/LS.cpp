#include "LS.h"

int sensorPins[5] = {32, 33, 34, 35, 4};
int weights[5] = {-2, -1, 0, 1, 2};

float lastLine_error = 0;

void lineInit() {
    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

float getLineError() {
    int sum = 0, count = 0;

    for (int i = 0; i < 5; i++) {
        if (digitalRead(sensorPins[i]) == 0) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0) return lastLine_error;

    lastLine_error = (float)sum / count;
    return lastLine_error;
}
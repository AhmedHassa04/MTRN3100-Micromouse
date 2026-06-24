#include "Motor.h"
#include "Encoder.h"
#include <math.h>

mtrn3100::Motor motor1(11, 12);
mtrn3100::Encoder encoder1(2, 7);
mtrn3100::Motor motor2(9, 10);
mtrn3100::Encoder encoder2(3, 8);

const float WHEEL_DISTANCE = 7.55;
const float circumference = 3.2 * PI;
const float KP_STRAIGHT = 10.0;  // tune this

void resetEncoders() {
    encoder1.count = 0;
    encoder2.count = 0;
}

float wheelDist(mtrn3100::Encoder& enc) {
    return enc.getRotation() / (2.0 * PI) * circumference;
}

void driveDistance(float distance) {
    resetEncoders();
    while (true) {
        float d1 = fabs(wheelDist(encoder1));
        float d2 = fabs(wheelDist(encoder2));
        float avgDist = (d1 + d2) / 2.0;

        if (avgDist >= distance) {
            motor1.setPWM(0);
            motor2.setPWM(0);
            break;
        }

        // error — positive means encoder2 ahead, slow it down
        float error = d2 - d1;
        float correction = KP_STRAIGHT * error;

        motor1.setPWM(-200 + correction);
        motor2.setPWM( 200 - correction);
    }
}

void rotate(float angle_rad) {
    resetEncoders();
    float target_arc = fabs(angle_rad) * (WHEEL_DISTANCE / 2.0);
    int dir = (angle_rad >= 0) ? +1 : -1;
    while (true) {
        float d = fabs((wheelDist(encoder1) + wheelDist(encoder2))) / 2.0;
        if (d >= target_arc) break;
        motor1.setPWM(dir * 100);
        motor2.setPWM(dir * 100);
    }
    motor1.setPWM(0);
    motor2.setPWM(0);
}

void setup() {
    driveDistance(2000.0);
    delay(500);
    for (int i = 0; i < 4; i++) {
        rotate(+PI * 1.1 / 2.0);
        delay(300);
    }
    for (int i = 0; i < 4; i++) {
        rotate(-PI * 1.1 / 2.0);
        delay(300);
    }
}

void loop() {}

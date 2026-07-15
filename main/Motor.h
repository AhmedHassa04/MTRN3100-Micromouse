#pragma once
#include <Arduino.h>
#include <math.h>
namespace mtrn3100 {
// Your original Motor class, with an added invert flag for the mechanically
// flipped motor. PWM logic is unchanged from your original:
//   digitalWrite(dir, HIGH) for pwm >= 0, analogWrite(pwm_pin, abs(pwm)).
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2, bool invert = false)
      : pwm_pin(pwm_pin), dir_pin(in2), invert(invert) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }
    void setPWM(int16_t pwm) {
        pwm = constrain(pwm, -255, 255);
        if (invert) pwm = -pwm;
        if (pwm >= 0) {
            digitalWrite(dir_pin, HIGH);
        } else {
            digitalWrite(dir_pin, LOW);
        }
        analogWrite(pwm_pin, abs(pwm));
    }
private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    const bool invert;
};
}  // namespace mtrn3100

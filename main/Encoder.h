#pragma once
#include <Arduino.h>
#include <math.h>
namespace mtrn3100 {
// Restored to your original design: two encoders, both A-channels on hardware
// interrupt pins (D2 and D3 on the Nano). enc1 pin = interrupt/A channel,
// enc2 pin = direction/B channel.
//   encoder1(2, 7)  -> A on D2 (INT0), B on D7
//   encoder2(3, 8)  -> A on D3 (INT1), B on D8
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        if (instance1 == nullptr) {
            instance1 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        } else {
            instance2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
        }
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
    }
    void readEncoder() {
        noInterrupts();
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;
            direction = 1;
        } else {
            count--;
            direction = -1;
        }
        interrupts();
    }
    float getRotation() {
        noInterrupts();
        long c = count;
        interrupts();
        return (2.0 * PI * c) / counts_per_revolution;
    }
    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }
private:
    static void readEncoderISR1() { if (instance1) instance1->readEncoder(); }
    static void readEncoderISR2() { if (instance2) instance2->readEncoder(); }
public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 700;
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;
private:
    static Encoder* instance1;
    static Encoder* instance2;
};
}  // namespace mtrn3100

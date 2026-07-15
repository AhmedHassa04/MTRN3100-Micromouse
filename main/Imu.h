#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// MPU-6050 (kit part 018-MPU-6050, plugs into J12).
// We only need the Z-axis gyro to integrate heading (yaw) for straight-line
// tracking and turning. Accelerometer is not used for heading here.
//
// USAGE:
//   mtrn3100::IMU imu;
//   imu.begin();          // in setup(), AFTER Wire.begin()
//   imu.calibrate();      // robot MUST be stationary; measures gyro bias
//   ...
//   imu.update();         // call every loop iteration
//   float yaw = imu.getYawRad();  // radians, CCW positive, drift-integrated

namespace mtrn3100 {

class IMU {
public:
    static constexpr uint8_t MPU_ADDR = 0x68;   // ADO low
    // MPU6050 gyro sensitivity at +/-250 dps range = 131 LSB/(deg/s)
    static constexpr float GYRO_LSB_PER_DPS = 131.0f;

    void begin() {
        // Wake device (clear sleep bit in PWR_MGMT_1)
        writeReg(0x6B, 0x00);
        delay(10);
        // Gyro config: +/-250 dps (FS_SEL=0)
        writeReg(0x1B, 0x00);
        // DLPF ~44Hz to reduce noise (CONFIG reg 0x1A = 3)
        writeReg(0x1A, 0x03);
        delay(10);
        prev_us = micros();
    }

    // Robot must be completely still. Averages gyro Z to find zero-rate bias.
    void calibrate(uint16_t samples = 1000) {
        double sum = 0;
        for (uint16_t i = 0; i < samples; i++) {
            sum += readRawGyroZ();
            delayMicroseconds(500);
        }
        gz_bias = sum / samples;
        yaw_rad = 0.0f;
        prev_us = micros();
    }

    // Integrate gyro Z into yaw. Call as fast as possible in your control loop.
    void update() {
        uint32_t now = micros();
        float dt = (now - prev_us) * 1e-6f;
        prev_us = now;
        if (dt <= 0 || dt > 0.2f) return;   // guard against glitches

        float rate_dps = (readRawGyroZ() - gz_bias) / GYRO_LSB_PER_DPS;
        yaw_rad += (rate_dps * (float)M_PI / 180.0f) * dt;
    }

    float getYawRad() const { return yaw_rad; }
    float getYawDeg() const { return yaw_rad * 180.0f / (float)M_PI; }
    void  resetYaw() { yaw_rad = 0.0f; prev_us = micros(); }

private:
    float  gz_bias = 0.0f;
    float  yaw_rad = 0.0f;   // CCW positive
    uint32_t prev_us = 0;

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    int16_t readRawGyroZ() {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47);              // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((int)MPU_ADDR, 2);
        if (Wire.available() < 2) return 0;
        int16_t hi = Wire.read();
        int16_t lo = Wire.read();
        return (hi << 8) | lo;
    }
};

}  // namespace mtrn3100

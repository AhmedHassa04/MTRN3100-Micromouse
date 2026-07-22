#pragma once
#include <Arduino.h>
#include <Wire.h>

// Three VL6180X ToF sensors (kit part Pololu 2489: A2 front, A3/A4 sides).
// All VL6180X ship at the SAME default I2C address (0x29), so you cannot use
// them together without giving each a unique address at boot.
//
// The kit wires each sensor's GPIO0 pin to a separate Nano pin. On the VL6180X,
// GPIO0 doubles as the CE/enable line: driving it LOW holds the chip in reset
// (off the bus), driving it HIGH releases it. The boot sequence is:
//   1. Hold ALL sensors in reset (GPIO0 = LOW).
//   2. Release ONE sensor, change its address, leave it running.
//   3. Repeat for the next sensor.
//
// This driver depends on Pololu's VL6180X Arduino library. Install it via the
// Library Manager ("VL6180X" by Pololu) or your lab-provided copy.
//
//   #include <VL6180X.h>   // <-- required, provided by that library
//
// XSHUT / GPIO0 pin mapping (from MDICS11 schematic):
//   Sensor FRONT (J14, TOF1GP0) -> Nano D10
//   Sensor LEFT  (J15, TOF2GP0) -> Nano D12
//   Sensor RIGHT (J16, TOF3GP0) -> Nano A6   (verify: A6 is analog-in only on
//                                             some Nanos and cannot drive an
//                                             output. If so, move this XSHUT to
//                                             a spare digital pin via J3/J5.)

#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    // Pass the three GPIO0/XSHUT pins and the new I2C addresses you want.
    Lidar(uint8_t xshut_front, uint8_t xshut_left, uint8_t xshut_right,
          uint8_t addr_front = 0x30, uint8_t addr_left = 0x31,
          uint8_t addr_right = 0x32)
      : xsF(xshut_front), xsL(xshut_left), xsR(xshut_right),
        aF(addr_front), aL(addr_left), aR(addr_right) {}

    // Call in setup() AFTER Wire.begin().
    void begin() {
        // 1. Hold all three in reset.
        pinMode(xsF, OUTPUT); digitalWrite(xsF, LOW);
        pinMode(xsL, OUTPUT); digitalWrite(xsL, LOW);
        pinMode(xsR, OUTPUT); digitalWrite(xsR, LOW);
        delay(10);

        bringUp(front, xsF, aF);
        bringUp(left,  xsL, aL);
        bringUp(right, xsR, aR);
    }

    // Distances in millimetres. VL6180X range is ~0-200mm (good for this task,
    // where all setpoints are <=200mm). Returns 255 on timeout/out-of-range.
    uint8_t readFront() { return front.readRangeSingleMillimeters(); }
    uint8_t readLeft()  { return left.readRangeSingleMillimeters();  }
    uint8_t readRight() { return right.readRangeSingleMillimeters(); }

private:
    uint8_t xsF, xsL, xsR;
    uint8_t aF, aL, aR;
    VL6180X front, left, right;

    void bringUp(VL6180X& s, uint8_t xshut, uint8_t addr) {
        digitalWrite(xshut, HIGH);      // release this one only
        delay(50);
        s.init();
        s.configureDefault();
        s.setAddress(addr);             // move off 0x29 so the next won't clash
        s.setTimeout(250);
        // Continuous mode is faster for control loops, but single-shot is
        // simpler and plenty for these low-speed tasks.d
    }
};

}  // namespace mtrn3100

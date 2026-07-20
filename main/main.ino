#include <Wire.h>
#include "Motor.h"
#include "Encoder.h"
#include "IMU.h"
#include "Lidar.h"
#include <math.h>

// ============================================================================
//  PINS -- exactly matching your original working code.
//    motor1(11, 12)   motor2(9, 10)
//    encoder1(2, 7)   encoder2(3, 8)   (A-channels D2/D3 = hardware INT0/INT1)
// ============================================================================

// --- Geometry (centimetres) -------------------------------------------------
const float WHEEL_DIAMETER = 3.2f;
const float CIRCUMFERENCE  = WHEEL_DIAMETER * PI;
const float WHEEL_DISTANCE = 7.55f;

mtrn3100::Motor   motor1(11, 12, /*invert=*/false);
mtrn3100::Motor   motor2(9, 10, /*invert=*/false);
mtrn3100::Encoder encoder1(2, 7);
mtrn3100::Encoder encoder2(3, 8);
mtrn3100::IMU     imu;
mtrn3100::Lidar   lidar(10, 12, A6);   // XSHUT: front, left, right

// ============================================================================
//  GLOBAL HEADING
//  heading_deg holds the robot's CURRENT heading in degrees, measured relative
//  to the INITIAL coordinate frame that is locked once in setup() via
//  imu.resetYaw(). 0 = the direction the robot faced at startup.
//
//  Refreshed by updateHeading() (call it every loop, wherever imu.update() is
//  called). Read heading_deg anywhere to rectify against the initial frame:
//      float err = target_bearing - heading_deg;
//
//  IMPORTANT: resetYaw() is called ONLY in setup(). The movement functions must
//  NOT reset it, or the frame would be redefined mid-run and heading_deg would
//  no longer be relative to the initial frame.
// ============================================================================
float heading_deg = 0.0f;   // current heading, degrees, vs initial frame
float heading_rad = 0.0f;   // same, radians

void updateHeading() {
    imu.update();
    heading_deg = imu.getYawDeg();
    heading_rad = imu.getYawRad();
}

// Wrap an angle to [-180, 180] so heading errors always take the SHORT way
// around. Without this, rotating the robot past +/-180 by hand makes the raw
// integrated yaw keep climbing and the controller would spin the long way back.
float wrapDeg(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// --- Straight-line heading controller (IMU-based, per Task 3.1 tip) ---------
const float KP_HEADING = 10.0f;   // tune; flip sign if it steers the wrong way
const int   BASE_PWM   = 160;
const int   MAX_CORR   = 80;

// --- Turning controller (Task 3.3, IMU closed-loop, pure P + speed cap) -----
// Positive yaw = CCW (left). A clockwise turn is a NEGATIVE target.
const float KP_TURN     = 1.5f;    // proportional gain
const int   TURN_MINPWM = 30;      // min PWM to break stiction (sign-preserving)
const int   TURN_MAXPWM = 100;     // low cap = slow approach = no overshoot
const float TURN_TOL    = 3.0f;    // deg resting band; well inside +/-5 deg mark
const float FLOOR_BAND  = 2.5f;    // apply MINPWM floor only when err > this

// --- Wall-distance controller (Task 3.2) ------------------------------------
const float WALL_SETPOINT_MM = 100.0f;
const float WALL_TOL_MM      = 3.5f;   // rest band, inside the +/-5mm mark
const float KP_WALL          = 1.2f;   // PWM per mm of error; tune
const float KP_WALL_HEADING = 3.0f;   // gentler than KP_HEADING for slow driving
const int   WALL_MINPWM      = 25;     // stiction floor
const int   WALL_MAXPWM      = 90;     // low cap = gentle approach
const float LIDAR_OFFSET_MM  = 5.0f;   // sensor face vs robot front; verify

const float CELL_SIZE_CM = 20.0f;   // 200mm cells. for task 3.4

void resetEncoders() {
    encoder1.count = 0;
    encoder2.count = 0;
}

float wheelDist(mtrn3100::Encoder& enc) {
    return enc.getRotation() / (2.0f * PI) * CIRCUMFERENCE;   // cm
}

// Drive forward `distance_cm`, holding the CURRENT global bearing (captured at
// call time) using the IMU. Does NOT reset the frame.
float target_bearing = 0.0f;        // absolute bearing, multiples of 90 deg

void driveStraight(float distance_cm) {
    resetEncoders();
    while (true) {
        updateHeading();

        float d1 = fabs(wheelDist(encoder1));
        float d2 = fabs(wheelDist(encoder2));
        float avg = (d1 + d2) / 2.0f;
        if (avg >= distance_cm) break;

        float yaw_err = wrapDeg(target_bearing - heading_deg);   // vs global frame
        float corr = KP_HEADING * yaw_err;
        corr = constrain(corr, -MAX_CORR, MAX_CORR);

        motor1.setPWM(-(BASE_PWM - (int)corr));
        motor2.setPWM( (BASE_PWM + (int)corr));

        Serial.print("heading: "); Serial.println(heading_deg);
    }

    motor1.setPWM(0);
    motor2.setPWM(0);
}

// Spin in place. If the robot turns the WRONG way or drives instead of
// spinning, see the notes below the code for the exact fix.
void spinInPlace(int pwm) {
    motor1.setPWM(pwm);
    motor2.setPWM( pwm);
}

// Turn to / hold an ABSOLUTE bearing target_deg (relative to the initial frame)
// for hold_ms. Because it holds against the global frame and never resets it, a
// disturbance (lift + rotate) is driven back to the same absolute bearing.
void holdHeading(float target_deg, uint32_t hold_ms) {
    uint32_t start = millis();

    while (millis() - start < hold_ms) {
        updateHeading();

        float err = wrapDeg(target_deg - heading_deg);   // short-way, vs frame

        Serial.print("heading "); Serial.print(heading_deg);
        Serial.print("  err "); Serial.println(err);

        if (fabs(err) < TURN_TOL) {
            spinInPlace(0);
            continue;
        }

        float output = KP_TURN * err;
        int cmd = (int)output;

        if (fabs(err) > FLOOR_BAND) {
            if (cmd > 0 && cmd < TURN_MINPWM)  cmd = TURN_MINPWM;
            if (cmd < 0 && cmd > -TURN_MINPWM) cmd = -TURN_MINPWM;
        }
        cmd = constrain(cmd, -TURN_MAXPWM, TURN_MAXPWM);

        spinInPlace(cmd);
    }
    spinInPlace(0);
}

// Median-of-3 rejects single-reading spikes from the VL6180X, which are the
// main driver of twitching near the setpoint.
float readFrontFiltered() {
    uint8_t a = lidar.readFront();
    uint8_t b = lidar.readFront();
    uint8_t c = lidar.readFront();
    uint8_t m = (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                        : ((a > c) ? a : ((b > c) ? c : b));
    return (float)m - LIDAR_OFFSET_MM;
}

// Hold the front of the robot at setpoint_mm from the wall. Covers all three
// challenges of Task 3.2 in one continuous call: the wall moving just changes
// the error, and the controller follows it.
void holdWallDistance(float setpoint_mm, uint32_t hold_ms) {
    uint32_t start = millis();

    updateHeading();
    float wall_bearing = heading_deg;   // hold the bearing we start at

    while (millis() - start < hold_ms) {
        updateHeading();

        float dist = readFrontFiltered();
        float err  = dist - setpoint_mm;

        Serial.print("dist "); Serial.print(dist);
        Serial.print("  err "); Serial.println(err);

        // Heading correction, same as driveStraight.
        float yaw_err = wrapDeg(wall_bearing - heading_deg);
        float corr = constrain(KP_WALL_HEADING * yaw_err, -MAX_CORR, MAX_CORR);

        if (fabs(err) < WALL_TOL_MM) {
            motor1.setPWM(0);
            motor2.setPWM(0);
            continue;
        }

         int cmd = (int)(KP_WALL * err);
        if (fabs(err) > 6.0f) {
            if (cmd > 0 && cmd < WALL_MINPWM)  cmd = WALL_MINPWM;
            if (cmd < 0 && cmd > -WALL_MINPWM) cmd = -WALL_MINPWM;
        }
        cmd = constrain(cmd, -WALL_MAXPWM, WALL_MAXPWM);

        // Same structure as driveStraight: forward drive plus heading trim.
        motor1.setPWM(-(cmd - (int)corr));
        motor2.setPWM( (cmd + (int)corr));
    }
    motor1.setPWM(0);
    motor2.setPWM(0);
}

// --- Task 3.4: chaining movements -------------------------------------------
// Execute one command character.
void doMove(char c) {
    switch (c) {
        case 'f':
            driveStraight(CELL_SIZE_CM);
            break;
        case 'l':                        // left = CCW = positive yaw
            target_bearing = wrapDeg(target_bearing + 90.0f);
            holdHeading(target_bearing, 3000);
            break;
        case 'r':                        // right = CW = negative yaw
            target_bearing = wrapDeg(target_bearing - 90.0f);
            holdHeading(target_bearing, 3000);
            break;
    }
}

// Run a full command string, e.g. "lfrfflfr".
void runSequence(const char* seq) {
    for (uint8_t i = 0; seq[i] != '\0'; i++) {
        doMove(seq[i]);
        delay(200);                      // brief settle between moves
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    imu.begin();
    imu.calibrate();      // robot MUST be stationary here
    lidar.begin();

    delay(500);

    imu.resetYaw();       // <-- LOCK the initial frame ONCE, only here
    updateHeading();      // heading_deg now 0 = initial frame

    // ---- TASK 3.1: drive 1 metre straight ----
    //driveStraight(100.0f);

     // ---- TASK 3.2: drive and stop ----
    //holdWallDistance(WALL_SETPOINT_MM, 600000);

    // ---- TASK 3.3: turning to an absolute -90 deg bearing ----
    //holdHeading(-90.0f, 60000);
    

    //TASK3.4 CONTINOUS OPERATION 
     runSequence("lfrfflfr");   // replace with the string given on the day
}

// Keeps heading_deg live and prints it. Read heading_deg anywhere in your code.
void loop() {
    //Serial.println(lidar.readFront());
    //delay(100);    
    //updateHeading();
    //Serial.print("heading: "); Serial.println(heading_deg);
    //delay(100);
}

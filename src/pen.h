#ifndef Pen_h
#define Pen_h
#include <Arduino.h>
#include <ESP32Servo.h>
const int RETRACT_DISTANCE = 20;

// The "down" position is now a fixed constant rather than something the user
// calibrates a servo angle for - see PenCalibrationPhase. The servo always moves to
// exactly this angle, and the *pen itself* is adjusted in its holder (loosen, slide
// until it just touches the writing surface, tighten) to match. This means every real
// pen transition covers exactly the same, known distance, rather than a value that
// varies per calibration session - see PEN_SETTLE_MS below.
constexpr int PEN_UP_DEGREES = 90;
constexpr int PEN_DOWN_DEGREES = 0; // full servo extension - gives the most room to
                                     // mechanically adjust the pen's own position

// Servo speed, from the datasheet: 60 degrees in 0.11 seconds, at rated voltage and no
// load.
const double SERVO_RATED_DEG_PER_SEC = 60.0 / 0.11; // ~545 deg/sec

// Real-world load (the pen mechanism's own friction/spring, plus contact with the wall
// during the down->up transition) will always be slower than the no-load datasheet
// figure - this is a genuine guess at how much slower, not a measured value. If pen
// timing still looks off, this is the first constant to try lowering.
const double SERVO_LOAD_DERATING = 0.5;
const double SERVO_EFFECTIVE_DEG_PER_SEC = SERVO_RATED_DEG_PER_SEC * SERVO_LOAD_DERATING;

// Every pen up/down transition now covers exactly this many degrees (PEN_UP_DEGREES to
// PEN_DOWN_DEGREES or back), since the down position is a fixed constant instead of a
// per-session calibrated value. So rather than estimating a settle time per move, there's
// one fixed settle time, computed once, for a full swing - see doSlowMove() in pen.cpp.
const int PEN_SETTLE_MS = int(ceil((PEN_UP_DEGREES - PEN_DOWN_DEGREES) / SERVO_EFFECTIVE_DEG_PER_SEC * 1000));

class Pen {
    private:
    Servo *servo;
    int slowSpeedDegPerSec = 90;
    int currentPosition = PEN_UP_DEGREES;
    public:
    Pen();
    void setRawValue(int rawValue);
    void slowUp();
    void slowDown();
    bool isDown();
};
#endif
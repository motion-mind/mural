#include "pen.h"

bool shouldStop(int currentDegree, int targetDegree, bool positive) {
    if (positive) {
        return currentDegree > targetDegree;
    } else {
        return currentDegree < targetDegree;
    }
}

void doSlowMove(Pen* pen, int startDegree, int targetDegree, int speedDegPerSec) {
    if (startDegree == targetDegree) {
        return;
    }

    auto startTime = millis();

    bool positive;
    if (targetDegree > startDegree) {
        positive = true;
    } else {
        positive = false;
    }

    auto currentDegree = startDegree;

    while (!(shouldStop(currentDegree, targetDegree, positive))) {
        pen->setRawValue(currentDegree);
        delay(10);

        auto currentTime = millis();
        auto deltaTime = currentTime - startTime;
        auto progressDegrees = int(double(deltaTime) / 1000 * speedDegPerSec);

        if (!positive) {
            progressDegrees = progressDegrees * -1;
        }

        currentDegree = startDegree + progressDegrees;
    }
    pen->setRawValue(targetDegree);

    // Every real pen transition now covers exactly PEN_UP_DEGREES - PEN_DOWN_DEGREES
    // (the down position is a fixed constant, not a per-session calibrated value - see
    // pen.h), so there's one fixed settle time for a full swing rather than a per-move
    // estimate based on distance.
    delay(PEN_SETTLE_MS);
}


Pen::Pen()
{
    servo = new Servo();
    servo->attach(2);
    servo->write(PEN_UP_DEGREES);
    currentPosition = PEN_UP_DEGREES;
}

void Pen::setRawValue(int rawValue) {
    this->servo->write(rawValue);
    currentPosition = rawValue;
}

void Pen::slowUp() {
    doSlowMove(this, currentPosition, PEN_UP_DEGREES, slowSpeedDegPerSec);
    currentPosition = PEN_UP_DEGREES;
}

void Pen::slowDown() {
    doSlowMove(this, currentPosition, PEN_DOWN_DEGREES, slowSpeedDegPerSec);
    currentPosition = PEN_DOWN_DEGREES;
}

bool Pen::isDown() {
    return currentPosition == PEN_DOWN_DEGREES;
}

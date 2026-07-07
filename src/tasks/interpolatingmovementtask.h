#ifndef InterpolatingMovementTask_h
#define InterpolatingMovementTask_h
#include "movement.h"
#include "task.h"
const double INCREMENT = 1;
class InterpolatingMovementTask : public Task {
    private:
    Movement *movement;
    Movement::Point target;
    Movement::Point position;
    double speedMmPerSec;
    public:
    const static char* NAME;
    // speedMmPerSec defaults to defaultSpeedMmPerSec (movement.h) for callers that aren't
    // driven by a planned speed from the job file, e.g. the end-of-job return-to-home move.
    InterpolatingMovementTask(Movement *movement, Movement::Point target, double speedMmPerSec = defaultSpeedMmPerSec);
    bool isDone();
    void startRunning();
    const char* name() {
        return NAME;
    }
};
#endif
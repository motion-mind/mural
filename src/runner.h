#ifndef Runner_h
#define Runner_h
#include "movement.h"
#include "tasks/task.h"
#include "pen.h"
#include "LittleFS.h"

// Binary job format - see tsc/src/binaryFormat.ts for the browser-side encoder.
// Little-endian throughout, which matches the ESP32's native byte order.
#define MURAL_FORMAT_VERSION 3
#define MURAL_OPCODE_PEN_UP 0x00
#define MURAL_OPCODE_PEN_DOWN 0x01
#define MURAL_OPCODE_MOVE 0x02

// Move coordinates are fixed-point int32: 1 unit = 1/MURAL_COORDINATE_SCALE mm.
// Must match COORDINATE_SCALE in tsc/src/binaryFormat.ts exactly.
#define MURAL_COORDINATE_SCALE 100.0

// Move speed is fixed-point uint16: 1 unit = 1/MURAL_SPEED_SCALE mm/s.
// Must match SPEED_SCALE in tsc/src/binaryFormat.ts exactly.
#define MURAL_SPEED_SCALE 10.0

class Runner {
    private:
    Movement *movement;
    Pen *pen;
    void initTaskProvider();
    Task* getNextTask();
    Task* currentTask;
    bool stopped;
    File openedFile;
    double totalDistance;
    double distanceSoFar;
    Movement::Point startPosition;
    Movement::Point targetPosition;
    int progress;
    Task *finishingSequence[1];
    int sequenceIx = 0;
    public:
    Runner(Movement *movement, Pen *pen);
    void start();
    void run();
    void dryRun();
};
#endif
#include "runner.h"
#include "tasks/movementtask.h"
#include "tasks/interpolatingmovementtask.h"
#include "tasks/pentask.h"
#include "pen.h"
#include "LittleFS.h"
using namespace std;

Runner::Runner(Movement *movement, Pen *pen) {
    stopped = true;
    this->movement = movement;
    this->pen = pen;
}

void Runner::initTaskProvider() {
    openedFile = LittleFS.open("/commands");
    if (!openedFile || !openedFile.available()) {
        Serial.println("Failed to open file");
        throw std::invalid_argument("No File");
    }

    uint8_t version;
    if (openedFile.read(&version, 1) != 1 || version != MURAL_FORMAT_VERSION) {
        Serial.println("Bad file - bad version");
        throw std::invalid_argument("bad file");
    }

    float distanceValue;
    if (openedFile.read((uint8_t *)&distanceValue, 4) != 4) {
        Serial.println("Bad file - no distance");
        throw std::invalid_argument("bad file");
    }
    totalDistance = distanceValue;

    float heightValue;
    if (openedFile.read((uint8_t *)&heightValue, 4) != 4) {
        Serial.println("Bad file - no height");
        throw std::invalid_argument("bad file");
    }
    // we actually dont need it, just validating

    Serial.println("Total distance to travel: " + String(totalDistance));

    distanceSoFar = 0;
    progress = -1; // so 0% appears right away
    startPosition = movement->getCoordinates();

    auto homeCoordinates = movement->getHomeCoordinates();
    finishingSequence[0] = new InterpolatingMovementTask(movement, homeCoordinates);
}

void Runner::start() {
    initTaskProvider();
    currentTask = getNextTask();
    currentTask->startRunning();
    stopped = false;
}

Task *Runner::getNextTask()
{
    if (openedFile.available())
    {
        uint8_t opcode;
        openedFile.read(&opcode, 1);

        if (opcode == MURAL_OPCODE_PEN_DOWN)
        {
            //Serial.println("Pen down");
            return new PenTask(false, pen);
        }
        else if (opcode == MURAL_OPCODE_PEN_UP)
        {
            //Serial.println("Pen up");
            return new PenTask(true, pen);
        }
        else if (opcode == MURAL_OPCODE_MOVE)
        {
            int32_t xFixed, yFixed;
            uint16_t speedFixed;
            openedFile.read((uint8_t *)&xFixed, 4);
            openedFile.read((uint8_t *)&yFixed, 4);
            openedFile.read((uint8_t *)&speedFixed, 2);
            targetPosition = Movement::Point(xFixed / MURAL_COORDINATE_SCALE, yFixed / MURAL_COORDINATE_SCALE);
            return new InterpolatingMovementTask(movement, targetPosition, speedFixed / MURAL_SPEED_SCALE);
        }
        else
        {
            Serial.println("Bad opcode in commands file: " + String(opcode));
            throw std::invalid_argument("bad opcode");
        }
    }
    else
    {
        if (sequenceIx < (end(finishingSequence) - begin(finishingSequence))) {
            auto currentIx = sequenceIx;
            sequenceIx = sequenceIx + 1;
            return finishingSequence[currentIx];
        } else {
            // DistanceState::storeDistance(movement->getTopDistance());
            delay(200);
            ESP.restart();
            // unreachable
            return NULL;
        }
    }
}

void Runner::run()
{
    if (stopped)
    {
        return;
    }

    if (currentTask->isDone())
    {
        if (currentTask->name() == InterpolatingMovementTask::NAME) {
            auto distanceCovered = Movement::distanceBetweenPoints(startPosition, targetPosition);
            distanceSoFar += distanceCovered;
            startPosition = targetPosition;
            auto newProgress = int(floor(distanceSoFar / totalDistance * 100));
            if (newProgress > 100) {
                newProgress = 100;
            }
            if (progress != newProgress) {
                Serial.println("Progress: " + String(newProgress));
                progress = newProgress;
            }

        }
        delete currentTask;
        currentTask = getNextTask();
        if (currentTask != NULL)
        {
            currentTask->startRunning();
        }
        else
        {
            stopped = true;
        }
    }
}

void Runner::dryRun() {
    initTaskProvider();
    auto task = getNextTask();
    auto index = 1;
    while (task != NULL) {
        //Serial.println(String(index));
        index = index + 1;
        delete task;
        task = getNextTask();
    }
    Serial.println("All done");
}
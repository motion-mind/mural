#ifndef PenCalibrationPhase_h
#define PenCalibrationPhase_h
#include "notsupportedphase.h"
#include "phasemanager.h"
#include "pen.h"
class PenCalibrationPhase : public NotSupportedPhase {
    private:
    PhaseManager* manager;
    Pen* pen;
    Runner* runner;
    public:
    PenCalibrationPhase(PhaseManager* manager, Pen* pen);
    void setServo(AsyncWebServerRequest *request);
    void confirmPenCalibration(AsyncWebServerRequest *request);
    const char* getName();
    // Called by PhaseManager::setPhase() when (re-)entering this phase: moves the servo
    // to its fixed "down" position immediately, so the person calibrating adjusts the
    // pen's own mechanical position against a known, constant servo angle rather than
    // the other way around - see pen.h.
    void onEnter();
};
#endif

#pragma once

#include "IO/OPRGyro.h"
#include "OPRConstants.h"
#include "OPRSubsystemIncludes.h"

class AutoMode {
   public:
    int id;
    int step;
    std::string name;
    units::degree_t initialAngle;

    // Subsystems
    OPRDriveBase* DriveBase;
    OPRCameras* Cameras;
    OPRGyro* Gyro;
    OPRSensors* Sensors;

    // Timers
    frc::Timer* autoTimer;
    frc::Timer* totalAutoTimer;

    AutoMode() {
        autoTimer = new frc::Timer();
        totalAutoTimer = new frc::Timer();

        DriveBase = OPRDriveBase::GetInstance();
        Cameras = OPRCameras::GetInstance();
        Gyro = OPRGyro::GetInstance();
        Sensors = OPRSensors::GetInstance();

        initialAngle = 0_deg;  // tbd
        Gyro->SetAngle(initialAngle.value());
        step = 0;
    }

    ~AutoMode() {}

    virtual void Run(){};
    virtual void ReflectIfRed(){};

    std::string GetName() { return name; };

    void ResetAuto() {
        step = 0;
        Gyro->SetAngle(initialAngle.value());
        totalAutoTimer->Reset();
        totalAutoTimer->Stop();
    }

    void ResetTimer() {
        autoTimer->Reset();
        autoTimer->Start();
    }

    int GetStep() { return step; }

    OPRVector AMP_OFFSET_START_VECTOR = {52.5_in, 40_in};
    OPRVector SPEAKER_OFFSET_START_VECTOR = {54_in, 0_in};
    OPRVector SOURCE_OFFSET_START_VECTOR = {52.5_in, -87_in};
    OPRVector FAR_OFFSET_START_VECTOR = {52.5_in, -155_in};

    double speed = 180;
    units::second_t timeout = 1.0_s;
};
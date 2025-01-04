#pragma once

#include "AutoMode.h"
#include "OPRIncludes.h"
#include "OPRSubsystemIncludes.h"

// Auto Modes

class OPRAuto : public OPRLoggable {
   public:
    static OPRAuto* GetInstance();

    // public functions
    void DisabledInit();
    void AutonomousInit();
    void AutonomousPeriodic();
    void ShowOnDashboard();

    // control funcs
    void SensorReset();
    void StartTimers();
    void AutoSelector();

    // Getters/Setters
    int GetAutoMode();
    double GetAutoDelay();

   private:
    OPRAuto();
    ~OPRAuto();

    static OPRAuto* instance;

    // Class variables
    int SelectedAutoMode;
    double AutoDelay;

    // Control Functions
    void ResetStates();
    std::string GetAutoModeString();

    OPRDriveBase* DriveBase;
    OPRGyro* Gyro;
    OPRCameras* Cameras;
    OPRControllers* GamePads;

    std::vector<AutoMode*> AutoModes;

    // Timers
    frc::Timer* DelayTimer;
    frc::Timer* AutoTimer;
    frc::Timer* AutoAwaitTimer;
};
#pragma once

#include "../OPRIncludes.h"
#include "../OPRSubsystemIncludes.h"

class OPRSubsystemController : public OPRSubsystem, public OPRLoggable {
   public:
    static OPRSubsystemController* GetInstance();

    void RobotInit();
    void RobotPeriodic();
    void DisabledInit();
    void DisabledPeriodic();
    void TeleopInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
    void ShowOnDashboard();

   private:
    OPRSubsystemController(void);
    ~OPRSubsystemController(void);
    static OPRSubsystemController* instance;

    OPRDriveBase* _DriveBase;
};
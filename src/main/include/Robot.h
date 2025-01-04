// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "IO/OPRGyro.h"
#include "OPRAuto.h"
#include "Subsystems/OPRSubsystemController.h"
#include "Utils/OPRLeds.h"

class Robot : public frc::TimedRobot {
   public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

   private:
    OPRSubsystemController* _SubsystemController = OPRSubsystemController::GetInstance();
    OPRCameras* _Cameras = OPRCameras::GetInstance();
    OPRLidar* _Lidar = OPRLidar::GetInstance();
    OPRLeds* _Leds = OPRLeds::GetInstance();
    OPRAuto* _AutoController = OPRAuto::GetInstance();
    OPRControllers* _GamePads = OPRControllers::GetInstance();

    units::time::second_t lastDashboardTime = 0_s;

    void ShowOnDashboard();
    std::string rioID;
    std::string eventName;
    int matchNumber;
};

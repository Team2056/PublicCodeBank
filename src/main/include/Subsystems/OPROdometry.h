#pragma once

#include "IO/OPRGyro.h"
#include "OPRIncludes.h"
#include "Subsystems/OPRSwerveModule.h"

class OPROdometry {
   public:
    static OPROdometry* GetInstance(std::vector<OPRSwerveModule*> _swerveModule);
    OPROdometry(std::vector<OPRSwerveModule*> _swerveModules);

    OPRPose Update();
    OPRPose CalculateCenter();
    OPRPose RefitGeometry();
    OPRPose GetPose();
    OPRVector GetVelocity();

    // Sets the X and Y position of the robot. NOT THE HEADING
    void SetPose(OPRPose _pose);
    void ToString();

    OPRPose GetCenterOfPair(OPRPose p1, OPRPose p2, OPRVector p1Offset, OPRVector p2Offset);
    OPRPose GetCenterFromWheel(OPRPose p1, OPRVector p1Offset);

    void Reset();

   private:
    std::vector<OPRPose> modulePose;
    std::vector<OPRVector> moduleLastVector;
    std::vector<OPRSwerveModule*> swerveModules;
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;

    OPRPose RobotPosition;
    OPRPose PrevRobotPosition;
    OPRVector RobotVelocity;

    OPRGyro* Gyro;
    ctre::phoenix6::StatusSignal<units::angle::degree_t>* GyroAngle;

    units::second_t prevTime = 0.0_s;
    units::second_t elapsedTime = 0.0_s;
    units::second_t currTime = 0.0_s;

    static OPROdometry* instance;
};
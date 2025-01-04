#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <networktables/NetworkTableInstance.h>

#include "IO/OPRGyro.h"
#include "Utils/OPRLoggable.h"
#include "Utils/OPRTree.h"
#include "Utils/OPRVector.h"

class OPRLimeLight {
   public:
    OPRLimeLight(std::string _id);
    ~OPRLimeLight();

    void RobotPeriodic();

    double GetVerticalOffset();
    double GetHorizontalOffset();
    double GetArea();
    double GetSkew();
    bool GetValidTarget();
    bool RawValidTarget();
    double GetDistance();
    double GetDistanceTY();
    double GetLatency();
    bool IsConnected();
    void SetPipeline(int _pipeline);
    int GetPipeline();
    OPRVector GetTargetVector();
    AprilTag GetCurrentAprilTag();

   private:
    double distance = 0;
    double oldDistance;
    double oldTX;
    double currTX;
    units::angle::degree_t lastKnowHeading;
    double LimeLightHeartbeat[10];

    std::string id;
    std::shared_ptr<nt::NetworkTable> networkTableInstance;
    OPRGyro* _Gyro;

    OPRTree pracDistanceMap;
    OPRTree compDistanceMap;

    units::second_t limelightTime = 0_s;
    units::second_t horizontalTime = 0_s;
    units::second_t TargetValidTime = 0_s;
};
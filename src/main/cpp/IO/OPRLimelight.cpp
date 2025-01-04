#include "IO/OPRLimelight.h"

OPRLimeLight::OPRLimeLight(std::string _id) {
    id = _id;
    networkTableInstance = nt::NetworkTableInstance::GetDefault().GetTable(id);
    _Gyro = OPRGyro::GetInstance();

    pracDistanceMap.InsertValues({
        // TY | Real  |   Tape
        {7.192, 36},     // 0
        {4.731, 42},     // 6
        {2.468, 48},     // 12
        {0.391, 54},     // 18
        {-1.319, 60},    // 24
        {-2.847, 66},    // 30
        {-3.899, 72},    // 36
        {-4.994, 78},    // 42
        {-6.015, 84},    // 48
        {-7.035, 90},    // 54
        {-8.168, 96},    // 60
        {-9.177, 102},   // 66
        {-10.062, 108},  // 72
        {-10.702, 114},  // 78
        {-11.287, 120},  // 84
        {-11.777, 126},  // 90
        {-12.348, 132},  // 96
        {-12.821, 138},  // 102
        {-13.355, 144},  // 108
        {-13.804, 150},  // 114
        {-14.221, 156},  // 120
        {-14.572, 162},  // 126
        {-14.883, 168},  // 132
        {-15.247, 174},  // 138
        {-15.634, 180},  // 144
        {-15.979, 186},  // 150
        {-16.292, 192},  // 156
        {-16.654, 198},  // 162
        {-16.869, 204},  // 168
        {-17.182, 210},  // 174
        {-17.368, 216},  // 180
        {-17.5, 222},    // 186
    });

    compDistanceMap.InsertValues({
        // TY | Real  |   Tape
        {13.71, 38.5},  // 0
        {11.26, 42},    // 6
        {8.42, 48},     // 12
        {5.82, 54},     // 18
        {3.76, 60},     // 24
        {2.24, 66},     // 30
        {0.79, 72},     // 36
        {-0.34, 78},    // 42
        {-1.43, 84},    // 48
        {-2.43, 90},    // 54
        {-3.39, 96},    // 60
        {-4.25, 102},   // 66
        {-5.12, 108},   // 72
        {-5.9, 114},    // 78
        {-6.78, 120},   // 84
        {-7.43, 126},   // 90
        {-8.04, 132},   // 96
        {-8.56, 138},   // 102
        {-8.96, 144},   // 108
        {-9.38, 150},   // 114
        {-9.74, 156},   // 120
        {-10.1, 162},   // 126*
        {-10.48, 168},  // 132
        {-10.85, 174},  // 138+
        {-11.2, 180},   // 144
        {-11.63, 186},  // 150
        {-12.07, 192},  // 156
        {-12.55, 198},  // 162
        {-12.95, 204},  // 168
        {-13.35, 210},  // 174
        {-13.6, 216},   // 180
        {-13.78, 222},  // 186
    });
}

/**
 * @brief Get the vertical offset (TY) of the limelight
 * 
 * @return double 
 */
double OPRLimeLight::GetVerticalOffset() {
    return networkTableInstance->GetNumber("ty", 0.0);
}

/**
 * @brief Get the horizontal offset (TX) of the limelight
 * 
 * @return double 
 */
double OPRLimeLight::GetHorizontalOffset() {
    double Tv = networkTableInstance->GetNumber("tv", 0.0);
    currTX = networkTableInstance->GetNumber("tx", 0.0);

    if (id == "limelight-piece") {
        return networkTableInstance->GetNumber("tx", 0.0);
    }

    if (!Tv && ((frc::Timer::GetFPGATimestamp() - horizontalTime) < 300_ms)) {
        return oldTX - (lastKnowHeading - _Gyro->GetAngle()).value();
    }
    else if (Tv) {
        horizontalTime = frc::Timer::GetFPGATimestamp();
        oldTX = currTX;
        lastKnowHeading = _Gyro->GetAngle();
        return currTX;
    }
    else {
        return 0;
    }
}

/**
 * @brief Get the area of the bounding box
 * 
 * @return double 
 */
double OPRLimeLight::GetArea() {
    return networkTableInstance->GetNumber("ta", 0.0);
}

double OPRLimeLight::GetSkew() {
    double skew = GetHorizontalOffset();
    if (skew > -45) {
        return skew + 90;
    }
    return skew;
}

/**
 * @brief Return if the limelight has a target
 * 
 * @return true 
 * @return false 
 */
bool OPRLimeLight::GetValidTarget() {
    double x = networkTableInstance->GetNumber("tv", 0.0);

    if (id == "limelight-piece") {
        return networkTableInstance->GetNumber("tv", 0.0);
    }

    if ((x == 0) && ((frc::Timer::GetFPGATimestamp() - TargetValidTime) < 300_ms)) {
        return true;
    }
    else if (x == 1) {
        TargetValidTime = frc::Timer::GetFPGATimestamp();
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief Return the raw tv (Valid Target)
 * 
 * @return true 
 * @return false 
 */
bool OPRLimeLight::RawValidTarget() {
    if (networkTableInstance->GetNumber("tv", 0.0) == 1) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief Get the distance of the robot to a target
 * 
 * @return double 
 */
double OPRLimeLight::GetDistance() {
    double Tv = networkTableInstance->GetNumber("tv", 0.0);

    if (id == "limelight-april") { // if using the april tag limelight
        distance = GetDistanceTY(); // get the calculated distance using TY
    }
    else {
        distance = 1;
    }

    if (!Tv && ((frc::Timer::GetFPGATimestamp() - limelightTime) < 350_ms)) { // if no valid target but within 350ms of having a valid target
        return oldDistance;
    }
    else if (Tv) { // if there is a valid target
        limelightTime = frc::Timer::GetFPGATimestamp();
        oldDistance = distance;
        return distance;
    }
    else { // no target
        return 0;
    }
}

/**
 * @brief Get the calculated distance from an april tag using the Vertical Offset of the limelight
 * 
 * @return double 
 */
double OPRLimeLight::GetDistanceTY() {
    double x = GetVerticalOffset();

    if (USE_COMP_VALUES) { // COMP Robot
        if (id == "limelight-april") {
            distance = compDistanceMap.GetValue(x);
        }
        else {
            distance = 1;
        }
    }
    else {  // PRACTICE Robot
        if (id == "limelight-april") {
            distance = pracDistanceMap.GetValue(x);
        }
        else {
            distance = 1;
        }
    }
    return distance;
}

/**
 * @brief Get the latency of the limelight (tl)
 * 
 * @return double 
 */
double OPRLimeLight::GetLatency() {
    return networkTableInstance->GetNumber("tl", 0.0);
}

/**
 * @brief return if the limelight has a connection or not
 * 
 * @return true 
 * @return false 
 */
bool OPRLimeLight::IsConnected() {
    for (int i = 0; i < 9; i++) {
        if (LimeLightHeartbeat[i] != LimeLightHeartbeat[i + 1]) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Set the pipeline of a limelight
 * 
 * @param _pipeline pipeline to set
 */
void OPRLimeLight::SetPipeline(int _pipeline) {
    networkTableInstance->PutNumber("pipeline", _pipeline);
}

/**
 * @brief Return the pipeline number
 * 
 * @return int 
 */
int OPRLimeLight::GetPipeline() {
    return networkTableInstance->GetNumber("getpipe", 0.0);
}

/**
 * @brief Return a vector with the distance to the target and the x offset of the target
 * 
 * @return OPRVector 
 */
OPRVector OPRLimeLight::GetTargetVector() {
    return OPRVector(units::inch_t(GetDistance()), units::degree_t(GetHorizontalOffset()));
}

/**
 * @brief Return the enum of the april tag the limelight is looking at 
 * 
 * @return AprilTag 
 */
AprilTag OPRLimeLight::GetCurrentAprilTag() {
    return (AprilTag)networkTableInstance->GetNumber("tid", 0);
}

void OPRLimeLight::RobotPeriodic() {
    for (int i = 0; i < 9; i++) {
        LimeLightHeartbeat[i] = LimeLightHeartbeat[i + 1];
    }
    LimeLightHeartbeat[9] = GetLatency();
}

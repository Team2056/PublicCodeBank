#include "Subsystems/OPROdometry.h"

OPROdometry* OPROdometry::instance = NULL;

OPROdometry* OPROdometry::GetInstance(std::vector<OPRSwerveModule*> _swerveModules) {
    if (instance == NULL) {
        instance = new OPROdometry(_swerveModules);
    }
    return instance;
}

OPROdometry::OPROdometry(std::vector<OPRSwerveModule*> _swerveModules) {
    swerveModules = _swerveModules;
    Gyro = OPRGyro::GetInstance();

    signals.push_back(Gyro->GyroAngleSignal);

    for (int i = 0; i < 4; i++) {
        signals.push_back(swerveModules[i]->DrivePosition);
        signals.push_back(swerveModules[i]->SteerPosition);

        modulePose.push_back({swerveModules[i]->offset.GetX(), swerveModules[i]->offset.GetY(), swerveModules[i]->GetSteerAngle360()});
        moduleLastVector.push_back({swerveModules[i]->GetDrivePosition(), swerveModules[i]->GetSteerAngle360()});
    }

    prevTime = 0_s;
    currTime = 0_s;
}

/**
 * @brief Update the odometry of the robot
 * 
 * @return OPRPose 
 */
OPRPose OPROdometry::Update() {
    ctre::phoenix6::BaseStatusSignal::WaitForAll(15_ms, signals);
    currTime = frc::Timer::GetFPGATimestamp();
    elapsedTime = currTime - prevTime;

    for (int i = 0; i < 4; i++) {
        OPRVector currVector = {swerveModules[i]->GetDrivePosition(), swerveModules[i]->GetSteerAngle360() + units::degree_t(Gyro->GetAngle())};
        OPRVector difference = {currVector.GetMag() - moduleLastVector[i].GetMag(), swerveModules[i]->GetSteerAngle360() + units::degree_t(Gyro->GetAngle())};

        modulePose[i].AddVector(difference);
        moduleLastVector[i] = currVector;
    }
    CalculateCenter();
    RefitGeometry();

    RobotVelocity = OPRVector(RobotPosition.GetX(), RobotPosition.GetY());
    RobotVelocity.SubtractVectors(OPRVector(PrevRobotPosition.GetX(), PrevRobotPosition.GetY()));
    RobotVelocity.SetMag(RobotVelocity.GetMag() / units::scalar_t(elapsedTime.value()));

    prevTime = currTime;
    PrevRobotPosition = RobotPosition;
    return RobotPosition;
}

/**
 * @brief Determine the center of the robot
 * 
 * @return OPRPose 
 */
OPRPose OPROdometry::CalculateCenter() {
    units::inch_t totalX = 0_in;
    units::inch_t totalY = 0_in;
    std::vector<OPRPose> tmpPose;

    for (int i = 0; i < 4; i++) {
        tmpPose.push_back(GetCenterFromWheel(modulePose[i], swerveModules[i]->offset));
    }

    for (unsigned int i = 0; i < tmpPose.size(); i++) {
        totalX += tmpPose[i].GetX();
        totalY += tmpPose[i].GetY();
    }

    totalX /= 4.0;
    totalY /= 4.0;

    RobotPosition.SetPose(totalX, totalY, Gyro->GetAngle());

    return RobotPosition;
}

OPRPose OPROdometry::RefitGeometry() {
    for (int i = 0; i < 4; i++) {
        units::inch_t newX = RobotPosition.GetX() + swerveModules[i]->offset.GetMag() * units::math::cos(Gyro->GetAngle() + swerveModules[i]->offset.GetAngle());
        units::inch_t newY = RobotPosition.GetY() + swerveModules[i]->offset.GetMag() * units::math::sin(Gyro->GetAngle() + swerveModules[i]->offset.GetAngle());
        modulePose[i].SetX(newX);
        modulePose[i].SetY(newY);
    }

    return RobotPosition;
}

OPRPose OPROdometry::GetCenterFromWheel(OPRPose p1, OPRVector p1Offset) {
    units::degree_t currAngle = Gyro->GetAngle();
    units::inch_t offsetXAvg = p1Offset.GetMag() * units::math::cos(currAngle + p1Offset.GetAngle());
    units::inch_t offsetYAvg = p1Offset.GetMag() * units::math::sin(currAngle + p1Offset.GetAngle());
    p1.Subtract({offsetXAvg, offsetYAvg});
    return p1;
}

OPRPose OPROdometry::GetCenterOfPair(OPRPose p1, OPRPose p2, OPRVector p1Offset, OPRVector p2Offset) {
    units::degree_t currAngle = Gyro->GetAngle();
    units::inch_t xAvg = (p1.GetX() + p2.GetX()) / 2;
    units::inch_t yAvg = (p1.GetY() + p2.GetY()) / 2;
    units::inch_t offsetXAvg = (p1Offset.GetX() + p2Offset.GetX()) / 2;
    units::inch_t offsetYAvg = (p1Offset.GetY() + p2Offset.GetY()) / 2;

    OPRVector offsetVector = {offsetXAvg, offsetYAvg};

    offsetXAvg = offsetVector.GetMag() * units::math::cos(currAngle + offsetVector.GetAngle());
    offsetYAvg = offsetVector.GetMag() * units::math::sin(currAngle + offsetVector.GetAngle());

    return {xAvg - offsetXAvg, yAvg - offsetYAvg, currAngle};
}

/**
 * @brief Reset the odometry to 0
 * 
 */
void OPROdometry::Reset() {
    for (int i = 0; i < 4; i++) {
        modulePose[i].SetPose(swerveModules[i]->offset.GetX(), swerveModules[i]->offset.GetY(), swerveModules[i]->GetSteerAngle360());
        moduleLastVector[i] = {swerveModules[i]->GetDrivePosition(), swerveModules[i]->GetSteerAngle360()};
    }
}

/**
 * @brief Return the position of the robot
 * 
 * @return OPRPose 
 */
OPRPose OPROdometry::GetPose() {
    return RobotPosition;
}

/**
 * @brief Return the velocity of the robot
 * 
 * @return OPRVector 
 */
OPRVector OPROdometry::GetVelocity() {
    return RobotVelocity;
}

/**
 * @brief Set the position of the robot
 * 
 * @param _pose 
 */
void OPROdometry::SetPose(OPRPose _pose) {
    for (int i = 0; i < 4; i++) {
        modulePose[i].SetPose(swerveModules[i]->offset.GetX() + _pose.GetX(), swerveModules[i]->offset.GetY() + _pose.GetY(), swerveModules[i]->GetSteerAngle360());
        moduleLastVector[i] = {swerveModules[i]->GetDrivePosition(), swerveModules[i]->GetSteerAngle360()};
    }
    CalculateCenter();
    RefitGeometry();
}

void OPROdometry::ToString() {
    printf("Odometry: x %lf y %lf deg %lf\n", RobotPosition.GetX().value(), RobotPosition.GetY().value(), RobotPosition.GetAngle().value());
}
#include "./IO/OPRGyro.h"

OPRGyro* OPRGyro::instance = NULL;

OPRGyro* OPRGyro::GetInstance() {
    if (instance == NULL) {
        instance = new OPRGyro();
    }
    return instance;
}

OPRGyro::OPRGyro(void) {
    Gyro = new ctre::phoenix6::hardware::Pigeon2(PIGEON_ID, "DriveBus");
    GyroAngleTimer = new frc::Timer();
    GyroAngleTimer->Start();
    GyroPitchTimer = new frc::Timer();
    GyroPitchTimer->Start();
    GyroRollTimer = new frc::Timer();
    GyroRollTimer->Start();


    GyroAngleSignal = &Gyro->GetYaw();
    GyroPitchSignal = &Gyro->GetRoll();
}

/**
 * @brief Get the angle of the gyro
 * 
 * @return units::degree_t 
 */
units::degree_t OPRGyro::GetAngle() {
    return units::degree_t(Gyro->GetYaw().GetValue());
}

/**
 * @brief Get the pitch of the gyro
 * 
 * @return units::degree_t 
 */
units::degree_t OPRGyro::GetPitch() {
    return units::degree_t(Gyro->GetPitch().GetValue());
}

/**
 * @brief Get the roll of the gyro
 * 
 * @return units::degree_t 
 */
units::degree_t OPRGyro::GetRoll() {
    return units::degree_t(Gyro->GetRoll().GetValue());
}

/**
 * @brief Get the rate of change of the angle
 * 
 * @return double 
 */
double OPRGyro::GetAngleRate() {
    double res = (GetAngle().value() - lastAngle) / GyroAngleTimer->Get().value();
    lastAngle = GetAngle().value();
    GyroAngleTimer->Reset();
    return res;
}

/**
 * @brief get the x acceleration of the gyro
 * 
 * @return double 
 */
double OPRGyro::GetXAccel() {
    return Gyro->GetAccelerationX().GetValue().value();
}

/**
 * @brief Get the y acceleration of the gyro
 * 
 * @return double 
 */
double OPRGyro::GetYAccel() {
    return Gyro->GetAccelerationY().GetValue().value();
}

/**
 * @brief Get the z acceleration of the gyro
 * 
 * @return double 
 */
double OPRGyro::GetZAccel() {
    return Gyro->GetAccelerationZ().GetValue().value();
}

/**
 * @brief Get the angle of the gyro
 * 
 * @param pos angle to set
 */
void OPRGyro::SetAngle(double pos) {
    Gyro->SetYaw(units::angle::degree_t(pos));
}

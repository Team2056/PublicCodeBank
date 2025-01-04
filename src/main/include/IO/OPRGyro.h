#pragma once

#include <frc/Timer.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "./OPRConstants.h"

class OPRGyro {
   public:
    static OPRGyro* GetInstance();
    OPRGyro(void);

    /**
     * Returns the angle (yaw) of pigeon 2.
     * @return (degree_t) Angle of gyro in degrees
     */
    units::degree_t GetAngle();

    /**
     * Get the pitch of the gyro
     * @return (degree_t) pitch of gyro
     */
    units::degree_t GetPitch();

    /**
     * Get the roll of the gyro
     * @return (degree_t) roll of gyro
     */
    units::degree_t GetRoll();

    double GetAngleRate();

    // double GetPitchRate();

    // /**
    //  * Speed that robot is rotating
    //  * @return (double) robot rotation speed
    //  */
    // double GetRollRate();

    /**
     * @brief Acceleration of the robot in the X axis
     * @return (double) X axis Acceleration
     */
    double GetXAccel();

    /**
     * @brief Acceleration of the robot in the Y axis
     * @return (double) Y axis Acceleration
     */
    double GetYAccel();

    /**
     * @brief Acceleration of the robot in the z axis
     * @return (double) Z axis Acceleration
     */
    double GetZAccel();

    /**
     * Will set the angle (yaw) of pigeon 2
     * @param pos (double) angle to set in degrees
     */
    void SetAngle(double pos);

    ctre::phoenix6::StatusSignal<units::degree_t>* GyroAngleSignal;
    ctre::phoenix6::StatusSignal<units::degree_t>* GyroPitchSignal;

   private:
    ctre::phoenix6::hardware::Pigeon2* Gyro;

    frc::Timer* GyroAngleTimer;
    frc::Timer* GyroPitchTimer;
    frc::Timer* GyroRollTimer;

    double lastAngle;
    double lastRoll;
    double lastPitch;

    static OPRGyro* instance;
};
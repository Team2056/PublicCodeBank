#pragma once
#include "./OPRIncludes.h"

class OPRSwerveModule {
   public:
    OPRSwerveModule(int driveID, int steerId, int encoderID, units::inch_t _xOffset, units::inch_t _yOffset, bool invertDriveMotor = false, bool invertSteerMotor = false);
    // frc methods
    void TeleopInit();

    /**
     * Will set the output of th emodule to the new swerve state
     * @param newState a OPRModuleVelocity with a magnitude and direction
     */
    void SetVelocity(OPRModuleVelocity _velocity);

    /**
     * will reset the encoder position of the drive motor to 0 turn_t
     */
    void ResetDriveEncoder();

    // Getters/Setters

    /**
     * Set the Neutral mode of drive motor
     * @param _brakesOn brakes enabled
     */
    void BrakeDriveMotor(bool _brakesOn);

    /**
     * Get the angle of the steering motor in degrees
     * @return Total angle of the motor
     */
    units::degree_t GetSteerAngle();

    /**
     * Get the angle of a module from 0-360 degrees
     * @return The angle of the module from 0 - 360 degrees
     */
    units::degree_t GetSteerAngle360();

    /**
     * Get the current velocity of the drive motor
     * @return Angular velocity of drive motor in TPS
     */
    units::turns_per_second_t GetDriveVelocity();

    /**
     * Get the angle of the steering motor in degrees
     * @return Angular velocity of the drive motor in V
     */
    double GetDriveClosedLoopOutput();

    /**
     * Get the current distance the drive motor has traveled
     * @return Distance drive motor travels in inches
     */
    units::inch_t GetDrivePosition();

    /**
     * @brief Returns the temp of the drive motor
     * @return
     */
    double GetDriveTemperature();

    /**
     * @brief Returns the temp of the steer motor
     * @return
     */
    double GetSteerTemperature();

    /**
     * Get the current pose of a module
     * @return Pose with x/y offset and current angle of module
     */
    OPRPose GetPose();

    bool IsCoast();

    void SetEncoderOffset(double _offset);

    // public vars
    OPRVector offset{0_in, 0_in};
    ctre::phoenix6::StatusSignal<units::angle::turn_t>* SteerPosition;
    ctre::phoenix6::StatusSignal<units::angle::turn_t>* DrivePosition;

   private:
    ctre::phoenix6::hardware::TalonFX* DriveMotor;
    ctre::phoenix6::hardware::TalonFX* SteerMotor;
    ctre::phoenix6::hardware::CANcoder* AbsoluteEncoder;

    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};
    ctre::phoenix6::configs::TalonFXConfiguration steerConfigs{};

    ctre::phoenix6::controls::PositionVoltage steerPositionVoltage{0_deg, 0_tps, true, 0_V, 0, true};
    ctre::phoenix6::controls::VelocityVoltage driveVelocityVoltage{0_tps, 0_tr_per_s_sq, true, 0_V, 0, true};

    ctre::phoenix6::controls::VoltageOut PercentOut{0_V, true, false};

    units::turn_t startPosition;
    units::degree_t startAngleOffset;

    OPRModuleVelocity OptimizeState(OPRModuleVelocity _state);
};
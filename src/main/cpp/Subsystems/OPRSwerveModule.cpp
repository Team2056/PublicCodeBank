#include "Subsystems/OPRSwerveModule.h"

OPRSwerveModule::OPRSwerveModule(int driveID, int steerID, int encoderID, units::inch_t _xOffset, units::inch_t _yOffset, bool invertDriveMotor, bool invertSteerMotor) {
    DriveMotor = new ctre::phoenix6::hardware::TalonFX(driveID, "DriveBus");
    SteerMotor = new ctre::phoenix6::hardware::TalonFX(steerID, "DriveBus");
    AbsoluteEncoder = new ctre::phoenix6::hardware::CANcoder(encoderID, "DriveBus");

    steerConfigs.Slot0.kP = 80;
    steerConfigs.Slot0.kD = 0.5;
    steerConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    steerConfigs.Feedback.FeedbackRemoteSensorID = encoderID;
    steerConfigs.Feedback.SensorToMechanismRatio = 1.0;
    steerConfigs.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
    steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    if (invertSteerMotor) {
        steerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    }
    else {
        steerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    }
    steerConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
    steerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    steerConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;        // limit to 30 amps
    steerConfigs.CurrentLimits.SupplyCurrentThreshold = 50.0;    // if we exceed 50 amps
    steerConfigs.CurrentLimits.SupplyTimeThreshold = 1.0;        // for at least 1 second
    steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;  // and enable it
    steerConfigs.CurrentLimits.StatorCurrentLimit = 80.0;        // limit stator to 80 amps
    steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;  // and enable it

    SteerMotor->GetConfigurator().Apply(steerConfigs);

    driveConfigs.Slot0.kP = 0.11;
    driveConfigs.Slot0.kI = 0.0;
    driveConfigs.Slot0.kD = 0.0;
    driveConfigs.Slot0.kV = 0.109;
    driveConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    if (invertDriveMotor) {
        driveConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    }
    else {
        driveConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    }
    driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    driveConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;        // limit to 60 amps
    driveConfigs.CurrentLimits.SupplyCurrentThreshold = 80.0;    // if we exceed 80 amps
    driveConfigs.CurrentLimits.SupplyTimeThreshold = 2.0;        // for at least 2 second
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;  // and enable it
    driveConfigs.CurrentLimits.StatorCurrentLimit = 100.0;       // limit stator to 100 amp
    driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;  // and enable it

    DriveMotor->GetConfigurator().Apply(driveConfigs);

    offset = {_xOffset, _yOffset};

    SteerPosition = &SteerMotor->GetPosition();
    DrivePosition = &DriveMotor->GetPosition();

    // DrivePosition->SetUpdateFrequency(10_ms);
}

void OPRSwerveModule::TeleopInit() {
    SteerMotor->SetControl(ctre::phoenix6::controls::PositionDutyCycle(SteerMotor->GetPosition().GetValue()));
}

void OPRSwerveModule::SetVelocity(OPRModuleVelocity _velocity) {
    _velocity = OptimizeState(_velocity);

    DriveMotor->SetControl(driveVelocityVoltage.WithVelocity(_velocity.magnitude));
    SteerMotor->SetControl(steerPositionVoltage.WithPosition(_velocity.direction));
}

void OPRSwerveModule::ResetDriveEncoder() {
    DriveMotor->SetPosition(0_tr);
}

units::degree_t OPRSwerveModule::GetSteerAngle() {
    return SteerMotor->GetPosition().GetValue();
}

units::degree_t OPRSwerveModule::GetSteerAngle360() {
    units::degree_t dir(units::math::fmod(GetSteerAngle(), 360_deg));
    OPRCalculations::NormalizeAngle(dir);
    return dir;
}

units::turns_per_second_t OPRSwerveModule::GetDriveVelocity() {
    return DriveMotor->GetVelocity().GetValue();
}

void OPRSwerveModule::BrakeDriveMotor(bool _brakesOn) {
    if (_brakesOn == true) {
        driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    }
    else {
        DriveMotor->SetControl(PercentOut.WithOutput(0_V));
        driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    }

    DriveMotor->GetConfigurator().Apply(driveConfigs);
}

bool OPRSwerveModule::IsCoast() {
    if (driveConfigs.MotorOutput.NeutralMode == ctre::phoenix6::signals::NeutralModeValue::Coast) {
        return true;
    }
    else {
        return false;
    }
}

double OPRSwerveModule::GetDriveClosedLoopOutput() {
    return DriveMotor->GetClosedLoopOutput().GetValue();
}

units::inch_t OPRSwerveModule::GetDrivePosition() {
    return units::inch_t(DriveMotor->GetPosition().GetValue().value()) * DRIVE_GEAR_RATIO * (WHEEL_DIAMETER * std::numbers::pi);
}

double OPRSwerveModule::GetDriveTemperature() {
    return DriveMotor->GetDeviceTemp().GetValue().value();
}

double OPRSwerveModule::GetSteerTemperature() {
    return SteerMotor->GetDeviceTemp().GetValue().value();
}

OPRPose OPRSwerveModule::GetPose() {
    return {offset.GetX(), offset.GetY(), 0_deg};
}

void OPRSwerveModule::SetEncoderOffset(double _offset) {
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};

    encoderConfigs.MagnetSensor.MagnetOffset = _offset;

    AbsoluteEncoder->GetConfigurator().Apply(encoderConfigs);
}

OPRModuleVelocity OPRSwerveModule::OptimizeState(OPRModuleVelocity _state) {
    units::degree_t dir = GetSteerAngle360();

    units::degree_t delta = _state.direction - dir;
    OPRCalculations::NormalizeAngle(delta);

    if (units::math::fabs(delta) > 90_deg && units::math::fabs(delta) < 180_deg) {
        _state.magnitude *= -1;
        if (delta > 90_deg) {
            _state.direction -= 180_deg;
        }
        else {
            _state.direction += 180_deg;
        }
    }

    return _state;
}
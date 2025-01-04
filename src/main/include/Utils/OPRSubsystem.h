#pragma once

#include "IO/OPRControllers.h"

class OPRSubsystem {
   public:
    virtual void RobotInit(){};
    virtual void RobotPeriodic(){};
    virtual void DisabledInit(){};
    virtual void DisabledPeriodic(){};
    virtual void AutonomousInit(){};
    virtual void AutonomousPeriodic(){};
    virtual void TeleopInit(){};
    virtual void TeleopPeriodic(){};

    OPRControllers* GamePads = OPRControllers::GetInstance();

    ctre::phoenix6::controls::VoltageOut PercentOut{0_V, true, false};
    ctre::phoenix6::controls::PositionVoltage VoltageMove{0_tr, 0_tps, true, 0_V};
    ctre::phoenix6::controls::VelocityVoltage VelocityOut{0_tps};
    ctre::phoenix6::controls::MotionMagicVelocityVoltage MotionMagicVelocity{0_tps, 0_tr_per_s_sq, true, 0_V};
    ctre::phoenix6::controls::MotionMagicVoltage MotionMagicMove{0_tr, true, 0_V, false};
};
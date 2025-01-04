#pragma once

// clag-format off
// Unit Includes
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

// General Libs
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cmath>
#include <iostream>

// FRC/WPI Libs
#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/Solenoid.h>
#include <frc/Counter.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/SpanExtras.h>
#include <wpinet/PortForwarder.h>

// CTRE
#include <ctre/phoenix6/CANCoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

// OPR
#include "IO/OPRCameras.h"
#include "IO/OPRLidar.h"
#include "OPRCalculations.h"
#include "OPRConstants.h"
#include "Utils/OPRLoggable.h"
#include "Utils/OPRPose.h"
#include "Utils/OPRSlewRateLimiter.h"
#include "Utils/OPRSubsystem.h"
#include "Utils/OPRVector.h"
#include "Utils/SimPID.h"

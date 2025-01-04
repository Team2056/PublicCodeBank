#pragma once

#include <units/angle.h>

#include "IO/OPRXbox.h"

// enums

enum Alliances {
    RED,
    BLUE
};

enum AprilTag {
    BLUE_SOURCE_STAGE_SIDE = 1,
    BLUE_SOURCE_DRIVER_STATION_SIDE = 2,
    RED_SPEAKER_OUTER = 3,
    RED_SPEAKER_CENTER = 4,
    RED_AMP = 5,
    BLUE_AMP = 6,
    BLUE_SPEAKER_CENTER = 7,
    BLUE_SPEAKER_OUTER = 8,
    RED_SOURCE_DRIVER_STATION_SIDE = 9,
    RED_SOURCE_STAGE_SIDE = 10,
    RED_STAGE_SOURCE_SIDE = 11,
    RED_STAGE_AMP_SIDE = 12,
    RED_STAGE_CENTER = 13,
    BLUE_STAGE_CENTER = 14,
    BLUE_STAGE_AMP_SIDE = 15,
    BLUE_STAGE_SOURCE_SIDE = 16
};

// COMP VALS
inline bool USE_COMP_VALUES;
inline Alliances currAlliance;

// hardware ids
#define PIGEON_ID 50
#define LED_PORT 0
#define HANGER_SOLENOID_ID 1
#define LEFT_FLYWHEEL_MOTOR_ID 9
#define RIGHT_FLYWHEEL_MOTOR_ID 10
#define FEEDER_MOTOR_ID 11
#define PIVOT_MOTOR_ID 12
#define KICKER_MOTOR_ID 13
#define INTAKE_MOTOR_ID 14
#define SHOOTER_BEAM_BREAK_ID 0
#define INTAKE_BEAM_BREAK_ID 1

// drivebase ids
#define FR_DRIVE_MOTOR_ID 1
#define FR_STEER_MOTOR_ID 2
#define FR_ABSOLUTE_ENCODER_ID 41
#define FL_DRIVE_MOTOR_ID 3
#define FL_STEER_MOTOR_ID 4
#define FL_ABSOLUTE_ENCODER_ID 42
#define BR_DRIVE_MOTOR_ID 5
#define BR_STEER_MOTOR_ID 6
#define BR_ABSOLUTE_ENCODER_ID 43
#define BL_DRIVE_MOTOR_ID 7
#define BL_STEER_MOTOR_ID 8
#define BL_ABSOLUTE_ENCODER_ID 44

// Constants
static constexpr double RAD_TO_DEG = 180 / std::numbers::pi;
static constexpr double DEG_TO_RAD = std::numbers::pi / 180;

static constexpr units::scalar_t DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);  // L3+ratio
static constexpr units::scalar_t STEER_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
static constexpr units::scalar_t WHEEL_DIAMETER = 3.834;

static constexpr units::inch_t X_OFFSET_FORWARD = 10.876_in;
static constexpr units::inch_t X_OFFSET_BACK = 12.626_in;

static constexpr units::inch_t Y_OFFSET = 11.626_in;

static constexpr units::turns_per_second_t MAX_SPEED_TPS = 110_tps;
static constexpr double MAX_ROTATIONAL_SPEED = 2;  // turns of robot
static constexpr double MAX_SPEED = 198;           // inches per second

// structs

struct OPRModuleVelocity {
    units::turns_per_second_t magnitude;
    units::degree_t direction;
};

// controls

// driver

#define D_SPEAKER_SHOT OPRXbox::A
// #define D_AMP_SHOT OPRXbox::B
#define D_AMP_SHOT OPRXbox::DL
#define D_PASS_SHOT OPRXbox::DR
#define D_HARDSTOP_SHOT OPRXbox::Y
#define D_POOP_SHOT OPRXbox::START
#define D_HOME OPRXbox::X
#define D_PODIUM_HIGH OPRXbox::B
// #define D_PODIUM_HIGH OPRXbox::Y
#define D_INTAKE_OFF OPRXbox::LB
#define D_INTAKE_REVERSE OPRXbox::RB
#define D_FIELD_DRIVE OPRXbox::LS
#define D_ROBOT_DRIVE OPRXbox::RS
#define D_INTAKE_ON OPRXbox::LT
#define D_SHOOT OPRXbox::RT
#define D_HANGER_UP OPRXbox::DU
#define D_HANGER_DOWN OPRXbox::DD

// operator

#define O_MANUAL_PODIUM_SHOT OPRXbox::Y
#define O_MANUAL_SUBWOOFER_SHOT OPRXbox::B
#define O_MANUAL_AMP_SHOT OPRXbox::A
#define O_MANUAL_INTAKE_OUT OPRXbox::LB
#define O_MANUAL_SHOOTER_OFF OPRXbox::RB
#define O_MANUAL_INTAKE_IN OPRXbox::LT
#define O_MANUAL_SHOOTER_ON OPRXbox::RT
#define O_SIGNAL_HP OPRXbox::DU
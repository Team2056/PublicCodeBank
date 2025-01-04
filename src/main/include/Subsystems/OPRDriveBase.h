#pragma once

#include <math.h>

#include "IO/OPRGyro.h"
#include "IO/OPRSensors.h"
#include "OPRIncludes.h"
#include "OPROdometry.h"
#include "OPRSwerveModule.h"

class OPRDriveBase : public OPRLoggable, public OPRSubsystem {
   public:
    static OPRDriveBase* GetInstance();

    // frc methods
    void RobotInit();
    void RobotPeriodic();
    void DisabledInit();
    void DisabledPeriodic();
    void TeleopInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
    void ShowOnDashboard();

    typedef enum DriveStates {
        IDLE,
        FIELD_DRIVE,
        ROBOT_DRIVE
    } DriveStates;

    void ResetDrive();
    void SetState(DriveStates _state);
    DriveStates GetState();
    void ResetOdometryPos(OPRVector offsetVector);
    /**
     * @brief robot drives to a specified point on a cartesian plane relative to its original position (0,0)
     *
     * @param point the point to drive to
     * @param tolerance how close to the point the robot has to be within to return true
     * @param maxVel the max velocity the robot can drive to the point at
     * @param maxRVel the max rotational velocity of the robot
     * @param robotYComp y component factor to be added to the robots y velocity
     * @param isContinuous runs in continues mode
     * @return if we are within the tolerance
     */
    bool DriveToPoint(OPRPose point, double tolerance = 10.0, double maxVel = MAX_SPEED, double maxRVel = MAX_ROTATIONAL_SPEED, double robotYComp = 0.0, bool isContinuous = false);

    OPRVector GetRobotVelocity();
    OPRPose GetPose();

   private:
    OPRDriveBase(void);
    ~OPRDriveBase(void);
    static OPRDriveBase* instance;

    void StateMachine();

    void RobotCentricDrive(double xVel, double yVel, double rVel);
    void FieldCentricDrive(double xVel, double yVel, double rVel, double robotYComp = 0.0);

    void ResetOdometry();

    // Getters/Setters
    std::string GetStateStr();

    units::degree_t GetHeadingToPoint(OPRPose p);

    void SetPose(OPRPose pose);

    std::vector<std::string> swervePre{"FR", "FL", "BR", "BL"};
    std::vector<OPRSwerveModule*> swerveModules;
    std::vector<OPRModuleVelocity> swerveModulesVelocity;

    OPRPose desiredPoint;

    units::turns_per_second_t xMag = 0_tps;
    units::turns_per_second_t yMag = 0_tps;
    units::turns_per_second_t rMag = 0_tps;

    OPRSlewRateLimiter* xLimiter;
    OPRSlewRateLimiter* yLimiter;
    OPRSlewRateLimiter* rLimiter;

    units::degree_t heading;
    units::degree_t desiredHeading;

    DriveStates currState = IDLE;
    DriveStates lastState = IDLE;

    OPROdometry* Odometry;
    OPRGyro* Gyro;
    OPRCameras* Cameras;
    OPRSensors* Sensors;

    SimPID* TurnPID;
    SimPID* DrivePID;

    std::vector<units::degree_t> lastSwerveHeadings;

    double oldHeading = 0.0;
    double gamePieceYCompFactor = 0.0;
    frc::Timer* holdHeadingTimer;
    frc::Timer* brakeTimer;

    bool resetOffsetValues = false;
    bool brakesEnabled = false;
    
    bool autoAimHeadingSnap =  false;

    static constexpr double TurnP = 0.009;
    static constexpr double TurnI = 0.0;
    static constexpr double TurnD = 0.009;

    static constexpr double DriveP = 0.02;
    static constexpr double DriveI = 0.0;
    static constexpr double DriveD = 0.04;

    static constexpr double ASSISTED_DRIVE_SCALE = 0.03;

    static constexpr double FR_STEER_ENCODER_OFFSET_PRACTICE = 0.116;
    static constexpr double FL_STEER_ENCODER_OFFSET_PRACTICE = 0.327;
    static constexpr double BR_STEER_ENCODER_OFFSET_PRACTICE = 0.384;
    static constexpr double BL_STEER_ENCODER_OFFSET_PRACTICE = -0.437;

    static constexpr double FR_STEER_ENCODER_OFFSET_COMPETITION = 0.59;
    static constexpr double FL_STEER_ENCODER_OFFSET_COMPETITION = 0.7230;
    static constexpr double BR_STEER_ENCODER_OFFSET_COMPETITION = -0.7140;
    static constexpr double BL_STEER_ENCODER_OFFSET_COMPETITION = 0.6160;
};
#include "Subsystems/OPRDriveBase.h"

OPRDriveBase::OPRDriveBase(void) {
    swerveModules.push_back(new OPRSwerveModule(FR_DRIVE_MOTOR_ID, FR_STEER_MOTOR_ID, FR_ABSOLUTE_ENCODER_ID, X_OFFSET_FORWARD, -Y_OFFSET, false, true));
    swerveModules.push_back(new OPRSwerveModule(FL_DRIVE_MOTOR_ID, FL_STEER_MOTOR_ID, FL_ABSOLUTE_ENCODER_ID, X_OFFSET_FORWARD, Y_OFFSET, true, false));
    swerveModules.push_back(new OPRSwerveModule(BR_DRIVE_MOTOR_ID, BR_STEER_MOTOR_ID, BR_ABSOLUTE_ENCODER_ID, -X_OFFSET_BACK, -Y_OFFSET, false, false));
    swerveModules.push_back(new OPRSwerveModule(BL_DRIVE_MOTOR_ID, BL_STEER_MOTOR_ID, BL_ABSOLUTE_ENCODER_ID, -X_OFFSET_BACK, Y_OFFSET, false, false));

    Gyro = OPRGyro::GetInstance();
    Odometry = OPROdometry::GetInstance(swerveModules);
    Cameras = OPRCameras::GetInstance();
    Sensors = OPRSensors::GetInstance();

    DrivePID = new SimPID();
    TurnPID = new SimPID();

    xLimiter = new OPRSlewRateLimiter(600.0, 20000.0);
    yLimiter = new OPRSlewRateLimiter(600.0, 20000.0);
    rLimiter = new OPRSlewRateLimiter(16.0, 240);
    holdHeadingTimer = new frc::Timer();
    brakeTimer = new frc::Timer();
    holdHeadingTimer->Start();

    for (int i = 0; i < 4; i++) {
        swerveModulesVelocity.push_back({0_tps, swerveModules[i]->GetSteerAngle()});
        lastSwerveHeadings.push_back(swerveModules[i]->GetSteerAngle());
    }
}

OPRDriveBase::~OPRDriveBase(void) {}

//==========================================================================================================
//
//  #####  #####     ####        #####  ##   ##  ##     ##   ####  ######  ##   #####   ##     ##   ####
//  ##     ##  ##   ##           ##     ##   ##  ####   ##  ##       ##    ##  ##   ##  ####   ##  ##
//  #####  #####    ##           #####  ##   ##  ##  ## ##  ##       ##    ##  ##   ##  ##  ## ##   ###
//  ##     ##  ##   ##           ##     ##   ##  ##    ###  ##       ##    ##  ##   ##  ##    ###     ##
//  ##     ##   ##   ####        ##      #####   ##     ##   ####    ##    ##   #####   ##     ##  ####
//
//==========================================================================================================

void OPRDriveBase::RobotInit() {
    printf("START: OPRDriveBase::RobotInit()\n");
    TurnPID->setConstants(TurnP, TurnI, TurnD);
    DrivePID->setConstants(DriveP, DriveI, DriveD);

    Gyro->SetAngle(0);

    heading = Gyro->GetAngle();
    oldHeading = 0.0;

    if (USE_COMP_VALUES) {
        swerveModules[0]->SetEncoderOffset(FR_STEER_ENCODER_OFFSET_COMPETITION);
        swerveModules[1]->SetEncoderOffset(FL_STEER_ENCODER_OFFSET_COMPETITION);
        swerveModules[2]->SetEncoderOffset(BR_STEER_ENCODER_OFFSET_COMPETITION);
        swerveModules[3]->SetEncoderOffset(BL_STEER_ENCODER_OFFSET_COMPETITION);
    }
    else {
        swerveModules[0]->SetEncoderOffset(FR_STEER_ENCODER_OFFSET_PRACTICE);
        swerveModules[1]->SetEncoderOffset(FL_STEER_ENCODER_OFFSET_PRACTICE);
        swerveModules[2]->SetEncoderOffset(BR_STEER_ENCODER_OFFSET_PRACTICE);
        swerveModules[3]->SetEncoderOffset(BL_STEER_ENCODER_OFFSET_PRACTICE);
    }
    printf("END: OPRDriveBase::RobotInit()\n");
}

void OPRDriveBase::RobotPeriodic() {
    Odometry->Update();

    if (GamePads->Driver->GetButton(OPRXbox::START) && GamePads->Driver->GetButton(OPRXbox::BACK)) {
        ResetDrive();
    }
}

void OPRDriveBase::DisabledInit() {
    printf("START: OPRDriveBase::DisabledInit()\n");
    holdHeadingTimer->Reset();
    brakeTimer->Reset();
    brakeTimer->Start();
    printf("END: OPRDriveBase::DisabledInit()\n");
}

void OPRDriveBase::DisabledPeriodic() {
    if (resetOffsetValues) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i]->SetEncoderOffset(0);
        }
        resetOffsetValues = false;

        frc::SmartDashboard::PutBoolean("Reset Encoder Offset", resetOffsetValues);
    }
    if (brakeTimer->Get() > 3_s && brakesEnabled == true) {
        printf("~~~~~~~~~~~~~~~~~BrakeTimer~~~~~~~~~~~~~~~~~~\n");
        for (int i = 0; i < 4; i++) {
            swerveModules[i]->BrakeDriveMotor(false);
        }
        brakesEnabled = false;
    }
}

void OPRDriveBase::TeleopInit() {
    printf("START: OPRDriveBase::TeleopInit()\n");
    heading = Gyro->GetAngle();
    oldHeading = Gyro->GetAngle().value();
    currState = FIELD_DRIVE;
    if (brakesEnabled == false) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i]->BrakeDriveMotor(true);
        }
        brakesEnabled = true;
    }
    printf("END: OPRDriveBase::TeleopInit()\n");
}

void OPRDriveBase::TeleopPeriodic() {
    StateMachine();
}

void OPRDriveBase::AutonomousInit() {
    printf("START: OPRDriveBase::AutonomousInit()\n");
    if (brakesEnabled == false) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i]->BrakeDriveMotor(true);
        }
        brakesEnabled = true;
    }
    printf("END: OPRDriveBase::AutonomousInit()\n");
}

void OPRDriveBase::AutonomousPeriodic() {
}

//=========================================================================================================
//
//   ####  ######    ###    ######  #####        ###    ###    ###     ####  ##   ##  ##  ##     ##  #####
//  ##       ##     ## ##     ##    ##           ## #  # ##   ## ##   ##     ##   ##  ##  ####   ##  ##
//   ###     ##    ##   ##    ##    #####        ##  ##  ##  ##   ##  ##     #######  ##  ##  ## ##  #####
//     ##    ##    #######    ##    ##           ##      ##  #######  ##     ##   ##  ##  ##    ###  ##
//  ####     ##    ##   ##    ##    #####        ##      ##  ##   ##   ####  ##   ##  ##  ##     ##  #####
//
//==========================================================================================================

void OPRDriveBase::StateMachine() {
    double xVel = OPRCalculations::SignSquare(GamePads->Driver->GetLeftY()) * MAX_SPEED;
    double yVel = OPRCalculations::SignSquare(GamePads->Driver->GetLeftX()) * MAX_SPEED;

    double rVel = OPRCalculations::SignSquare(GamePads->Driver->GetRightX()) * MAX_ROTATIONAL_SPEED;
    double _desiredHeading;

    switch (currState) {
        case IDLE:
            if (xVel != 0 || yVel != 0 || rVel != 0) {
                currState = FIELD_DRIVE;
            }
            oldHeading = Gyro->GetAngle().value();
            holdHeadingTimer->Reset();
            break;

        case FIELD_DRIVE:
            if (rVel == 0 && holdHeadingTimer->Get() > 350_ms) {
                TurnPID->setDesiredValue(oldHeading);
                rVel = MAX_ROTATIONAL_SPEED * TurnPID->calcPID(Gyro->GetAngle().value());
            }
            else if (rVel != 0) {
                holdHeadingTimer->Reset();
            }
            else {
                oldHeading = Gyro->GetAngle().value();
            }

            FieldCentricDrive(xVel, yVel, rVel);
            break;

        case ROBOT_DRIVE:
            RobotCentricDrive(xVel, yVel, rVel);
            oldHeading = Gyro->GetAngle().value();
            holdHeadingTimer->Reset();
            break;

        default:
            break;
    }

    if (GamePads->Driver->GetButton(D_FIELD_DRIVE)) {
        currState = FIELD_DRIVE;
    }
    else if (GamePads->Driver->GetButton(D_ROBOT_DRIVE)) {
        currState = ROBOT_DRIVE;
    }
}

//=============================================================================================================================
//
//   #####   ######  ##   ##  #####  #####          #####  ##   ##  ##     ##   ####  ######  ##   #####   ##     ##   ####
//  ##   ##    ##    ##   ##  ##     ##  ##         ##     ##   ##  ####   ##  ##       ##    ##  ##   ##  ####   ##  ##
//  ##   ##    ##    #######  #####  #####          #####  ##   ##  ##  ## ##  ##       ##    ##  ##   ##  ##  ## ##   ###
//  ##   ##    ##    ##   ##  ##     ##  ##         ##     ##   ##  ##    ###  ##       ##    ##  ##   ##  ##    ###     ##
//   #####     ##    ##   ##  #####  ##   ##        ##      #####   ##     ##   ####    ##    ##   #####   ##     ##  ####
//
//=============================================================================================================================

void OPRDriveBase::RobotCentricDrive(double xVel, double yVel, double rVel) {
    // calculates magnitude based on x, y components of the vector
    double vMag = OPRCalculations::pyth(xVel, yVel);

    // scales x,y so magnitude does not exceed max
    if (vMag > MAX_SPEED) {
        xVel = (xVel / vMag) * MAX_SPEED;
        yVel = (yVel / vMag) * MAX_SPEED;
    }

    xMag = (units::turns_per_second_t(xVel) / (WHEEL_DIAMETER * std::numbers::pi)) / DRIVE_GEAR_RATIO;
    yMag = (units::turns_per_second_t(yVel) / (WHEEL_DIAMETER * std::numbers::pi)) / DRIVE_GEAR_RATIO;

    for (int i = 0; i < 4; i++) {
        swerveModulesVelocity[i].direction = units::math::atan2(swerveModules[i]->GetPose().GetY(), swerveModules[i]->GetPose().GetX()) + 90_deg;
        rMag = units::turns_per_second_t(rVel) * (OPRCalculations::pyth(swerveModules[i]->GetPose().GetX().value(), swerveModules[i]->GetPose().GetY().value()) * 2 * std::numbers::pi / (WHEEL_DIAMETER * std::numbers::pi) / DRIVE_GEAR_RATIO);
        units::turns_per_second_t TmpXComp = units::math::cos(swerveModulesVelocity[i].direction) * rMag;
        units::turns_per_second_t TmpYComp = units::math::sin(swerveModulesVelocity[i].direction) * rMag;
        swerveModulesVelocity[i].magnitude =
            units::math::sqrt(units::math::pow<2>(xMag + (units::math::cos(swerveModulesVelocity[i].direction) * rMag)) + units::math::pow<2>(yMag + (units::math::sin(swerveModulesVelocity[i].direction) * rMag)));

        swerveModulesVelocity[i].direction = units::math::atan2(yMag + TmpYComp, xMag + TmpXComp);
    }

    units::turns_per_second_t maxMag = 0_tps;

    for (int i = 0; i < 4; i++) {
        if (swerveModulesVelocity[i].magnitude > maxMag) {
            maxMag = swerveModulesVelocity[i].magnitude;
        }
    }

    if (maxMag > MAX_SPEED_TPS) {
        for (int i = 0; i < 4; i++) {
            swerveModulesVelocity[i].magnitude *= MAX_SPEED_TPS / maxMag;
        }
    }

    for (int i = 0; i < 4; i++) {
        if (units::math::fabs(swerveModulesVelocity[i].magnitude) > 0.5_tps) {
            swerveModules[i]->SetVelocity(swerveModulesVelocity[i]);
            lastSwerveHeadings[i] = swerveModulesVelocity[i].direction;
        }
        else {
            swerveModules[i]->SetVelocity({0_tps, lastSwerveHeadings[i]});
        }
    }
}

void OPRDriveBase::FieldCentricDrive(double xVel, double yVel, double rVel, double robotYComp) {
    double currAngle = Gyro->GetAngle().value();

    xVel = xLimiter->Calculate(xVel);
    yVel = yLimiter->Calculate(yVel);
    rVel = rLimiter->Calculate(rVel);

    double robotCentricX = xVel * cos(currAngle * DEG_TO_RAD) + yVel * sin(currAngle * DEG_TO_RAD);
    double robotCentricY = -xVel * sin(currAngle * DEG_TO_RAD) + yVel * cos(currAngle * DEG_TO_RAD);

    // normal compensation
    OPRVector v = OPRVector(units::inch_t(robotCentricX), units::inch_t(robotCentricY));

    double translationScale = 0.5 * v.GetMag().value();
    double rotationScale = 1.0 * rVel;

    OPRVector n = OPRVector(v);
    units::degree_t normalAngle = -90_deg;

    units::inch_t normalMag = units::inch_t(0.65 * (translationScale * rotationScale));

    n.Transform(normalAngle);
    n.SetMag(normalMag);

    OPRVector resultV = OPRVector(v);
    resultV.Add(n);
    resultV.SetMag(v.GetMag());

    if (robotYComp != 0) {
        robotYComp -= robotCentricY;
    }

    if (robotCentricX < 0) {
        robotYComp = 0.0;
    }

    RobotCentricDrive(resultV.GetX().value(), resultV.GetY().value() + robotYComp, rVel);
}

bool OPRDriveBase::DriveToPoint(OPRPose point, double tolerance, double maxVel, double maxRVel, double robotYComp, bool isContinuous) {
    double xVel, yVel, rVel;
    double translationMag;

    DrivePID->setDesiredValue(0.0);
    TurnPID->setDesiredValue(point.GetAngle().value());

    auto difference = point.Subtract(Odometry->GetPose());
    if (isContinuous) {
        translationMag = maxVel;
    }
    else {
        translationMag = -MAX_SPEED * DrivePID->calcPID(difference.GetMag().value());
    }
    translationMag = std::min(translationMag, maxVel);

    xVel = translationMag * units::math::cos(difference.GetAngle());
    yVel = translationMag * units::math::sin(difference.GetAngle());
    rVel = MAX_ROTATIONAL_SPEED * TurnPID->calcPID(Gyro->GetAngle().value());

    if (rVel > 0) {
        rVel = std::min(rVel, maxRVel);
    }
    else {
        rVel = std::max(rVel, -maxRVel);
    }

    FieldCentricDrive(xVel, yVel, rVel, robotYComp);

    return difference.GetMag().value() < tolerance;
}

void OPRDriveBase::ResetDrive() {
    Gyro->SetAngle(0.0);
    Odometry->Reset();
    heading = Gyro->GetAngle();
    holdHeadingTimer->Reset();
}

void OPRDriveBase::ResetOdometry() {
    Odometry->Reset();
    heading = Gyro->GetAngle();
    holdHeadingTimer->Reset();
}

void OPRDriveBase::ShowOnDashboard() {
    frc::SmartDashboard::PutNumber("FRSteerAngle", swerveModules[0]->GetSteerAngle().value());
    frc::SmartDashboard::PutNumber("FLSteerAngle", swerveModules[1]->GetSteerAngle().value());
    frc::SmartDashboard::PutNumber("BRSteerAngle", swerveModules[2]->GetSteerAngle().value());
    frc::SmartDashboard::PutNumber("BLSteerAngle", swerveModules[3]->GetSteerAngle().value());

    frc::SmartDashboard::PutNumber("FRSteerAngle 0-360", units::turn_t(swerveModules[0]->GetSteerAngle360()).value());
    frc::SmartDashboard::PutNumber("FLSteerAngle 0-360", units::turn_t(swerveModules[1]->GetSteerAngle360()).value());
    frc::SmartDashboard::PutNumber("BRSteerAngle 0-360", units::turn_t(swerveModules[2]->GetSteerAngle360()).value());
    frc::SmartDashboard::PutNumber("BLSteerAngle 0-360", units::turn_t(swerveModules[3]->GetSteerAngle360()).value());

    frc::SmartDashboard::PutNumber("FRDrive Pos", swerveModules[0]->GetDrivePosition().value());
    frc::SmartDashboard::PutNumber("FLDrive Pos", swerveModules[1]->GetDrivePosition().value());
    frc::SmartDashboard::PutNumber("BRDrive Pos", swerveModules[2]->GetDrivePosition().value());
    frc::SmartDashboard::PutNumber("BLDrive Pos", swerveModules[3]->GetDrivePosition().value());

    frc::SmartDashboard::PutNumber("FR Drive Temp", swerveModules[0]->GetDriveTemperature());
    frc::SmartDashboard::PutNumber("FL Drive Temp", swerveModules[1]->GetDriveTemperature());
    frc::SmartDashboard::PutNumber("BR Drive Temp", swerveModules[2]->GetDriveTemperature());
    frc::SmartDashboard::PutNumber("BL Drive Temp", swerveModules[3]->GetDriveTemperature());

    frc::SmartDashboard::PutNumber("FR Steer Temp", swerveModules[0]->GetSteerTemperature());
    frc::SmartDashboard::PutNumber("FL Steer Temp", swerveModules[1]->GetSteerTemperature());
    frc::SmartDashboard::PutNumber("BR Steer Temp", swerveModules[2]->GetSteerTemperature());
    frc::SmartDashboard::PutNumber("BL Steer Temp", swerveModules[3]->GetSteerTemperature());

    frc::SmartDashboard::PutBoolean("FRDrive Mode", swerveModules[0]->IsCoast());

    frc::SmartDashboard::PutNumber("Heading", Gyro->GetAngle().value());
    frc::SmartDashboard::PutNumber("Pitch", Gyro->GetPitch().value());
    frc::SmartDashboard::PutNumber("Roll", Gyro->GetRoll().value());
    frc::SmartDashboard::PutNumber("X Accel", Gyro->GetXAccel());
    frc::SmartDashboard::PutNumber("Y Accel", Gyro->GetYAccel());
    frc::SmartDashboard::PutNumber("Z Accel", Gyro->GetZAccel());

    frc::SmartDashboard::PutNumber("Robot X:", Odometry->GetPose().GetX().value());
    frc::SmartDashboard::PutNumber("Robot Y:", Odometry->GetPose().GetY().value());

    frc::SmartDashboard::PutNumber("Robot X Velocity:", Odometry->GetVelocity().GetX().value());
    frc::SmartDashboard::PutNumber("Robot Y Velocity:", Odometry->GetVelocity().GetY().value());
    frc::SmartDashboard::PutNumber("Robot Total Velocity:", Odometry->GetVelocity().GetMag().value());

    frc::SmartDashboard::PutNumber("Driver Left X", GamePads->Driver->GetLeftX());
    frc::SmartDashboard::PutNumber("Driver Left Y", GamePads->Driver->GetLeftY());
    frc::SmartDashboard::PutNumber("Driver Right X", GamePads->Driver->GetRightX());
    frc::SmartDashboard::PutNumber("Operator Right Y", GamePads->Operator->GetRightY());

    frc::SmartDashboard::PutNumber("Driver Left 360", GamePads->Driver->GetLeftPlusMinus180());

    frc::SmartDashboard::PutString("Drive State", GetStateStr());
    frc::SmartDashboard::PutBoolean("DriveBase Braked", brakesEnabled);
}
//======================================================================================================================
//
//   ####    #####  ######  ######  #####  #####     ####     ##  ####  #####  ######  ######  #####  #####     ####
//  ##       ##       ##      ##    ##     ##  ##   ##       ##  ##     ##       ##      ##    ##     ##  ##   ##
//  ##  ###  #####    ##      ##    #####  #####     ###    ##    ###   #####    ##      ##    #####  #####     ###
//  ##   ##  ##       ##      ##    ##     ##  ##      ##  ##       ##  ##       ##      ##    ##     ##  ##      ##
//   ####    #####    ##      ##    #####  ##   ##  ####  ##     ####   #####    ##      ##    #####  ##   ##  ####
//
//======================================================================================================================

void OPRDriveBase::SetState(DriveStates _state) {
    currState = _state;
}

OPRDriveBase::DriveStates OPRDriveBase::GetState() {
    return currState;
}

std::string OPRDriveBase::GetStateStr() {
    switch (currState) {
        case IDLE:
            return "IDLE";
            break;

        case FIELD_DRIVE:
            return "FIELD_DRIVE";
            break;

        case ROBOT_DRIVE:
            return "ROBOT_DRIVE";
            break;

        default:
            return "DRIVE BASE IS REALLY BROKEN!!!!!!!!";
            break;
    }
}

units::degree_t OPRDriveBase::GetHeadingToPoint(OPRPose p) {
    return p.Subtract(Odometry->GetPose()).GetAngle();
}

void OPRDriveBase::SetPose(OPRPose pose) {
    Odometry->SetPose(pose);
}

OPRPose OPRDriveBase::GetPose() {
    return Odometry->GetPose();
}

OPRVector OPRDriveBase::GetRobotVelocity() {
    return Odometry->GetVelocity();
}

void OPRDriveBase::ResetOdometryPos(OPRVector offsetVector) {
    if (Cameras->AprilTag->GetValidTarget() && Cameras->AprilTag->GetDistance() < 219.999) {
        
        double distance = Cameras->AprilTag->GetDistance() + 18.25;  // Add half the robot length to center distance to the center of the robot
        double x = distance * cos((Gyro->GetAngle().value() - Cameras->AprilTag->GetHorizontalOffset()) * DEG_TO_RAD) - (offsetVector.GetX().value());
        double y = distance * sin((Gyro->GetAngle().value() - Cameras->AprilTag->GetHorizontalOffset()) * DEG_TO_RAD) - (offsetVector.GetY().value());
        
        Odometry->SetPose(OPRPose{units::inch_t(x), units::inch_t(y), Gyro->GetAngle()});
    }
}

//===========================================================================
//
//  ##  ##     ##   ####  ######    ###    ##     ##   ####  #####   ####
//  ##  ####   ##  ##       ##     ## ##   ####   ##  ##     ##     ##
//  ##  ##  ## ##   ###     ##    ##   ##  ##  ## ##  ##     #####   ###
//  ##  ##    ###     ##    ##    #######  ##    ###  ##     ##        ##
//  ##  ##     ##  ####     ##    ##   ##  ##     ##   ####  #####  ####
//
//===========================================================================

OPRDriveBase* OPRDriveBase::GetInstance() {
    if (instance == NULL) {
        instance = new OPRDriveBase();
    }
    return instance;
}

OPRDriveBase* OPRDriveBase::instance = NULL;
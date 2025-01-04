#include "Subsystems/OPRSubsystemController.h"

OPRSubsystemController::OPRSubsystemController(void) {
    _DriveBase = OPRDriveBase::GetInstance();
}

OPRSubsystemController::~OPRSubsystemController(void) {}

// FRC Functions
void OPRSubsystemController::RobotInit() {
    _DriveBase->RobotInit();
}

void OPRSubsystemController::RobotPeriodic() {
    _DriveBase->RobotPeriodic();
}

void OPRSubsystemController::DisabledInit() {
    _DriveBase->DisabledInit();
}

void OPRSubsystemController::DisabledPeriodic() {
    _DriveBase->DisabledPeriodic();
}

void OPRSubsystemController::TeleopInit() {
    _DriveBase->TeleopInit();
}

void OPRSubsystemController::TeleopPeriodic() {
    _DriveBase->TeleopPeriodic();
}

void OPRSubsystemController::AutonomousInit() {
    _DriveBase->AutonomousInit();
}

void OPRSubsystemController::AutonomousPeriodic() {
}

void OPRSubsystemController::ShowOnDashboard() {
    _DriveBase->ShowOnDashboard();
}

// instances
OPRSubsystemController* OPRSubsystemController::instance = NULL;

OPRSubsystemController* OPRSubsystemController::GetInstance() {
    if (instance == NULL) {
        instance = new OPRSubsystemController();
    }
    return instance;
}

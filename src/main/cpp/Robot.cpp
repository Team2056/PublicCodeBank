// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
    rioID = frc::RobotController::GetSerialNumber();

    if (rioID == "03223877") {
        USE_COMP_VALUES = false;
    }
    else {
        USE_COMP_VALUES = true;
    }

    _SubsystemController->RobotInit();
    _Leds->InitializeLEDController();

    for (int port = 5800; port <= 5807; port++) {
        wpi::PortForwarder::GetInstance().Add(port, "10.20.56.12", port);
    }
}

void Robot::RobotPeriodic() {
    if (frc::Timer::GetFPGATimestamp() - lastDashboardTime > 150_ms) {
        _SubsystemController->ShowOnDashboard();
        _Cameras->ShowOnDashboard();
        _AutoController->ShowOnDashboard();
        _Lidar->ShowOnDashboard();
        lastDashboardTime = frc::Timer::GetFPGATimestamp();
    }
    _Cameras->RobotPeriodic();
    _SubsystemController->RobotPeriodic();
}

void Robot::AutonomousInit() {
    _SubsystemController->AutonomousInit();
    _AutoController->AutonomousInit();
}

void Robot::AutonomousPeriodic() {
    _AutoController->AutonomousPeriodic();
    _SubsystemController->AutonomousPeriodic();
    _Leds->UpdateLEDAuto();
}

void Robot::TeleopInit() {
    _SubsystemController->TeleopInit();
    _Leds->TeleopInit();
}

void Robot::TeleopPeriodic() {
    _SubsystemController->TeleopPeriodic();
    _Leds->UpdateLEDTeleop();
}

void Robot::DisabledInit() {
    _SubsystemController->DisabledInit();
    _AutoController->DisabledInit();
}

void Robot::DisabledPeriodic() {
    if (_GamePads->Driver->GetButton(OPRXbox::Button::RS)) {
        if (_GamePads->Driver->GetRightY() > 0.5) {
            currAlliance = Alliances::RED;
        }
        else if (_GamePads->Driver->GetRightY() < -0.5) {
            currAlliance = Alliances::BLUE;
        }
    }

    if (frc::DriverStation::IsDSAttached()) {
        ShowOnDashboard();
    }

    _SubsystemController->DisabledPeriodic();
    _AutoController->AutoSelector();
    _Leds->UpdateLEDDisabled();
}

void Robot::ShowOnDashboard() {
    frc::SmartDashboard::PutString("Rio ID", rioID);
    frc::SmartDashboard::PutBoolean("Comp Values", USE_COMP_VALUES);

    if (frc::DriverStation::IsDSAttached()) {
        frc::SmartDashboard::PutNumber("Match Number", frc::DriverStation::GetMatchNumber());
        frc::SmartDashboard::PutNumber("Station Number", frc::DriverStation::GetLocation().value());
        frc::SmartDashboard::PutString("Event Name", frc::DriverStation::GetEventName());
        frc::SmartDashboard::PutNumber("Match Timer:", frc::DriverStation::GetMatchTime().value());
    }

    frc::SmartDashboard::PutBoolean("Alliance Colour", currAlliance == Alliances::RED ? true : false);
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif

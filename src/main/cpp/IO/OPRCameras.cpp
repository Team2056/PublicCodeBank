#include "IO/OPRCameras.h"

OPRCameras* OPRCameras::instance = NULL;

OPRCameras::OPRCameras() {
    GamePiece = new OPRLimeLight("limelight-piece");
    AprilTag = new OPRLimeLight("limelight-april");
}

OPRCameras::~OPRCameras() {
}

OPRCameras* OPRCameras::GetInstance() {
    if (instance == NULL) {
        instance = new OPRCameras();
    }
    return instance;
}

void OPRCameras::ShowOnDashboard() {
    frc::SmartDashboard::PutNumber("April Tag LimeLight TX", AprilTag->GetHorizontalOffset());
    frc::SmartDashboard::PutNumber("April Tag LimeLight TY", AprilTag->GetVerticalOffset());
    frc::SmartDashboard::PutNumber("April Tag LimeLight DISTANCE", AprilTag->GetDistance());
    frc::SmartDashboard::PutBoolean("April Tag LimeLight CONNECTED", AprilTag->IsConnected());
    frc::SmartDashboard::PutBoolean("April Tag LimeLight Valid Target", AprilTag->GetValidTarget());

    frc::SmartDashboard::PutNumber("Game Piece LimeLight TX", GamePiece->GetHorizontalOffset());
    frc::SmartDashboard::PutNumber("Game Piece LimeLight TY", GamePiece->GetVerticalOffset());
    frc::SmartDashboard::PutNumber("Game Piece LimeLight AREA", GamePiece->GetArea());
    frc::SmartDashboard::PutBoolean("Game Piece LimeLight CONNECTED", GamePiece->IsConnected());
    frc::SmartDashboard::PutBoolean("Game Piece LimeLight Valid Target", GamePiece->GetValidTarget());
}

void OPRCameras::RobotPeriodic() {
    AprilTag->RobotPeriodic();
    GamePiece->RobotPeriodic();
}
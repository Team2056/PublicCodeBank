#include "OPRAuto.h"

// include all autos

OPRAuto *OPRAuto::instance = NULL;

OPRAuto::OPRAuto() {
    SelectedAutoMode = 0;
    AutoDelay = 0.0;

    DriveBase = OPRDriveBase::GetInstance();
    Gyro = OPRGyro::GetInstance();
    Cameras = OPRCameras::GetInstance();
    GamePads = OPRControllers::GetInstance();

    AutoTimer = new frc::Timer();
    AutoAwaitTimer = new frc::Timer();
    DelayTimer = new frc::Timer();

    AutoModes = std::vector<AutoMode *>({});
}

OPRAuto::~OPRAuto() {}

OPRAuto *OPRAuto::GetInstance() {
    if (instance == NULL) {
        instance = new OPRAuto();
    }
    return instance;
}

void OPRAuto::SensorReset() {
    DriveBase->ResetDrive();
}

void OPRAuto::StartTimers() {
    DelayTimer->Start();
    DelayTimer->Reset();
    AutoAwaitTimer->Start();
}

void OPRAuto::AutoSelector() {
    if (AutoAwaitTimer->Get() > 3.1_s && !GamePads->Driver->GetButton(OPRXbox::Button::LS) && !GamePads->Driver->GetButton(OPRXbox::Button::RS)) {
        for (int i = 0; i < (int)AutoModes.size(); i++) {
            if (GamePads->Driver->GetButton(static_cast<OPRXbox::Button>(i + 1))) {
                SelectedAutoMode = i;
                if (fabs(GamePads->Driver->GetLeftY()) > 0.01) {
                    AutoDelay = (round(fabs(GamePads->Driver->GetLeftY()) * 10 * 2.0) / 2.0);
                }
            }
        }
    }

    if (GamePads->Driver->GetButton(OPRXbox::Button::LS)) {
        AutoDelay = 0.0;
    }
}

int OPRAuto::GetAutoMode() {
    return SelectedAutoMode;
}

double OPRAuto::GetAutoDelay() {
    return AutoDelay;
}

void OPRAuto::ResetStates() {
    DriveBase->SetState(OPRDriveBase::DriveStates::IDLE);
}

void OPRAuto::DisabledInit() {
    AutoAwaitTimer->Start();
    AutoAwaitTimer->Reset();
}

void OPRAuto::AutonomousInit() {
    SensorReset();
    StartTimers();
    ResetStates();

    AutoModes[SelectedAutoMode]->ResetAuto();
    AutoModes[SelectedAutoMode]->ReflectIfRed();
}

void OPRAuto::AutonomousPeriodic() {
    if (DelayTimer->Get().value() > AutoDelay) {
        AutoModes[SelectedAutoMode]->Run();
    }
}

std::string OPRAuto::GetAutoModeString() {
    return AutoModes[SelectedAutoMode]->GetName();
}

void OPRAuto::ShowOnDashboard() {
    frc::SmartDashboard::PutString("Current Auto", GetAutoModeString());
    frc::SmartDashboard::PutNumber("Delay Timer", GetAutoDelay());
    frc::SmartDashboard::PutNumber("Auto Step", AutoModes[SelectedAutoMode]->GetStep());
}
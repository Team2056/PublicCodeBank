#include "IO/OPRXbox.h"

/**
 * Constructs an instance of the Xbox class.
 *
 * @param port The port on the driver station that the gamepad is plugged into.
 */
OPRXbox::OPRXbox(int port) {
    joystick = new frc::XboxController(port);
    checkJoystick = false;
}

OPRXbox::~OPRXbox() {}

/**
 * Detects if a button/trigger/DPAD on the controller has been pressed
 *
 * @param _button button to check (enum Button)
 * @return true/false if button is pressed
 */
bool OPRXbox::GetButton(Button _button) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (_button == RT) {
            return joystick->GetRightTriggerAxis() >= TRIGGER_DEADBAND;
        }
        else if (_button == LT) {
            return joystick->GetLeftTriggerAxis() >= TRIGGER_DEADBAND;
        }
        else if (_button == DU) {
            return joystick->GetPOV() == 0;
        }
        else if (_button == DR) {
            return joystick->GetPOV() == 90;
        }
        else if (_button == DD) {
            return joystick->GetPOV() == 180;
        }
        else if (_button == DL) {
            return joystick->GetPOV() == 270;
        }

        return joystick->GetRawButton(_button);
    }
    else {
        return false;
    }
}

/**
 * @brief Check if a button has been released
 * 
 * @param _button button to check
 * @return true 
 * @return false 
 */
bool OPRXbox::GetButtonReleased(Button _button) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) { // if the joystick is connected
        return joystick->GetRawButtonReleased(_button);
    }
    else {
        return false;
    }
}

/*
 * Gets the left joystick X axis value between -1 and 1
 *
 * @return Left X axis value between -1 and 1
 */
double OPRXbox::GetLeftX() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetLeftX()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetLeftX();
    }
    else {
        return 0.0;
    }
}

/*
 * Gets the left joystick Y axis value between -1 and 1
 *
 * @return Left Y axis value between -1 and 1
 */
double OPRXbox::GetLeftY() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetLeftY()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetLeftY();
    }
    else {
        return 0.0;
    }
}

/*
 * Gets the right joystick X axis value between -1 and 1
 *
 * @return Right X axis value between -1 and 1
 */
double OPRXbox::GetRightX() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetRightX()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetRightX();
    }
    else {
        return 0.0;
    }
}

/*
 * Gets the Right joystick Y axis value between -1 and 1
 *
 * @return Right Y axis value between -1 and 1
 */
double OPRXbox::GetRightY() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetRightY()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetRightY();
    }
    else {
        return 0.0;
    }
}

/**
 * @brief Return angle in degrees
 * 
 * @return double 
 */
double OPRXbox::GetLeftPlusMinus180() {
    return atan2(GetLeftX(), GetLeftY()) * (180 / std::numbers::pi);
}

/**
 * @param intensity set the intensity of the rumble from 0 - 1
 *
 * @brief Sets the rumble of the controller
 */
void OPRXbox::SetRumble(double intensity) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        joystick->SetRumble(frc::XboxController::RumbleType::kBothRumble, intensity);
    }
}
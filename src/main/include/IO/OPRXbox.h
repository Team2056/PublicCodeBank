#pragma once

#include <frc/XboxController.h>
#include <math.h>

class OPRXbox {
   public:
    enum Button {
        A = 1,
        B,
        X,
        Y,
        LB,     // left bumper
        RB,     // right bumpers
        BACK,   // left face button
        START,  // right face button
        LS,     // left stick in
        RS,     // right stick in

        // Triggers are analog
        LT,  // right trigger
        RT,  // left trigger

        // D-Pad
        DU,  // D-Pad up
        DD,  // D-Pad Down
        DL,  // D-Pad Left
        DR   // D-Pad Right
    };

    OPRXbox(int port);
    ~OPRXbox();

    void SetRumble(double intensity);
    bool GetButton(Button _button);
    bool GetButtonReleased(Button _button);
    double GetLeftX();
    double GetLeftY();
    double GetRightX();
    double GetRightY();
    double GetLeftPlusMinus180();

   private:
    frc::XboxController* joystick;
    const double TRIGGER_DEADBAND = 0.05;
    const double STICK_DEADBAND = 0.06;
    double currRumble = 0.0;

    bool checkJoystick = false;
};
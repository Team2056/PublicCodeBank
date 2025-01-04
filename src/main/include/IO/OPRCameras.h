#pragma once

#include "IO/OPRLimelight.h"

class OPRCameras : public OPRLoggable {
   public:
    static OPRCameras* GetInstance();
    void ShowOnDashboard();
    void RobotPeriodic();

    OPRLimeLight* GamePiece;
    OPRLimeLight* AprilTag;

   private:
    OPRCameras();
    ~OPRCameras();

    static OPRCameras* instance;
};
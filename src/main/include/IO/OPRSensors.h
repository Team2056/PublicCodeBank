#pragma once

#include "OPRIncludes.h"

class OPRSensors {
    public:
        static OPRSensors* GetInstance();

    private:
        OPRSensors(void);
        ~OPRSensors();

        static OPRSensors* instance;
};
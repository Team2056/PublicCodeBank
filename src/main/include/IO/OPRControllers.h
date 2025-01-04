#pragma once

#include <math.h>

#include "./OPRXbox.h"

class OPRControllers {
    public:
        static OPRControllers* GetInstance();

        OPRXbox* Driver;
        OPRXbox* Operator;

    private:
        OPRControllers();
        ~OPRControllers();
        static OPRControllers* instance;  
};
#include "./IO/OPRControllers.h"

OPRControllers *OPRControllers::instance = NULL;

OPRControllers::OPRControllers(void) {
    Driver = new OPRXbox(0);
    Operator = new OPRXbox(1);
}

OPRControllers::~OPRControllers(void) {
}

OPRControllers *OPRControllers::GetInstance() {
    if (instance == NULL) {
        instance = new OPRControllers();
    }

    return instance;
}
#include "IO/OPRSensors.h"

OPRSensors::OPRSensors() {
    instance = NULL;
}

OPRSensors::~OPRSensors() {   
}

OPRSensors* OPRSensors::instance = NULL;

OPRSensors* OPRSensors::GetInstance() {
    if(instance == NULL) {
        instance = new OPRSensors();
    }
    return instance;
}
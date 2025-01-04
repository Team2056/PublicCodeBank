#include "Utils/OPRSlewRateLimiter.h"

OPRSlewRateLimiter::OPRSlewRateLimiter(double _rateLimit, double _jerkLimit) {
    maxRateLimit = _rateLimit;
    jerkLimit = _jerkLimit;
    currRateLimit = 0.0;
    prevVal = 0.0;
    prevTime = 0.0_s;
    currTime = 0.0_s;
}

double OPRSlewRateLimiter::Calculate(double _input) {
    double output = 0.0;
    currTime = frc::Timer::GetFPGATimestamp();

    elapsedTime = currTime - prevTime;

    if (_input > (prevVal + currRateLimit * elapsedTime.value())) {
        output = prevVal + currRateLimit * elapsedTime.value();
        currRateLimit = OPRCalculations::LimitOutput(currRateLimit + jerkLimit * elapsedTime.value(), maxRateLimit);
    }
    else if (_input < (prevVal - currRateLimit * elapsedTime.value())) {
        output = prevVal - currRateLimit * elapsedTime.value();
        currRateLimit = OPRCalculations::LimitOutput(currRateLimit + jerkLimit * elapsedTime.value(), maxRateLimit);
    }
    else {
        output = _input;
        currRateLimit = (prevVal - _input) * elapsedTime.value();
    }

    prevTime = currTime;
    prevVal = output;
    return output;
}

void OPRSlewRateLimiter::Reset(double _input) {
    prevVal = _input;
    prevTime = frc::Timer::GetFPGATimestamp();
}

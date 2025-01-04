#pragma once
#include <OPRCalculations.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <wpimath/MathShared.h>

class OPRSlewRateLimiter {
   public:
    /**
     * @brief Construct a new OPRSlewRateLimiter object (initial value is 0)
     *
     * @param _rateLimit rate of change limit in positive and negative
     */
    explicit OPRSlewRateLimiter(double _rateLimit, double);

    /**
     * @brief Filters input to limit slew rate
     *
     * @param input value to be limited
     * @return filtered value
     */
    double Calculate(double _input);

    /**
     * @brief Resets slew rate limiter to specified value
     *
     * @param _input Value to be reset to
     */
    void Reset(double _input);

   private:
    double maxRateLimit = 0.0;
    double currRateLimit = 0.0;
    double prevRateLimit = 0.0;
    double jerkLimit = 0.0;
    double prevVal = 0.0;
    units::second_t prevTime = 0.0_s;
    units::second_t elapsedTime = 0.0_s;
    units::second_t currTime = 0.0_s;
};
#pragma once
#include <units/angle.h>

class OPRCalculations {
   public:
    /// @brief Will square the input and keep the sign
    /// @param input number to square
    /// @return result
    static double SignSquare(double input) {
        if (input < 0) {
            return (-input * input);
        }
        else {
            return (input * input);
        }
    }

    /// @brief Given angle will return it from -180 to 180
    /// @param input the angle to normalize
    static void NormalizeAngle(units::angle::degree_t &input) {
        while (input < -180_deg) {
            input += 360_deg;
        }
        while (input > 180_deg) {
            input -= 360_deg;
        }
    }

    /// @brief Will limit the inputted value
    /// @param input number to limit
    /// @param LIMIT the limit
    /// @return the value if not above the limit, otherwise the limit
    static double LimitOutput(double input, const double LIMIT) {
        if (input > LIMIT) {  // check if the input is greater the LIMIT
            return LIMIT;
        }
        else if (input < -LIMIT) {  // check if the input is less then the negative LIMIT
            return (-LIMIT);
        }
        else {  // we aren't over the LIMIT
            return input;
        }
    }

    static double pyth(double x, double y) {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    static double pyth(int x, int y) {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    /**
     * @brief Returns the max of 2 numbers
     * @param num1 
     * @param num2 
     * @return double - the biggest number
     */
    static double Max(double num1, double num2) {
        if (num1 > num2) {
            return num1;
        } else {
            return num2;
        }
    }

    /**
     * @brief Retrns the sign of the number
     * @param value - num to check
     * @return double - negitiev or positie one
     */
    static double GetSign(double value) {
        if (value >= 0.0) {
            return 1;
        } else {
            return -1;
        }
    }

};
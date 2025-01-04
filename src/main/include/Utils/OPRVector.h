#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "./OPRCalculations.h"
#include "./OPRConstants.h"

class OPRVector {
   public:
    /// @brief Create a new vector, calculates the angle and magnitude
    /// @param _x the x component
    /// @param _y the y component
    OPRVector(units::inch_t _x, units::inch_t _y) {
        x = _x;
        y = _y;
        angle = units::math::atan2(y, x);
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /// @brief Creates a new vector, will only use angle if x and y are both 0
    /// @param _x the x component
    /// @param _y the y component
    /// @param _angle angle if x&y = 0
    OPRVector(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
        x = _x;
        y = _y;

        if (x == 0_in && y == 0_in) {
            angle = _angle;
        }
        else {
            angle = units::math::atan2(y, x);
        }
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /// @brief creates a new vector with a magnitude and angle, calculates x and y
    /// @param _mag the magnitude of the vector
    /// @param _angle the angle of the vector
    OPRVector(units::inch_t _mag, units::degree_t _angle) {
        mag = _mag;
        angle = _angle;
        OPRCalculations::NormalizeAngle(angle);
        x = mag * units::math::cos(angle);
        y = mag * units::math::sin(angle);
    }

    /// @brief Subtracts the x and y components of 2 vectors and calculates a new angle
    /// @param v the vector to subtract
    void SubtractVectors(OPRVector v) {
        x -= v.GetX();
        y -= v.GetY();
        angle = units::math::atan2(y, x);
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /// @brief add 2 vectors together
    /// @param v the vector to add
    void Add(OPRVector v) {
        x += v.GetX();
        y += v.GetY();
        angle = units::math::atan2(y, x);
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /**
     * Will change the frame of reference from robot to field
     * @param GyroAngle the angle of the gyro as a double, but in degrees
     */
    void Transform(units::degree_t GyroAngle) {
        angle += GyroAngle;
        x = mag * units::math::cos(angle);
        y = mag * units::math::sin(angle);
    }

    /**
     * @return x component in inches
     */
    units::inch_t GetX() {
        return x;
    }

    /**
     * @return y component in inches
     */
    units::inch_t GetY() {
        return y;
    }

    /**
     * @brief Set the y component of the vector
     * 
     * @param _y 
     */
    void SetY(units::inch_t _y) {
        y = _y;
        angle = units::math::atan2(y, x);
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /**
     * @brief Set the x component of the vector
     * 
     * @param _x 
     */
    void SetX(units::inch_t _x) {
        x = _x;
        angle = units::math::atan2(y, x);
        mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
    }

    /**
     * @return angle component in degrees
     */
    units::degree_t GetAngle() {
        return angle;
    }

    /**
     * @return magnitude component in inches
     */
    units::inch_t GetMag() {
        return mag;
    }

    /**
     * set the magnitude
     * @param mag (inches) magnitude to be set to
     */
    void SetMag(units::inch_t _mag) {
        x = _mag * units::math::cos(angle);
        y = _mag * units::math::sin(angle);
        mag = _mag;
    }

    /**
     * @brief Set the Angle of the vector
     * 
     * @param _angle 
     */
    void SetAngle(units::degree_t _angle) {
        x = mag * units::math::cos(_angle);
        y = mag * units::math::sin(_angle);
        angle = _angle;
    }

    /**
     * reflex the vector on y axis
     */
    void ReflectY() {
        x = -x;
        angle = units::math::atan2(units::math::sin(angle), -units::math::cos(angle));
    }

    /// @brief Prints the vector with a pre string
    /// @param pre the string to preface the print
    void Print(const char* pre) {
        printf("%s: x:%f y:%f a:%f\n", pre, x.value(), y.value(), angle.value());
    }

    /// @brief Will print the vector
    void Print() {
        printf("x:%f y:%f a:%f\n", x.value(), y.value(), angle.value());
    }

    OPRVector() {}

   private:
    units::inch_t x = 0_in;
    units::inch_t y = 0_in;
    units::inch_t mag = 0_in;
    units::degree_t angle = 0_deg;
};
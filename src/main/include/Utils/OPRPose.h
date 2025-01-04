#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "./OPRVector.h"

class OPRPose {
   public:
    /// @brief Creates a new pose
    /// @param _x the x component
    /// @param _y the y component
    /// @param _angle the angle of the pose
    OPRPose(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
        x = _x;
        y = _y;
        angle = _angle;
    }

    /// @brief Creates a new pose at 90deg
    OPRPose() {
        x = 0_in;
        y = 0_in;
        angle = 90_deg;
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
     * @return x component in inches
     */
    void SetX(units::inch_t _x) {
        x = _x;
    }

    /**
     * @return y component in inches
     */
    void SetY(units::inch_t _y) {
        y = _y;
    }

    /// @brief Sets the angle of the pos
    /// @param _angle the new angle
    void SetAngle(units::degree_t _angle) {
        angle = _angle;
    }

    /// @brief Reflects the pose along the y axis
    void ReflectY() {
        x = -x;
        angle = units::math::atan2(units::math::sin(angle), -units::math::cos(angle));
    }

    /**
     * @return angle component in degrees
     */
    units::degree_t GetAngle() {
        return angle;
    }

    /**
     * Adds a vector to the pose
     * @param v the vector to add
     */
    void AddVector(OPRVector v) {
        x += v.GetX();
        y += v.GetY();
    }

    /// @brief Subtracts a vector from the pose
    /// @param v vector to substract
    void Subtract(OPRVector v) {
        x -= v.GetX();
        y -= v.GetY();
    }

    /// @brief Subtracts a pose from the pose, and returns a vector
    /// @param p The pose to subtract
    /// @return The resultant vector
    OPRVector Subtract(OPRPose p) {
        // x -= p.GetX();
        // y -= p.GetY();
        return {x - p.GetX(), y - p.GetY()};
    }

    /**
     * Sets the x y and angle components
     * @param _x the new x component
     * @param _y the new y component
     * @param _angle the new anlge component
     */
    void SetPose(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
        x = _x;
        y = _y;
        angle = _angle;
    }

    /**
     * Sets the x y and angle components
     * @param _pose New Pose
     */
    void SetPose(OPRPose _pose) {
        x = _pose.GetX();
        y = _pose.GetY();
        angle = _pose.GetAngle();
    }

    /**
     * Will change the frame of reference from robot to field
     * @param GyroAngle the angle of the gyro as a double, but in degrees
     */
    void Transform(double GyroAngle) {
        // x = x * sin(GyroAngle * DEG_TO_RAD) - y * cos(GyroAngle * DEG_TO_RAD);
        // y = x * cos(GyroAngle * DEG_TO_RAD) + y * sin(GyroAngle * DEG_TO_RAD);
        angle += units::degree_t(GyroAngle);
        OPRCalculations::NormalizeAngle(angle);
    }

    /// @brief Print the pose with a prefix/pre string
    /// @param pre the string to print before pose
    void Print(const char* pre) {
        printf("%s: x:%f y:%f a:%f\n", pre, x.value(), y.value(), angle.value());
    }

    /// @brief Prints the pose with a pre string and number
    /// @param pre the string to print before the pose
    /// @param i the number to print before the pose
    void Print(const char* pre, int i) {
        printf("%s %d: x:%f y:%f a:%f\n", pre, i, x.value(), y.value(), angle.value());
    }

    /// @brief Prints the pose with a number before
    /// @param i the number to print before
    void Print(int i) {
        printf("%d: x:%f y:%f a:%f\n", i, x.value(), y.value(), angle.value());
    }

    /// @brief Prints the pose
    void Print() {
        printf("x:%f y:%f a:%f\n", x.value(), y.value(), angle.value());
    }

   private:
    units::inch_t x;
    units::inch_t y;
    units::degree_t angle;
};
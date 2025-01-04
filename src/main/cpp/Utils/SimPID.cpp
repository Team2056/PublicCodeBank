#include "./utils/SimPID.h"

/**
 * Initializes the SimPID object. All parameters default to 0.
 */

SimPID::SimPID(float p, float i, float d, float epsilon) {
    m_p = p;
    m_i = i;
    m_d = d;

    m_errorEpsilon = epsilon;
    m_desiredValue = 0;  // Default to 0, set later by the user
    m_oldDesiredValue = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_maxOutput = 1.0;  // Default to full range
    m_errorSum = 0;
    m_errorIncrement = 1;

    m_cycleCount = 0;
    m_minCycleCount = 10;  // Default

    m_izone = 0;

    pidTimer = new frc::Timer();
    pidTimer->Start();
    pidTimer->Reset();
}

/**
 * Sets the PID constants to new values.
 */

void SimPID::setConstants(float p, float i, float d) {
    m_p = p;
    m_i = i;
    m_d = d;
}

void SimPID::setIzone(float zone) {
    m_izone = zone;
}

/**
 * Sets the allowable error range away from the desired value.
 */

void SimPID::setErrorEpsilon(float epsilon) {
    m_errorEpsilon = epsilon;
}

/**
 * Sets the maximum increment to the error sum used in the I component
 * calculation.
 * This defaults to 1 in the constructor, which has worked well for 1114 the
 * past few years.
 */

void SimPID::setErrorIncrement(float inc) {
    m_errorIncrement = inc;
}

// Sets Desired Value

void SimPID::setDesiredValue(float val) {
    m_desiredValue = val;
}

// sets the ceiling for the output of the calculation
// This defaults to the 1.0 (full output).  Values shouldl be between 0.0 and 1.0

void SimPID::setMaxOutput(float max) {
    if (max >= 0.0 && max <= 1.0) {
        m_maxOutput = max;
    }
}

// Resets the error sum back to zero

void SimPID::resetErrorSum(void) {
    m_errorSum = 0;
}

/**
 * Calculates the PID output based on the current value.
 * PID constants and desired value should be set before calling this
 * function.
 */

float SimPID::calcPID(float currentValue) {
    // init all components to 0.0 to start
    float pVal = 0.0;
    float iVal = 0.0;
    float dVal = 0.0;

    // Don't apply D the first time through
    if (m_firstCycle) {
        m_previousValue = currentValue;  // Effectively velocity of 0
        m_firstCycle = false;
        pidTimer->Reset();
    }

    if (m_oldDesiredValue != m_desiredValue) {
        m_firstCycle = true;
    }

    // claculate p conponent
    float error = m_desiredValue - currentValue;
    pVal = m_p * (float)error;

    // calculate I component
    // error is positive and outside epsilon band
    if (error >= m_errorEpsilon) {
        if (m_errorSum < 0) {
            // if we are fighting away from the point, reset the error
            // m_errorSum = 0;
        }
        if (error < m_errorIncrement) {
            // If the error is smaller than the max increment amount, add it.
            m_errorSum += error;
        }
        else {
            // Otherwise, add the maximum increment per cycle.
            m_errorSum += m_errorIncrement;
        }
    }
    // error is negative and outside the epsilon band
    else if (error <= -m_errorEpsilon) {
        if (m_errorSum > 0) {
            // If we are fighting away from the point, reset the error.
            // m_errorSum = 0;
        }
        // error is small than max contribution -> just subtract error amount
        if (error > -m_errorIncrement) {
            // iff the error is smaller than the max increment amount add it
            m_errorSum += error;  // error is negative
        }
        else {
            // otherwise, subtract the max increment per cycle
            m_errorSum -= m_errorIncrement;
        }
    }
    // error is inside the epsilon band
    else {
        m_errorSum = 0;
    }

    if (m_izone != 0 && fabs(error) > m_izone) {
        m_errorSum = 0;
    }
    iVal = m_i * (float)m_errorSum;

    // Calculate D component
    double velocity = (currentValue - m_previousValue) / ((double)pidTimer->Get()) * 0.02;
    if (!m_firstCycle) {
        dVal = m_d * (float)velocity;
    }
    else {
        dVal = 0;
    }

    // calculae and limit the output: Output = P + I - D
    float output = pVal + iVal - dVal;
    if (output > m_maxOutput) {
        output = m_maxOutput;
    }
    else if (output < -m_maxOutput) {
        output = -m_maxOutput;
    }

    // Save the current value for the next cycle's D calculation
    m_previousValue = currentValue;
    pidTimer->Reset();
    m_oldDesiredValue = m_desiredValue;
    return output;
}

/**
 * Sets the minimum number of cycles the value must be in the epsilon range
 * before the system is considered stable.
 */

void SimPID::setMinDoneCycles(int n) {
    m_minCycleCount = n;
}

/**
 * Returns true if the last input was within the epsilon range of the
 * destination value, and the system is stable. EDITED
 */

bool SimPID::isDone(void) {
    if (m_previousValue <= m_desiredValue + m_errorEpsilon && m_previousValue >= m_desiredValue - m_errorEpsilon && !m_firstCycle) {
        if (m_cycleCount >= m_minCycleCount) {
            m_cycleCount = 0;
            return true;
        }
        else {
            m_cycleCount++;
        }
    }
    return false;
}
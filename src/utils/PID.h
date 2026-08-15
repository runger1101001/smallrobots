#pragma once

#include <cmath>

namespace SmallRobots {

/**
 * Generic single-axis PID controller.
 *
 * Units are caller-defined. For wheel velocity correction,
 * setpoint and measurement are in rad/s and the output is rad/s.
 *
 * Usage:
 *   PID pid;
 *   pid.P = 0.5f; pid.I = 0.1f; pid.D = 0.0f;
 *   pid.integral_limit = 2.0f;   // anti-windup
 *   pid.output_limit   = 3.0f;   // clamp output
 *
 *   float correction = pid.compute(setpoint, measurement, dt_seconds);
 */
class PID {
public:
    float P             = 0.0f;
    float I             = 0.0f;
    float D             = 0.0f;
    float integral_limit = 10.0f;   // max absolute integral accumulator
    float output_limit  = 10.0f;    // max absolute output (0 = unclamped)

    PID() = default;
    PID(float p, float i, float d, float int_limit = 10.0f, float out_limit = 10.0f)
        : P(p), I(i), D(d), integral_limit(int_limit), output_limit(out_limit) {}

    /** Compute PID correction. dt must be > 0. */
    float compute(float setpoint, float measurement, float dt_seconds) {
        if (dt_seconds <= 0.0f) return 0.0f;

        const float error = setpoint - measurement;

        // Integral with anti-windup clamping
        _integral += error * dt_seconds;
        if (_integral >  integral_limit) _integral =  integral_limit;
        if (_integral < -integral_limit) _integral = -integral_limit;

        // Derivative (on measurement to avoid derivative kick on setpoint changes)
        const float derivative = (measurement - _prev_measurement) / dt_seconds;
        _prev_measurement = measurement;

        float output = P * error + I * _integral - D * derivative;

        // Output clamping
        if (output_limit > 0.0f) {
            if (output >  output_limit) output =  output_limit;
            if (output < -output_limit) output = -output_limit;
        }

        return output;
    }

    /** Reset integral accumulator and derivative state. */
    void reset() {
        _integral         = 0.0f;
        _prev_measurement = 0.0f;
    }

    void setGains(float p, float i, float d) {
        P = p; I = i; D = d;
    }

private:
    float _integral         = 0.0f;
    float _prev_measurement = 0.0f;
};

} // namespace SmallRobots

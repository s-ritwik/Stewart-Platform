/*
 * pid.c
 *
 *  Created on: Nov 24, 2023
 *      Author: admin
 */

#include "pidcontroller.h"
// First-order IIR (Infinite Impulse Response) filter
float LPF(float x, float fc, float dt, float yOld) {
    // Precompute the filter coefficient
    float alpha = (dt * fc) / (1.0f + dt * fc);

    // Update the filtered output using the precomputed coefficient
    return alpha * x + (1.0f - alpha) * yOld;
}
/* Saturation function */
float sat(float value, float max, float min) {
    // Ensure that the value is within the specified range
    return (value >= max) ? max : (value <= min) ? min : value;
}

// Reset the PID controller to its initial state
void Controller_Reset(PIDController *pid) {
    pid->I = 0.0f;
    pid->state_old = 0.0f;
    pid->state_f = 0.0f;
}

// Enable the PID controller
void Controller_Enable(PIDController *pid) {
    pid->enabled = true;
}

// Disable the PID controller
void Controller_Disable(PIDController *pid) {
    pid->enabled = false;
}

// Update the PID controller output based on the current state and setpoint
float Controller_Update(PIDController *pid, float set, float state, Mode ControllerType) {

    if (pid->enabled) {

        float e = set - state;  // Calculate the error between the setpoint and the current state

        // Declare variables for proportional, integral, and derivative terms
        float P, D, U_out, U_sat;

        switch (ControllerType) {
            case P_Cont:
                // Proportional control only
                return sat(pid->kp * e, pid->U_max, pid->U_min);

            case PI_Cont:
                // Proportional-Integral control
                P = pid->kp * e;
                U_out = P + pid->ki * pid->I;
                U_sat = sat(U_out, pid->U_max, pid->U_min);

                // Anti-windup using clamping method: Integrate only if the controller does not saturate and both error and controller output having opposite sign
                if (U_out == U_sat || e * U_out <= 0) {
                    pid->I += e * pid->dt;
                }
                return U_sat;

            case PID_Cont:
                // Proportional-Integral-Derivative control
                P = pid->kp * e;

                // Low pass filter on state
                pid->state_f = LPF(state, pid->fc, pid->dt, pid->state_f);

                // Calculate the derivative term of the state instead of the error to overcome the problem of derivative kick
                D = pid->kd * ((pid->state_f - pid->state_old) / pid->dt);

                // Update state variables
                pid->state_old = pid->state_f;

                // Calculate the output of the controller
                // Note: Taking the derivative term negative because it's the derivative of the state, not the error term
                U_out = P + pid->ki * pid->I - D;

                // Saturate the controller output to stay within the specified bounds
                U_sat = sat(U_out, pid->U_max, pid->U_min);

                // Anti-windup using clamping method: Integrate only if the controller does not saturate and both error and controller output having opposite sign
                if (U_out == U_sat || e * U_out <= 0) {
                    pid->I += e * pid->dt;
                }
                return U_sat;

            default:
                return 0;  // Default to return the saturated output
        }
    }

    return 0;  // Return 0 if the controller is disabled
}









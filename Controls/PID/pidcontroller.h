#ifndef PID_PID_H_ // Header guard to prevent multiple inclusions
#define PID_PID_H_

#include <math.h> // Include math.h header file
#include <stdbool.h>

/* Structure to initialize Control variables */
typedef struct {
    /* CONTROLLER GAINS */
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain

    /* CONTROLLER OUTPUT LIMIT */
    float U_max; // Maximum output limit
    float U_min; // Minimum output limit

    /* CONTROLLER PARAMETERS */
    float dt;        // Sampling time
    float I;         // Integrator memory
    float state_old; // Old value of state
    float state_f;   // Filter output of actual data
    float fc;        // Gain for low pass filter
    bool enabled;     // Flag to indicate if the controller is enabled
} PIDController;

// Enum to define controller types
typedef enum {
	 P_Cont,     // Proportional control only
	PI_Cont,    // Proportional-Integral control
	PID_Cont    // Proportional-Integral-Derivative control
} Mode;

/* Saturation function */
float sat(float value, float max, float min);

/* PID Controller Function */
float Controller_Update(PIDController *pid, float set, float state,  Mode ControllerType);

void Controller_Reset(PIDController *pid);
void Controller_Enable(PIDController *pid);
void Controller_Disable(PIDController *pid);
float LPF(float x, float fc, float dt, float yOld);

#endif /* PID_PID_H_ */

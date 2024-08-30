//#include <pidcontroller.h>

// Basic demo for accelerometer readings from Adafruit MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "CytronMotorDriver.h"
#include <stdbool.h>


MPU6050 accelgyro(0x68); // <-- use for AD0 high
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
////////////////////////////////////////////////////////////////////////DEFINE//////////////////////////////////////////////////////////////////////////////////////////////


#define beta 33.56
#define phi (90 - beta)
#define rs 200
#define alpha 0.85
#define DEG_TO_RAD M_PI/180


////////////////////////////////////////////////////////////////////////STRUCTS//////////////////////////////////////////////////////////////////////////////////////////////
CytronMD aft(PWM_DIR, 13,12); 
CytronMD forward(PWM_DIR, 11,10); 
CytronMD lateral(PWM_DIR, 6,5); 

int16_t ax, ay, az,gx, gy, gz;


// Enum to define controller types
typedef enum {
   P_Cont,     // Proportional control only
  PI_Cont,    // Proportional-Integral control
  PID_Cont    // Proportional-Integral-Derivative control
} Mode;


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

typedef struct data{
    float x;
    float y;
    float z;
    float bx;
    float by;
    float bz;
  }vec;

typedef struct actu{
    float lateral;
    float aft;
    float forward;
    
  }actuator;

typedef struct s{
  vec accel;
  vec gyro;
  float roll;
  float pitch;
  float roll_acc;
  float pitch_acc;
  float roll_bias=0;
  float pitch_bias=0;
}sensor;

float roll_req=00,pitch_req=0;
 
////////////////////////////////////////////////////////////////////////GLOBAL VARIABLES//////////////////////////////////////////////////////////////////////////////////////////////
float dt=0.01;
double MPUOffsets[6] = { -2752.0/16384, -4504.0/16384, 1615.0/16384, 91, 11, 6 };
sensor imu;
int c=0;
actuator current,required;
  float kd=01,kp=25,ki=0.3;
  PIDController ControlLateral = {kp,ki,kd,255,-255,dt,0,0,0,80,false};
  PIDController ControlAft = {kp,ki,kd,255,-255,dt,0,0,0,80,false};
  PIDController ControlForward = {kp,ki,kd,255,-255,dt,0,0,0,80,false};
////////////////////////////////////////////////////////////////////////FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////

// First-order IIR (Infinite Impulse Response) filter
float LPF(float x, float fc, float dt, float yOld) {
    // Precompute the filter coefficient
    float alph = (dt * fc) / (1.0f + dt * fc);

    // Update the filtered output using the precomputed coefficient
    return alph * x + (1.0f - alph) * yOld;
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



void gyro_calibration(){
  for(uint16_t i=0; i<500; i++){
    mpu.getEvent(&a, &g, &temp);
    imu.gyro.bx += g.gyro.x;
    imu.gyro.by += g.gyro.y;
    imu.gyro.bz += g.gyro.z;
    delay(10);
  }
  imu.gyro.bx = imu.gyro.bx/500;
  imu.gyro.by = imu.gyro.by/500;
  imu.gyro.bz = imu.gyro.bz/500;
  imu.pitch=0;
  imu.roll=0;
}



void actuator_movements_curr()
{
  current.aft=-1*rs*sinf(DEG_TO_RAD*(imu.roll));
  current.forward=rs*sinf(DEG_TO_RAD*(imu.roll));
  current.lateral=rs*sinf(DEG_TO_RAD*(imu.pitch));
}
void actuator_movements_req()
{
  required.aft=-1*rs*sinf(DEG_TO_RAD*roll_req);
  required.forward=rs*sinf(DEG_TO_RAD*roll_req);
  required.lateral=rs*sinf(DEG_TO_RAD*pitch_req);
}

// ... (existing code)

////////////////////////////////////////////////////////////////////////PID CONTROLLER//////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////PID CONTROLLER//////////////////////////////////////////////////////////////////////////////////////////////

// ... (existing code)




////////////////////////////////////////////////////////////////////////MAIN//////////////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
  // put your setup code here, to run once:
  int c=0;
 

  Controller_Enable(&ControlLateral);
  Controller_Enable(&ControlAft);
  Controller_Enable(&ControlForward);
  
  Serial.begin(115200);
  Serial.println("Adafruit MPU6050 test!");
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  aft.setSpeed(-255);
  forward.setSpeed(-255);
  lateral.setSpeed(-255);
  delay(5000);
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("");
  gyro_calibration();
  double MPUOffsets[6] = { -2776.00, -4439.00  ,1605.00, 91, 11, 6 };
//-2681.00  -4437.00  1592.00
  accelgyro.setXAccelOffset(MPUOffsets[0]);
  accelgyro.setYAccelOffset(MPUOffsets[1]);
  accelgyro.setZAccelOffset(MPUOffsets[2]);
  float rollbias=0,pitchbias=0;
  //

  
  delay(100);
  Serial.println("starting");
  aft.setSpeed(255);
  forward.setSpeed(255);
  lateral.setSpeed(255);
  delay(1400);
  aft.setSpeed(0);
  forward.setSpeed(0);
  lateral.setSpeed(0);
}
#define SIN_FREQ 0.3 // Frequency of the sinusoidal motion in Hz
#define AMPLITUDE 10 // Amplitude of the sinusoidal motion in degrees

// Function to generate sinusoidal desired roll and pitch values
void generateSinusoidalDesiredValues(float &roll, float &pitch) {
  // Calculate the current time in seconds
  float t = millis() / 1000.0;

  // Calculate the desired roll and pitch values using sine function
  // Adjust the frequency and amplitude as needed
  roll = AMPLITUDE * sin(2 * PI * SIN_FREQ * t);
  pitch = AMPLITUDE * sin(2 * PI * SIN_FREQ * t + PI / 2); // 90 degrees phase shift for pitch
}
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long startTime = micros();
  /////////////////////////////////roll pitch/////////////////////////////////////////////////////////////////

  mpu.getEvent(&a, &g, &temp);

  //standard
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu.accel.x = static_cast<double>(ax) / 16384.0*9.6;
  imu.accel.y = static_cast<double>(ay) / 16384.0*10.1;
  imu.accel.z = static_cast<double>(az) / 16384.0*9.7;
  
  /////////////////////// GYROOOOOOO/////////////////////////////////////
  imu.gyro.x=(g.gyro.x-imu.gyro.bx/200)* 180 / PI;
  imu.gyro.y=(g.gyro.y-imu.gyro.by/200)* 180 / PI;
  imu.gyro.z=(g.gyro.z-imu.gyro.bz/200)* 180 / PI;
  
  imu.pitch_acc =  (atan2((imu.accel.y),sqrt(pow((imu.accel.x), 2) + pow((imu.accel.z), 2))) * 180 / PI);
  imu.roll_acc =  (atan2(-1 * (imu.accel.x) ,sqrt(pow((imu.accel.y), 2) + pow((imu.accel.z), 2))) * 180 / PI);


  imu.roll= alpha*(imu.roll+imu.gyro.x*dt)+(1-alpha)*(imu.roll_acc);
  imu.pitch= alpha*(imu.pitch+imu.gyro.y*dt)+(1-alpha)*(imu.pitch_acc);
  if(c<200)
  {
    c++;
    imu.pitch_bias+=imu.pitch;
    imu.roll_bias+=imu.roll;
    imu.gyro.bx+=g.gyro.x;
    imu.gyro.by+=g.gyro.y;
    imu.gyro.bz+=g.gyro.z;
  }

  else
  {
//    Serial.print("ROLL: ACC:");
//    Serial.println(imu.roll_acc-imu.roll_bias/200);
//    Serial.print(", PITCH: ACC:");
//    Serial.println(imu.pitch_acc-imu.pitch_bias/200);
//    generateSinusoidalDesiredValues(pitch_req,roll_req);
    imu.roll_acc=imu.roll_acc-imu.roll_bias/200;
    imu.pitch_acc=imu.pitch_acc-imu.pitch_bias/200;
    actuator_movements_curr();
    actuator_movements_req();

  aft.setSpeed(-1*Controller_Update(&ControlAft,required.aft,current.aft,PID_Cont));
  forward.setSpeed(-1*Controller_Update(&ControlForward,required.forward,current.forward,PID_Cont));
  lateral.setSpeed(-1*Controller_Update(&ControlLateral,required.lateral,current.lateral,PID_Cont));


  Serial.print(imu.roll);
//   Serial.print("   , pitch : ");
  Serial.print(",");

  Serial.println(imu.pitch);
  }
    



  
//  actuator_movements_required();
  delay(4);

  unsigned long endTime = micros();
  float elapsedTime = float(endTime - startTime);
  dt= (elapsedTime)/1000000.0f;
//  Serial.println(dt,6);
}

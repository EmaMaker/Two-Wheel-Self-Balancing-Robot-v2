#include <QuickPID.h>

#define MOT_DX_STEP 19
#define MOT_DX_DIR 18
#define MOT_SX_STEP 17
#define MOT_SX_DIR 16

// Nema 17 make 1.8° per step. Using A4988 drivers, and 1/16th microstepping, it results in 0.1125° per step
constexpr double ANGLE_PER_STEP = 0.1125;
// Just used a kitchen scale, good enough
constexpr double WEIGHT = 0.961;
constexpr double WHEEL_RADIUS = 0.0475;
// Experimentally, the lowest pulse my steppers can handle without stalling + some leeway
constexpr double MAX_HALF_PERIOD = 75; // in microseconds
// Which means there is a maximum velocity achievable
constexpr double MAX_VELOCITY = 1000000 * WHEEL_RADIUS / (2 * MAX_HALF_PERIOD * ANGLE_PER_STEP) * PI / 180;

// Derived and analytical model, linearized it and simulated in MATLAB.
// PID values are then calculated and verified by simulation in Simulink. I ain't calibrating a PID by hand on this robot
// I modified the ArduPID library to make it accept negative values for the parameters
constexpr double KP = 42;
constexpr double KI = KP * 10.8852;
constexpr double KD = 0.3; 

// PI constants for outer velocity control loop
constexpr double KP_vel = 0.01;
constexpr double KI_vel = 0.005;

float angle_setpoint = 0.0;
float angle_output = 0;
float angle_input = 0;

float vel_setpoint = 0.0;
float vel_input = 0.0;


float yaw{ 0 }, pitch{ 0 }, roll{ 0 };



QuickPID pitchCtrl(&angle_input, &angle_output, &angle_setpoint, KP, KI, KD, pitchCtrl.pMode::pOnError, pitchCtrl.dMode::dOnMeas, pitchCtrl.iAwMode::iAwCondition, pitchCtrl.Action::reverse);
// TODO: a little deadzone when the input is almost 0, just to avoid unnecessary "nervous" control and eventual oscillations
QuickPID velCtrl(&angle_output, &angle_setpoint, &vel_setpoint, KP_vel, KI_vel, 0, velCtrl.pMode::pOnMeas, velCtrl.dMode::dOnMeas, velCtrl.iAwMode::iAwCondition, velCtrl.Action::direct);

void setup() {
  Serial.begin(9600);

  delay(1000);

  setup_imu();
  
  // Just to signal it is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Let the initial error from madgwick filter discharge without affecting the integral term of the PID
  unsigned long t = millis();
  while (millis() - t < 2000) {
    update_imu();
  }

  // Backward because all coefficients need to be negative
  pitchCtrl.SetOutputLimits(-MAX_VELOCITY, MAX_VELOCITY);
  pitchCtrl.SetMode(pitchCtrl.Control::automatic);
  pitchCtrl.SetSampleTimeUs(1000);

  velCtrl.SetOutputLimits(-0.35, 0.35);
  velCtrl.SetMode(velCtrl.Control::automatic);
  velCtrl.SetSampleTimeUs(10000);

  digitalWrite(LED_BUILTIN, LOW);
}

void setup1(){
  pinMode(MOT_DX_DIR, OUTPUT);
  pinMode(MOT_SX_DIR, OUTPUT); 

  pinMode(MOT_DX_STEP, OUTPUT);
  pinMode(MOT_SX_STEP, OUTPUT);
}

unsigned long last_time_motors = micros(), current_time_motors = micros();
bool b = true;
// Such a long halfperiod means that the motors are not moving
uint32_t t = INT_MAX;
int32_t period = INT_MAX, halfperiod1 = INT_MAX;

void loop1(){
  // TODO: handle the steppers using interrupt timers. The second core could be used for IMU processing, while the first one handles the different control loops

  // Retrieve the half period value from core0. Non blocking call
  if(rp2040.fifo.available()) {
    rp2040.fifo.pop_nb(&t);
    
    int32_t period = (int32_t)t;

    // Direction need to be changed during motor pulse
    // Doing it here unsure it happens at the correct time
    if(period > 0){
      // Positivie direction
      digitalWriteFast(MOT_DX_DIR, HIGH);
      digitalWriteFast(MOT_SX_DIR, HIGH);
      halfperiod1 = (uint32_t)(period);
    }else{
      // Negative direction
      digitalWriteFast(MOT_DX_DIR, LOW);
      digitalWriteFast(MOT_SX_DIR, LOW);
      halfperiod1 = (uint32_t)(-period);
    }
  }

  current_time_motors = micros();
  if(current_time_motors - last_time_motors > halfperiod1){
    // Half a pulse. Next cycle will be the rest
    digitalWriteFast(MOT_DX_STEP, b);
    digitalWriteFast(MOT_SX_STEP, b);
    b = !b;
    last_time_motors = current_time_motors;
  }
}

double frequency = 0;
int32_t half_period0 = 0;
double velocity = 0;
    
uint32_t current_time = millis(), last_time = millis();

void loop() {
  update_imu();
  
  velCtrl.Compute();

  angle_input = pitch;
  if(pitchCtrl.Compute()){
    double tvelocity = angle_output;
    
    tvelocity = tvelocity / WHEEL_RADIUS * 180 / PI;
    frequency = tvelocity *  0.1125;
    half_period0 = 1000000 / (2*frequency);

    // Send the half period to core 1. Non blocking
    rp2040.fifo.push_nb(half_period0);
  }

}
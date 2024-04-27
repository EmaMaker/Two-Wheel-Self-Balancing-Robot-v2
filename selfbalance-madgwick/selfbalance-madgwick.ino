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
constexpr double MAX_HALF_PERIOD = 75;  // in microseconds
// Which means there is a maximum velocity achievable
constexpr double MAX_VELOCITY = 1000000 * WHEEL_RADIUS / (2 * MAX_HALF_PERIOD * ANGLE_PER_STEP) * PI / 180;

constexpr double CONVERSION_FACTOR =  180 / WHEEL_RADIUS / PI;

// Derived and analytical model, linearized it and simulated in MATLAB.
// PID values are then calculated and verified by simulation in Simulink. I ain't calibrating a PID by hand on this robot
// I modified the ArduPID library to make it accept negative values for the parameters
constexpr float KP = 42;
constexpr float KI = KP * 10.8852;
constexpr float KD = 0.3;

// PI constants for outer velocity control loop
constexpr float KP_vel = 0.0065;
constexpr float KI_vel = 0.0005;

constexpr float KP_yaw = 0.8;
constexpr float KI_yaw = 0.06;

float angle_setpoint = 0.0;
float angle_output = 0;
float angle_input = 0;

float yaw_setpoint = 0.0;
float yaw_output = 0;
float yaw_input = 0;

float vel_setpoint = 0.0;
float vel_input = 0.0;



float yaw{ 0 }, pitch{ 0 }, roll{ 0 };



QuickPID pitchCtrl(&pitch, &angle_output, &angle_setpoint, KP, KI, KD, pitchCtrl.pMode::pOnError, pitchCtrl.dMode::dOnMeas, pitchCtrl.iAwMode::iAwCondition, pitchCtrl.Action::reverse);
// TODO: a little deadzone when the input is almost 0, just to avoid unnecessary "nervous" control and eventual oscillations
QuickPID velCtrl(&angle_output, &angle_setpoint, &vel_setpoint, KP_vel, KI_vel, 0, velCtrl.pMode::pOnError, velCtrl.dMode::dOnMeas, velCtrl.iAwMode::iAwClamp, velCtrl.Action::direct);

QuickPID yawCtrl(&yaw, &yaw_output, &yaw_setpoint, KP_yaw, KI_yaw, 0, yawCtrl.pMode::pOnError, yawCtrl.dMode::dOnMeas, yawCtrl.iAwMode::iAwClamp, yawCtrl.Action::direct);


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

  yawCtrl.SetOutputLimits(-MAX_VELOCITY*0.01, MAX_VELOCITY*0.01);
  yawCtrl.SetMode(yawCtrl.Control::automatic);
  yawCtrl.SetSampleTimeUs(10000);

  digitalWrite(LED_BUILTIN, LOW);

  yaw_setpoint = yaw;
}

void setup1() {
  pinMode(MOT_DX_DIR, OUTPUT);
  pinMode(MOT_SX_DIR, OUTPUT);

  pinMode(MOT_DX_STEP, OUTPUT);
  pinMode(MOT_SX_STEP, OUTPUT);
}

void loop1() {
  static bool sx_b = true, dx_b = true;
  static uint32_t last_time_sx = micros(), last_time_dx = micros(), current_time_motors = micros();
  // TODO: handle the steppers using interrupt timers. The second core could be used for IMU processing, while the first one handles the different control loops
  static int32_t tmotsx_period = INT_MAX, tmotdx_period = INT_MAX;
  static uint32_t motsx_period = INT_MAX, motdx_period = INT_MAX;
  // Retrieve the half period value from core0. Non blocking call

  if (rp2040.fifo.available() > 2) {
    uint32_t t;
    rp2040.fifo.pop_nb(&t);
    tmotsx_period = (int32_t) t;
    rp2040.fifo.pop_nb(&t);
    tmotdx_period = (int32_t) t;
  }

  // Direction need to be changed during motor pulse
  // Doing it here unsure it happens at the correct time
  if (tmotsx_period > 0) {
    digitalWriteFast(MOT_SX_DIR, HIGH);
    motsx_period = (uint32_t)(tmotsx_period);
  } else {
    // Negative direction
    digitalWriteFast(MOT_SX_DIR, LOW);
    motsx_period = (uint32_t)(-tmotsx_period);
  }
  if (tmotdx_period > 0) {
    digitalWriteFast(MOT_DX_DIR, HIGH);
    motdx_period = (uint32_t)(tmotdx_period);
  } else {
    // Negative direction
    digitalWriteFast(MOT_DX_DIR, LOW);
    motdx_period = (uint32_t)(-tmotdx_period);
  }

  current_time_motors = micros();
  if (current_time_motors - last_time_sx > motsx_period) {
    // Half a pulse. Next cycle will be the rest
    digitalWriteFast(MOT_SX_STEP, sx_b);
    sx_b = !sx_b;
    last_time_sx = current_time_motors;
  }
  if (current_time_motors - last_time_dx > motdx_period) {
    // Half a pulse. Next cycle will be the rest
    digitalWriteFast(MOT_DX_STEP, dx_b);
    dx_b = !dx_b;
    last_time_dx = current_time_motors;
  }
}

double frequency = 0;
int32_t half_period0 = 0;
double velocity = 0;

uint32_t current_time = millis(), last_time = millis();

void loop() {
  update_imu();

  velCtrl.Compute();
  yawCtrl.Compute();

  if (pitchCtrl.Compute()) {
    double tvelocity = angle_output;

    frequency = (tvelocity + yaw_output) * CONVERSION_FACTOR * ANGLE_PER_STEP ;
    half_period0 = 1000000 / (2 * frequency);
    // Send the half period of one motor to core 1. Non blocking
    rp2040.fifo.push_nb(half_period0);

    frequency = (tvelocity - yaw_output) * CONVERSION_FACTOR * ANGLE_PER_STEP ;
    half_period0 = 1000000 / (2 * frequency);
    // Send the half period of the other motor to core 1. Non blocking
    rp2040.fifo.push_nb(half_period0);
  }
}
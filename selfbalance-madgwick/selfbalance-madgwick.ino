#include <ArduPID.h>

ArduPID pitchCtrl;

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
constexpr double KP = -50;
constexpr double KI = -600;
constexpr double KD = 0.05; 

// IMU little bit tilted
// TODO: Implement an outer control loop for angular velocity. But that requires encoders on the motors
// TODO: try to achieve it crudely by just using a PI controller on the velocity given by the PID balance controller
double setpoint = -0.06;
double output = 0;
double input = 0;

double yaw{ 0 }, pitch{ 0 }, roll{ 0 };

void setup() {
  Serial.begin(9600);

  delay(1000);

  setup_imu();
  
  pinMode(MOT_DX_DIR, OUTPUT);
  pinMode(MOT_SX_DIR, OUTPUT); 

  // Just to signal it is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Let the initial error from madgwick filter discharge without affecting the integral term of the PID
  unsigned long t = millis();
  while (millis() - t < 2000) {
    update_imu();
  }

  pitchCtrl.begin(&input, &output, &setpoint, KP, KI, KD, P_ON_E, FORWARD);
  pitchCtrl.setOutputLimits(-MAX_VELOCITY, MAX_VELOCITY);  // double of max torque motors can exhert
  //pitchCtrl.setWindUpLimits(-10, 10);
  pitchCtrl.setSampleTime(1);
  pitchCtrl.start();
}

void setup1(){
  pinMode(MOT_DX_STEP, OUTPUT);
  pinMode(MOT_SX_STEP, OUTPUT);
}

unsigned long last_time_motors = micros(), current_time_motors = micros();
bool b = true;
// Such a long halfperiod means that the motors are not moving
uint32_t halfperiod1 = INT_MAX;
void loop1(){
  // TODO: handle the steppers using interrupt timers. The second core could be used for IMU processing, while the first one handles the different control loops

  // Retrieve the half period value from core0. Non blocking call
  if(rp2040.fifo.available()) rp2040.fifo.pop_nb(&halfperiod1);

  current_time_motors = micros();

  if(current_time_motors - last_time_motors > halfperiod1){
    // Half a pulse. Next cycle will be the rest
    digitalWriteFast(MOT_DX_STEP, b);
    digitalWriteFast(MOT_SX_STEP, b);
    b = !b;
    last_time_motors = current_time_motors;
  }
}

unsigned long last_time = millis(), current_time = millis(), time_diff;
double frequency = 0;
unsigned long half_period0 = 0;
double velocity = 0;

void loop() {
  current_time = millis();
  time_diff = current_time - last_time;

  update_imu();
  // I also modified the ArduPID library to use compute as a boolean. If calculations were done, it returns true. If not enough time has elapsed, it returns false
  if(pitchCtrl.compute()){
    input = pitch;

    // Keeping it here
    /*
    double force_per_motor = output / WHEEL_RADIUS;
    double accel = force_per_motor / (WEIGHT);
    velocity += accel * time_diff * 0.001;
    // anti-windup
    velocity = constrain(velocity, -MAX_VELOCITY, MAX_VELOCITY);*/

    double tvelocity = output;
    if(tvelocity < 0){
      digitalWriteFast(MOT_DX_DIR, LOW);
      digitalWriteFast(MOT_SX_DIR, LOW);
      tvelocity = -tvelocity;
    }else{
      digitalWriteFast(MOT_DX_DIR, HIGH);
      digitalWriteFast(MOT_SX_DIR, HIGH);
    }

    tvelocity = tvelocity * 180 / PI;
    frequency = tvelocity *  0.1125 / WHEEL_RADIUS;
    half_period0 = 1000000 / (2*frequency);

    // Send the half period to core 1. Non blocking
    rp2040.fifo.push_nb(half_period0);

    // Some ugly logging I used in debugging, keeping in around
    /*Serial.println(input);
    Serial.print(" | ");
    Serial.print(output);
    Serial.print(" | ");
    Serial.print(accel);
    Serial.print(" | ");
    Serial.print(velocity);
    Serial.print(" | ");
    Serial.print(frequency);
    Serial.print(" | ");
    Serial.println(half_period0);*/

    last_time = current_time;
  }

}
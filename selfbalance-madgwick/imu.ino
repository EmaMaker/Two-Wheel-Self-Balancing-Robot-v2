#include "FastIMU.h"
#include "Madgwick.h"
#include <Wire.h>

//#define PRINT_ANGLES
//#define PRINT_QUAT
// #define PERFORM_CALIBRATION //Comment to disable startup calibration

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
MPU6050 IMU;               //Change to the name of any supported IMU!

calData calib = {0};
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
Madgwick filter;

void setup_imu(){
  // TODO: write my own kalman filter
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  delay(1500);
  IMU.calibrateAccelGyro(&calib);
  delay(1500);
  IMU.init(calib, IMU_ADDRESS);
#endif
  filter.begin(0.2f);
  Serial.println("IMU up");
}


void update_imu(){
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);
  // MPU6050 has no magnetometer
  filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);

  float qw = filter.getQuatW();
  float qx = filter.getQuatX();
  float qy = filter.getQuatY();
  float qz = filter.getQuatZ();

  double heading{0}, bank{0}, attitude{0};

  double sqw = qw*qw;
  double sqx = qx*qx;
  double sqy = qy*qy;
  double sqz = qz*qz;
	double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	double test = qx*qy + qz*qw;
	if (test > 0.499*unit) { // singularity at north pole
		heading = 2 * atan2(qx,qw);
		attitude = M_PI/2;
		bank = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		heading = -2 * atan2(qx,qw);
		attitude = M_PI/2;
		bank = 0;
		return;
	}
  heading = atan2(2*qy*qw-2*qx*qz , sqx - sqy - sqz + sqw);
	attitude = asin(2*test/unit);
	bank = atan2(2*qx*qw-2*qy*qz , -sqx + sqy - sqz + sqw);

  pitch = heading;
  roll = bank;
  yaw = attitude;


#ifdef PRINT_ANGLES
  Serial.print("Roll: ");
  Serial.print(roll, 5);
  Serial.print("\tPitch: ");
  Serial.print(pitch, 5);
  Serial.print("\tYaw: ");
  Serial.println(yaw, 5);
#endif // PRINT_ANGLES

#ifdef PRINT_QUAT
  Serial.print("QW: ");
  Serial.print(qw, 5);
  Serial.print("\tQX: ");
  Serial.print(qx, 5);
  Serial.print("\tQY: ");
  Serial.print(qy, 5);
  Serial.print("\tQZ: ");
  Serial.println(qz, 5);
#endif // PRINT_QUAT

}

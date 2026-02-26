#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// Gyro bias
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

bool calibrated = false;
bool orientationReady = false;

unsigned long startCalTime;
unsigned long lastTime;
unsigned long stableStart = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Start I2C
  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);

  while (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU not detected...");
    delay(500);
  }

  // Calibration flags
  Serial.println("Keep board completely still...");
  Serial.println("Calibrating gyro for 5 seconds...");

  startCalTime = millis();

  filter.begin(100);      // 100 Hz
  filter.setBeta(0.05);   // Smooth correction
}

void loop() {

  if (!imu.dataReady()) return;

  imu.getAGMT();

  // ========================
  // GYRO CALIBRATION
  // ========================
  if (!calibrated) {

    static int sampleCount = 0;

    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();
    sampleCount++;

    if (millis() - startCalTime >= 5000) {

      // Take average bias
      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;

      calibrated = true;
      lastTime = micros();

      Serial.println("Gyro calibration complete.");
      Serial.println("Waiting for orientation lock...");
    }

    return;
  }

  // ========================
  // NORMAL OPERATION
  // ========================

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Convert mg → m/s²
  float ax = imu.accX() * 0.00981;
  float ay = imu.accY() * 0.00981;
  float az = imu.accZ() * 0.00981;

  // Bias-correct gyro (deg/s → rad/s)
  float gx = (imu.gyrX() - gyroBiasX) * PI / 180.0;
  float gy = (imu.gyrY() - gyroBiasY) * PI / 180.0;
  float gz = (imu.gyrZ() - gyroBiasZ) * PI / 180.0;

  // Update Madgwick
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // Get quaternion
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  // Rotate accel to world frame (Z axis)
  float az_world =
    (2*qx*qz - 2*qw*qy) * ax +
    (2*qy*qz + 2*qw*qx) * ay +
    (1 - 2*qx*qx - 2*qy*qy) * az;

  // ========================
  // MADGWICK CONVERGENCE CHECK
  // ========================
  if (!orientationReady) {

    // Check gravity alignment (~9.81 m/s²) - .15 tolerance for noise
    if (abs(az_world - 9.81) < 0.15) {

      if (stableStart == 0)
        stableStart = millis();

      // 500 ms stability required to lock start getting readings
      if (millis() - stableStart > 500) {
        orientationReady = true;
        Serial.println("Orientation locked. System ready.");
      }

    } else {
      stableStart = 0;
    }

    return;  // Wait until orientation locked
  }

  // ========================
  // REAL OUTPUT STARTS HERE
  // ========================

  Serial.println(az_world);
}
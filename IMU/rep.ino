#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// ========================
// Calibration
// ========================
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float restingGravity = 0;
int sampleCount = 0;

bool calibrated = false;
bool orientationReady = false;

unsigned long startCalTime;
unsigned long stableStart = 0;
unsigned long lastTime = 0;

// ========================
// Filtering
// ========================
float linearZFiltered = 0;
float alpha = 0.3;

// ========================
// Integration
// ========================
float velocity = 0;
float prevVelocity = 0;
float position = 0;

// Track local extrema
float currentMinPos = 0;
float currentMaxPos = 0;
float positionTolerance = 0.003;   // 3mm tolerance

// ========================
// Accel Reversal Detection
// ========================
float accelThreshold = 0.35;
int confirmSamples = 5;

int positiveCount = 0;
int negativeCount = 0;

bool wasDescending = false;
bool wasAscending  = false;

void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);

  while (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU not detected...");
    delay(500);
  }

  Serial.println("Keep board still...");
  Serial.println("Calibrating for 5 seconds...");

  startCalTime = millis();

  filter.begin(100);
  filter.setBeta(0.1);
}

void loop() {

  if (!imu.dataReady()) return;
  imu.getAGMT();

  // ========================
  // CALIBRATION
  // ========================
  if (!calibrated) {

    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();

    float ax = imu.accX() * 0.00981;
    float ay = imu.accY() * 0.00981;
    float az = imu.accZ() * 0.00981;

    float gx = imu.gyrX() * PI / 180.0;
    float gy = imu.gyrY() * PI / 180.0;
    float gz = imu.gyrZ() * PI / 180.0;

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    sampleCount++;

    if (millis() - startCalTime >= 5000) {

      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;

      float qw, qx, qy, qz;
      filter.getQuaternion(&qw, &qx, &qy, &qz);

      restingGravity =
        (2*qx*qz - 2*qw*qy) * ax +
        (2*qy*qz + 2*qw*qx) * ay +
        (1 - 2*qx*qx - 2*qy*qy) * az;

      calibrated = true;
      lastTime = micros();

      currentMinPos = position;
      currentMaxPos = position;

      Serial.println("Calibration complete.");
    }

    return;
  }

  // ========================
  // TIMING
  // ========================
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // ========================
  // SENSOR UPDATE
  // ========================
  float ax = imu.accX() * 0.00981;
  float ay = imu.accY() * 0.00981;
  float az = imu.accZ() * 0.00981;

  float gx = (imu.gyrX() - gyroBiasX) * PI / 180.0;
  float gy = (imu.gyrY() - gyroBiasY) * PI / 180.0;
  float gz = (imu.gyrZ() - gyroBiasZ) * PI / 180.0;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  float az_world =
    (2*qx*qz - 2*qw*qy) * ax +
    (2*qy*qz + 2*qw*qx) * ay +
    (1 - 2*qx*qx - 2*qy*qy) * az;

  // ========================
  // ORIENTATION LOCK
  // ========================
  if (!orientationReady) {

    if (abs(az_world - restingGravity) < 0.2) {

      if (stableStart == 0)
        stableStart = millis();

      if (millis() - stableStart > 300) {
        orientationReady = true;
        Serial.println("System ready.");
      }

    } else {
      stableStart = 0;
    }

    return;
  }

  // ========================
  // LINEAR ACCELERATION
  // ========================
  float linearZ = restingGravity - az_world;
  linearZFiltered += alpha * (linearZ - linearZFiltered);

  // ========================
  // INTEGRATION
  // ========================
  prevVelocity = velocity;
  velocity += linearZFiltered * dt;
  position += velocity * dt;

  // Track extrema
  if (position < currentMinPos) currentMinPos = position;
  if (position > currentMaxPos) currentMaxPos = position;

  // ========================
  // ACCEL SIGN TRACKING
  // ========================
  if (linearZFiltered > accelThreshold) {
    positiveCount++;
    negativeCount = 0;
  }
  else if (linearZFiltered < -accelThreshold) {
    negativeCount++;
    positiveCount = 0;
  }
  else {
    positiveCount = 0;
    negativeCount = 0;
  }

  // ========================
  // BOTTOM DETECTION
  // ========================
  if (negativeCount >= confirmSamples) {
    wasDescending = true;
  }

  if (wasDescending &&
      positiveCount >= confirmSamples &&
      abs(position - currentMinPos) < positionTolerance) {

      Serial.println("BOTTOM DETECTED");

      wasDescending = false;
      currentMaxPos = position;  // reset max for next phase
  }

  // ========================
  // TOP DETECTION
  // ========================
  if (positiveCount >= confirmSamples) {
    wasAscending = true;
  }

  if (wasAscending &&
      negativeCount >= confirmSamples &&
      abs(position - currentMaxPos) < positionTolerance) {

      Serial.println("TOP DETECTED");

      wasAscending = false;
      currentMinPos = position;  // reset min for next phase
  }

  // Optional debug
  // Serial.println(position);
}
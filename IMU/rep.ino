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
bool gravityCalibrated = false;

float gravityAccum = 0;
int gravitySampleCount = 0;

unsigned long gravityCalStart = 0;
unsigned long startCalTime;
unsigned long lastTime;
unsigned long stableStart = 0;

// ========================
// Filtering
// ========================
static float linearZFiltered = 0;
float alpha = 0.15;

// ========================
// Velocity
// ========================
float velocity = 0;
// ========================
// Position
// ========================
float position = 0;

// ========================
// Movement Status Variables
// ========================

// ========================
// Rep Thresholds
// ========================


void setup() {

  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);

  while (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU not detected...");
    delay(500);
  }

  Serial.println("Keep board completely still...");
  Serial.println("Calibrating gyro for 5 seconds...");

  startCalTime = millis();

  filter.begin(100);
  filter.setBeta(0.15);
}

void loop() {

  if (!imu.dataReady()) return;
  imu.getAGMT();

  // ========================
  // PHASE 1: GYRO CALIBRATION
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

      calibrated = true;
      lastTime = micros();

      Serial.println("Gyro calibration complete.");
      Serial.println("Waiting for orientation lock...");
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
  // PHASE 2: ORIENTATION LOCK
  // ========================
  if (!orientationReady) {

    float accelMag = sqrt(ax*ax + ay*ay + az*az);

    if (abs(accelMag - 9.81) < 0.4) {
      if (stableStart == 0)
        stableStart = millis();

      if (millis() - stableStart > 500) {
        orientationReady = true;
        Serial.println("Orientation locked. Sampling resting gravity...");
      }
    } else {
      stableStart = 0;
    }

    return;
  }

  // ========================
  // PHASE 3: GRAVITY SAMPLE
  // ========================
  if (!gravityCalibrated) {

    if (gravityCalStart == 0)
      gravityCalStart = millis();

    gravityAccum += az_world;
    gravitySampleCount++;

    if (millis() - gravityCalStart >= 1000) {

      restingGravity = gravityAccum / gravitySampleCount;
      gravityCalibrated = true;

      Serial.println("System ready.");
      Serial.print("Resting gravity: ");
      Serial.println(restingGravity);
    }

    return;
  }

  // ========================
  // PHASE 4: LINEAR ACCELERATION
  // ========================
  float linearZ =  az_world - restingGravity;

  linearZFiltered += alpha * (linearZ - linearZFiltered);

 // ========================
// VELOCITY INTEGRATION
// ========================
velocity += linearZFiltered * dt;

// ========================
// POSITION INTEGRATION
// ========================
position += velocity * dt;




// ========================
// REP LOGIC STARTS
// ========================

}
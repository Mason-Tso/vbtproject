
#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// =============================
// TUNING PARAMETERS
// =============================
const float ACCEL_THRESHOLD      = 0.20f;   // ZUPT threshold
const float START_MOVE_THRESHOLD = 0.40f;   // Concentric start
const float MIN_REP_VELOCITY     = 0.15f;   // Valid rep minimum
const float EMA_ALPHA            = 0.20f;   // Smoothing

// =============================
// VARIABLES
// =============================
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool calibrated = false;

unsigned long startCalTime;
unsigned long lastTime;

float velocityZ = 0;
float peakVelocity = 0;
float sumVelocity = 0;
int sampleCount = 0;

float lastFilteredAz = 0;

enum State { IDLE, CONCENTRIC, RESETTING };
State deviceState = IDLE;

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) {
    delay(500);
  }

  Serial.println("Keep device STILL — calibrating gyro...");
  startCalTime = millis();

  filter.begin(100);      // 100 Hz
  filter.setBeta(0.05f);  // Stable gravity alignment
}

// =============================
// MAIN LOOP
// =============================
void loop() {

  if (!imu.dataReady()) return;
  imu.getAGMT();

  // =============================
  // 1) GYRO CALIBRATION (5s)
  // =============================
  if (!calibrated) {
    static int samples = 0;

    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();
    samples++;

    if (millis() - startCalTime >= 5000) {
      gyroBiasX /= samples;
      gyroBiasY /= samples;
      gyroBiasZ /= samples;
      calibrated = true;
      lastTime = micros();
      Serial.println("System Ready. Lift now.");
    }
    return;
  }

  // =============================
  // 2) TIMING
  // =============================
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;
  lastTime = now;

  // =============================
  // 3) SENSOR PROCESSING
  // =============================

  // mg → m/s² conversion (CRITICAL FIX)
  float ax = imu.accX() * 0.00980665f;
  float ay = imu.accY() * 0.00980665f;
  float az = imu.accZ() * 0.00980665f;

  float gx = (imu.gyrX() - gyroBiasX) * PI / 180.0f;
  float gy = (imu.gyrY() - gyroBiasY) * PI / 180.0f;
  float gz = (imu.gyrZ() - gyroBiasZ) * PI / 180.0f;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  float az_world =
      (2*qx*qz - 2*qw*qy) * ax
    + (2*qy*qz + 2*qw*qx) * ay
    + (1 - 2*qx*qx - 2*qy*qy) * az;

  // Gravity removal 
  float linearZ = az_world - 9.80665f;

  // Optional direction flip if needed:
  // linearZ = -linearZ;

  // =============================
  // 4) EMA FILTER
  // =============================
  float filteredAz = (EMA_ALPHA * linearZ)
                   + (1.0f - EMA_ALPHA) * lastFilteredAz;
  lastFilteredAz = filteredAz;

  // =============================
  // 5) ZUPT + VELOCITY INTEGRATION
  // =============================
  if (fabs(filteredAz) < ACCEL_THRESHOLD && deviceState != CONCENTRIC) {
    velocityZ = 0;
    if (deviceState == RESETTING)
      deviceState = IDLE;
  } else {
    velocityZ += filteredAz * dt;
  }

  // =============================
  // 6) VBT STATE MACHINE
  // =============================
  switch (deviceState) {

    case IDLE:
      if (filteredAz > START_MOVE_THRESHOLD) {
        deviceState = CONCENTRIC;
        peakVelocity = 0;
        sumVelocity = 0;
        sampleCount = 0;
      }
      break;

    case CONCENTRIC:

      if (velocityZ > 0) {
        sumVelocity += velocityZ;
        sampleCount++;
        if (velocityZ > peakVelocity)
          peakVelocity = velocityZ;
      }

      if (velocityZ <= 0.05f) {

        if (peakVelocity >= MIN_REP_VELOCITY && sampleCount > 5) {

          float meanVelocity = sumVelocity / sampleCount;

          Serial.println("---------- REP DETECTED ----------");
          Serial.print("MEAN VELOCITY: ");
          Serial.print(meanVelocity, 3);
          Serial.println(" m/s");

          Serial.print("PEAK VELOCITY: ");
          Serial.print(peakVelocity, 3);
          Serial.println(" m/s");

          Serial.println("----------------------------------");
        }

        deviceState = RESETTING;
        velocityZ = 0;
      }
      break;

    case RESETTING:
      break;
  }

  // =============================
  // DEBUG OUTPUT
  // =============================
  Serial.print(filteredAz, 4);
  Serial.print(",");
  Serial.println(velocityZ, 4);
}
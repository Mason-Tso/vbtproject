#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// ================================================================
// === CALIBRATION
// ================================================================
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float restingGravity = 0;
int sampleCount = 0;

bool calibrated = false;
bool orientationReady = false;
bool gravityCalibrated = false;

float gravityAccum = 0;
int gravitySampleCount = 0;
unsigned long startCalTime;
unsigned long lastTime;
unsigned long stableStart = 0;

// ================================================================
// === PHYSICS & FILTERING
// ================================================================
static float linearZFiltered = 0;
float alpha = 0.15;

float velocity = 0;
float position = 0;

// ================================================================
// === STATE MACHINE
// ================================================================
enum State { SEARCH_BOTTOM, SEARCH_TOP };
State currentState = SEARCH_BOTTOM;

int repCount = 0;
float currentROMGate = 0.10;
float velocityDeadzone = 0.03;
bool wasDescending = false;
bool wasAscending = false;

// ================================================================
// === TOP STABILITY + ZUPT
// ================================================================
float VEL_STABLE_THRESH = 0.06;
float ACC_STABLE_THRESH = 0.60;
unsigned long TOP_STABLE_MS = 120;

unsigned long topStableStart = 0;
float topStablePosition = 0;

// ================================================================
// === CONCENTRIC START + MCV
// ================================================================
float MIN_CONCENTRIC_ACCEL = 0.5;
float MIN_CONCENTRIC_VEL = 0.05;

bool concentricStarted = false;

// Time-weighted MCV
float velocityIntegral = 0;
float concentricTime = 0;
float meanConcentricVelocity = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) {
    delay(500);
  }

  Serial.println("IMU OK");
  Serial.println("Hold still for calibration...");

  startCalTime = millis();

  filter.begin(100);
  filter.setBeta(0.15);
}

void loop() {
  if (!imu.dataReady()) return;
  imu.getAGMT();

  // ================================================================
  // GYRO CALIBRATION
  // ================================================================
  if (!calibrated) {

    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();

    float ax = imu.accX() * 0.00981;
    float ay = imu.accY() * 0.00981;
    float az = imu.accZ() * 0.00981;

    filter.updateIMU(
      imu.gyrX()*PI/180,
      imu.gyrY()*PI/180,
      imu.gyrZ()*PI/180,
      ax, ay, az);

    if (millis() - startCalTime >= 5000) {
      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;
      calibrated = true;
      lastTime = micros();
      Serial.println("Gyro calibration complete.");
      Serial.println("Waiting for orientation lock...");
    }

    sampleCount++;
    return;
  }

  // ================================================================
  // TIMING
  // ================================================================
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // ================================================================
  // SENSOR UPDATE
  // ================================================================
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

  // ================================================================
  // GRAVITY CALIBRATION
  // ================================================================
  if (!orientationReady || !gravityCalibrated) {

    float accelMag = sqrt(ax*ax + ay*ay + az*az);

    if (abs(accelMag - 9.81) < 0.4) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart > 500) {
        orientationReady = true;
        Serial.println("Orientation locked.");
      }
    } else {
      stableStart = 0;
    }

    if (orientationReady && !gravityCalibrated) {
      gravityAccum += az_world;
      gravitySampleCount++;
      if (gravitySampleCount > 100) {
        restingGravity = gravityAccum / gravitySampleCount;
        gravityCalibrated = true;
        Serial.println("Gravity calibrated.");
        Serial.println("SYSTEM READY.");
      }
    }
    return;
  }

  // ================================================================
  // PHYSICS
  // ================================================================
  float linearZ = az_world - restingGravity;
  linearZFiltered += alpha * (linearZ - linearZFiltered);

  velocity += linearZFiltered * dt;
  position += velocity * dt;

  // ================================================================
  // SAFE ACTIVE ZUPT (ONLY BETWEEN REPS)
  // ================================================================
  if (currentState == SEARCH_BOTTOM &&
      abs(velocity) < 0.04 &&
      abs(linearZFiltered) < 0.25) {
    velocity = 0;
  }

  if (velocity < -velocityDeadzone) wasDescending = true;
  if (velocity > velocityDeadzone)  wasAscending  = true;

  // ================================================================
  // SEARCHING FOR BOTTOM
  // ================================================================
  if (currentState == SEARCH_BOTTOM) {

    if (position < -currentROMGate &&
        wasDescending &&
        velocity > velocityDeadzone) {

      Serial.println(">>> BOTTOM DETECTED");

      velocity = 0;
      position = 0;

      wasDescending = false;
      wasAscending = false;

      topStableStart = 0;
      concentricStarted = false;

      currentState = SEARCH_TOP;
    }
  }

  // ================================================================
  // SEARCHING FOR TOP
  // ================================================================
  else if (currentState == SEARCH_TOP) {
    // -------------------------------
    // CONCENTRIC START
    // -------------------------------
    if (!concentricStarted &&
        linearZFiltered > MIN_CONCENTRIC_ACCEL &&
        velocity > MIN_CONCENTRIC_VEL) {
      Serial.println(">>> CONCENTRIC START");
      velocity = 0;
      concentricStarted = true;
      velocityIntegral = 0;
      concentricTime = 0;
    }
    // -------------------------------
    // CONCENTRIC ZUPT
    // -------------------------------
    if (concentricStarted &&
        abs(velocity) < 0.15 &&
        abs(linearZFiltered) < 0.15) {
      velocity = 0;
    }

    // -------------------------------
    // MCV BUFFERING
    // -------------------------------
    if (concentricStarted && velocity > 0) {
      velocityIntegral += velocity * dt;
      concentricTime += dt;
    }

    // -------------------------------
    // TOP STABILITY
    // -------------------------------
    if (position > currentROMGate && wasAscending) {

      bool nearRest =
        (abs(velocity) < VEL_STABLE_THRESH) &&
        (abs(linearZFiltered) < ACC_STABLE_THRESH);

      if (nearRest) {

        if (topStableStart == 0) {
          Serial.println(">>> TOP STABILITY START");
          topStableStart = millis();
          topStablePosition = position;
        }

        if (millis() - topStableStart > TOP_STABLE_MS) {

          repCount++;

          float finalROM = abs(topStablePosition);
          currentROMGate = finalROM * 0.6;

          if (concentricTime > 0)
            meanConcentricVelocity = velocityIntegral / concentricTime;
          else
            meanConcentricVelocity = 0;

          Serial.print(">>> TOP CONFIRMED | Rep ");
          Serial.print(repCount);
          Serial.print(" | MCV: ");
          Serial.println(meanConcentricVelocity, 3);

          velocity = 0;
          position = 0;

          wasDescending = false;
          wasAscending = false;

          topStableStart = 0;

          currentState = SEARCH_BOTTOM;
        }
      }
      else {
        topStableStart = 0;
      }
    }
    else {
      topStableStart = 0;
    }
  }

  // Optional debug stream
  Serial.print(millis());
  Serial.print(",");
  Serial.print(linearZFiltered);
  Serial.print(",");
  Serial.print(velocity);
  Serial.print(",");
  Serial.println(position);
}
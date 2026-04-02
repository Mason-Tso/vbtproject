#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>
#define AD0_VAL 0
ICM_20948_I2C imu;
Adafruit_Madgwick filter;
// ================================================================
// === CALIBRATION & TIMING
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
// === BUTTERWORTH FILTER (15 Hz @ 225 Hz)
// ================================================================
float v_in[3]  = {0, 0, 0};
float v_out[3] = {0, 0, 0};
const float b_coeff[3] = {0.067455, 0.134911, 0.067455};
const float a_coeff[3] = {1.000000, -1.142980, 0.412802};
float applyButterworth(float input) {
  v_in[0] = input;
  float output =
      b_coeff[0]*v_in[0] +
      b_coeff[1]*v_in[1] +
      b_coeff[2]*v_in[2] -
      a_coeff[1]*v_out[1] -
      a_coeff[2]*v_out[2];
  v_in[2]  = v_in[1];
  v_in[1]  = v_in[0];
  v_out[2] = v_out[1];
  v_out[1] = output;
  return output;
}
// ================================================================
// === PHYSICS & STATE
// ================================================================
float linearZFiltered = 0;
float lastLinearZ = 0;
float velocity = 0;
float lastVelocity = 0;
float position = 0;
float ACC_DEADBAND = 0.04;
float DECAY = 0.990;
enum State { SEARCH_BOTTOM, SEARCH_TOP };
State currentState = SEARCH_BOTTOM;
int repCount = 0;
float currentROMGate = 0.10;
float velocityDeadzone = 0.03;
bool wasDescending = false, wasAscending = false;
// Stability & MCV
float VEL_STABLE_THRESH = 0.06;
float ACC_STABLE_THRESH = 0.60;
unsigned long TOP_STABLE_MS = 180;
unsigned long topStableStart = 0;
unsigned long topUnstableStart = 0;
float topStablePosition = 0;
float MIN_CONCENTRIC_ACCEL = 0.5;
float MIN_CONCENTRIC_VEL = 0.05;
bool concentricStarted = false;
float velocityIntegral = 0, concentricTime = 0, meanConcentricVelocity = 0;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(400000);
  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) delay(500);
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 4;
  mySmplrt.a = 4;
  imu.setSampleRate(ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc, mySmplrt);
  Serial.println("VBT V1 Ready");
  startCalTime = millis();
  filter.begin(225);
  filter.setBeta(0.1);
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
    filter.updateIMU(
      imu.gyrX()*PI/180,
      imu.gyrY()*PI/180,
      imu.gyrZ()*PI/180,
      imu.accX(),
      imu.accY(),
      imu.accZ());
    if (millis() - startCalTime >= 5000) {
      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;
      calibrated = true;
      lastTime = micros();
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
  if (dt <= 0 || dt > 0.1) dt = 0.0045;
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
  // GRAVITY LOCK
  // ================================================================
  if (!orientationReady || !gravityCalibrated) {
    float accelMag = sqrt(ax*ax + ay*ay + az*az);
    if (abs(accelMag - 9.81) < 0.3) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart > 1000) orientationReady = true;
    } else stableStart = 0;
    if (orientationReady && !gravityCalibrated) {
      gravityAccum += az_world;
      gravitySampleCount++;
      if (gravitySampleCount > 200) {
        restingGravity = gravityAccum / gravitySampleCount;
        gravityCalibrated = true;
        Serial.println("SYSTEM LIVE");
      }
    }
    return;
  }
  // ================================================================
  // PHYSICS
  // ================================================================
  float linearZ = az_world - restingGravity;
  linearZFiltered = applyButterworth(linearZ);
  velocity += ((linearZFiltered + lastLinearZ) / 2.0) * dt;
  if (abs(linearZFiltered) < ACC_DEADBAND)
      velocity *= DECAY;
  position += ((velocity + lastVelocity) / 2.0) * dt;
  if (velocity < -velocityDeadzone) wasDescending = true;
  if (velocity > velocityDeadzone)  wasAscending  = true;
  // ================================================================
  // STATE MACHINE
  // ================================================================
  if (currentState == SEARCH_BOTTOM) {
    if (abs(velocity) < 0.04 && abs(linearZFiltered) < 0.25)
      velocity = 0;
    if (position < -currentROMGate &&
        wasDescending &&
        velocity > velocityDeadzone) {
      velocity = 0;
      position = 0;
      wasDescending = false;
      wasAscending = false;
      concentricStarted = false;
      topStableStart = 0;
      topUnstableStart = 0;
      currentState = SEARCH_TOP;
      Serial.println(">>> BOTTOM");
    }
  }
  else if (currentState == SEARCH_TOP) {
    if (!concentricStarted &&
        linearZFiltered > MIN_CONCENTRIC_ACCEL &&
        velocity > MIN_CONCENTRIC_VEL) {
      concentricStarted = true;
      velocityIntegral = 0;
      concentricTime = 0;
    }
    if (concentricStarted && velocity > 0) {
      velocityIntegral += ((velocity + lastVelocity) / 2.0) * dt;
      concentricTime += dt;
    }
    if (position > currentROMGate && wasAscending) {
      bool nearRest =
        (abs(velocity) < VEL_STABLE_THRESH) &&
        (abs(linearZFiltered) < ACC_STABLE_THRESH);
      if (nearRest) {
        topUnstableStart = 0;
        if (topStableStart == 0) {
          topStableStart = millis();
          topStablePosition = position;
        }
        if (millis() - topStableStart > TOP_STABLE_MS) {
          repCount++;
          currentROMGate = abs(topStablePosition) * 0.6;
          meanConcentricVelocity =
              (concentricTime > 0) ?
              (velocityIntegral / concentricTime) : 0;
          Serial.print(">>> REP: ");
          Serial.print(repCount);
          Serial.print(" | MCV: ");
          Serial.println(meanConcentricVelocity, 3);
          velocity = 0;
          position = 0;
          wasDescending = false;
          wasAscending = false;
          topStableStart = 0;
          topUnstableStart = 0;
          currentState = SEARCH_BOTTOM;
        }
      } else {
        if (topUnstableStart == 0) topUnstableStart = millis();
        if (millis() - topUnstableStart > 100) {
          topStableStart = 0;
        }
      }
    } else {
      topStableStart = 0;
      topUnstableStart = 0;
    }
  }
  lastLinearZ = linearZFiltered;
  lastVelocity = velocity;
  Serial.print(millis()); Serial.print(",");
  Serial.print(linearZFiltered); Serial.print(",");
  Serial.print(velocity); Serial.print(",");
  Serial.println(position);
}

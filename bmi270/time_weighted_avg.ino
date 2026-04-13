#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h>

BMI270 imu;
Adafruit_Madgwick filter;

// ==========================
// STRUCTS & RESULTS
// ==========================
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
};

struct RepResult {
  uint8_t   repNumber;
  float     meanVelocity;
  float     peakVelocity;
  float     rom;
  uint16_t  durationMs;
  bool      valid;
  uint32_t  timestamp;
};

// ==========================
// CALIBRATION & TIMING
// ==========================
unsigned long lastTime;
bool gyroCalibrated = false;
int  gyroSamples    = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

bool orientationReady = false;
bool gravityReady     = false;
float restingGravity  = 0;
float gravityAccum    = 0;
int   gravitySamples  = 0;

unsigned long stableStart = 0;
float last_qw = 1, last_qx = 0, last_qy = 0, last_qz = 0;

// ==========================
// STILLNESS & VARIANCE
// ==========================
#define WINDOW_SIZE 15
float linZBuffer[WINDOW_SIZE];
float gyroBuffer[WINDOW_SIZE];
int   bufferIndex = 0;
bool  bufferFull  = false;

// Industrial-grade noise floor constants
const float LINZ_VAR_THRESH = 0.005;  
const float GYRO_VAR_THRESH = 0.0005; 
const float STILL_ACC_THRESH = 0.25;  

unsigned long stillStart = 0;
float linZVar = 0, gyroVar = 0, stillStrength = 0;

// ==========================
// VELOCITY & POSITION
// ==========================
float velocityZ = 0, prevLinearZ = 0, velocityFiltered_global = 0;
float positionZ = 0, prevVelocityZ = 0;

// Butterworth 4Hz @ 800Hz
float v0 = 0, v1 = 0, v2 = 0;
const float b0 = 0.0028981946, b1 = 0.0057963892, b2 = 0.0028981946;
const float a1 = -1.8226949, a2 = 0.8371817;

// ==========================
// STATE MACHINE GLOBALS
// ==========================
enum LiftState { STATE_IDLE, STATE_UNRACKING, STATE_ECCENTRIC, STATE_CONCENTRIC, STATE_FAULT };
LiftState currentState = STATE_IDLE;
unsigned long stateEnterTime = 0;
uint8_t repCount = 0;

int8_t concentricSign = 0;
float  concentricArea = 0; // The Integral: Sum(Velocity * dt)
float  concentricTime = 0; // The Time: Sum(dt)
float  concentricPeak = 0;
unsigned long concentricStart = 0;
float  startPosConcentric = 0;

// Confirmation Timers
unsigned long motionStartTimer = 0; 
unsigned long stillConfirmStart = 0;
unsigned long reversalStart = 0;
unsigned long concentricCandidateStart = 0;

// Industry Standard Thresholds
const unsigned long CONCENTRIC_CONFIRM_MS = 60;
const unsigned long REVERSAL_CONFIRM_MS = 40;
const float VEL_THRESHOLD_CONCENTRIC = 0.08;
const float VEL_ACCUMULATE_MIN = 0.05;
const float MIN_ROM = 0.10; 
const unsigned long MIN_CONCENTRIC_MS = 100;
const unsigned long REP_COOLDOWN_MS = 500;
unsigned long lastRepTime = 0;

// ==========================
// FORWARD DECLARATIONS
// ==========================
void transitionTo(LiftState newState);
void onExitConcentric();
void emitRep(RepResult &rep);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (imu.beginI2C() != BMI2_OK) {
    Serial.println("BMI270 Fail");
    while (1);
  }

  filter.begin(800);
  filter.setBeta(0.5);
  lastTime = micros();
  Serial.println("System Ready. Calibrating...");
}

// --------------------------
// MATH ENGINE
// --------------------------
ImuData imu_read() {
  imu.getSensorData();
  ImuData d;
  d.ax = imu.data.accelX; d.ay = imu.data.accelY; d.az = imu.data.accelZ;
  d.gx = imu.data.gyroX * PI / 180.0; d.gy = imu.data.gyroY * PI / 180.0; d.gz = imu.data.gyroZ * PI / 180.0;
  return d;
}

float computeVariance(float *buf) {
  float mean = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) mean += buf[i];
  mean /= WINDOW_SIZE;
  float var = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) { float d = buf[i] - mean; var += d * d; }
  return var / WINDOW_SIZE;
}

float butterworth(float input) {
  v0 = input - a1 * v1 - a2 * v2;
  float out = b0 * v0 + b1 * v1 + b2 * v2;
  v2 = v1; v1 = v0;
  return out;
}

void updateStillnessBuffers(float lz, float gMag) {
  linZBuffer[bufferIndex] = lz; gyroBuffer[bufferIndex] = gMag;
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
  if (bufferIndex == 0) bufferFull = true;
}

// --------------------------
// STATE MACHINE LOGIC
// --------------------------
void transitionTo(LiftState newState) {
  if (currentState == STATE_CONCENTRIC) onExitConcentric();
  
  currentState = newState;
  stateEnterTime = millis();
  
  if (newState == STATE_IDLE) Serial.println(">>> STATE: IDLE");
  if (newState == STATE_UNRACKING) Serial.println(">>> STATE: UNRACKING");
  if (newState == STATE_ECCENTRIC) Serial.println(">>> STATE: ECCENTRIC");
  if (newState == STATE_CONCENTRIC) {
    concentricArea = 0; concentricTime = 0; concentricPeak = 0;
    concentricStart = millis(); startPosConcentric = positionZ;
    concentricSign = (velocityFiltered_global > 0) ? 1 : -1;
    Serial.println(">>> STATE: CONCENTRIC");
  }
}

void onExitConcentric() {
  // Check total accumulated time instead of samples
  if (concentricTime < (MIN_CONCENTRIC_MS / 1000.0)) return;
  if (millis() - lastRepTime < REP_COOLDOWN_MS) return;

  float rom = abs(positionZ - startPosConcentric);
  if (rom < MIN_ROM) return;

  RepResult rep;
  rep.repNumber = ++repCount;
  // Calculate Mean via Time-Weighted Integral
  rep.meanVelocity = (concentricTime > 0) ? (concentricArea / concentricTime) : 0;
  rep.peakVelocity = concentricPeak;
  rep.rom = rom;
  rep.durationMs = (uint16_t)(concentricTime * 1000.0);
  rep.valid = (rep.meanVelocity > 0.05);

  lastRepTime = millis();
  emitRep(rep);
}

void emitRep(RepResult &rep) {
  if (!rep.valid) { repCount--; return; }
  Serial.print(">>> REP "); Serial.print(rep.repNumber);
  Serial.print(" | MCV: "); Serial.print(rep.meanVelocity, 3);
  Serial.print(" m/s | ROM: "); Serial.print(rep.rom * 100.0, 1);
  Serial.println(" cm");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.01) dt = 0.00125;

  ImuData raw = imu_read();

  // 1. Core Calibration
  if (!gyroCalibrated) {
    float gMag = sqrt(raw.gx*raw.gx + raw.gy*raw.gy + raw.gz*raw.gz);
    if (gMag < 0.05) { gyroBiasX += raw.gx; gyroBiasY += raw.gy; gyroBiasZ += raw.gz; gyroSamples++; }
    if (gyroSamples > 1000) { gyroBiasX /= 1000; gyroBiasY /= 1000; gyroBiasZ /= 1000; gyroCalibrated = true; }
    return;
  }

  // 2. Fusion & World Z
  float gx = raw.gx - gyroBiasX, gy = raw.gy - gyroBiasY, gz = raw.gz - gyroBiasZ;
  float ax_n = raw.ax, ay_n = raw.ay, az_n = raw.az;
  float norm = sqrt(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
  if (norm > 0) { ax_n /= norm; ay_n /= norm; az_n /= norm; }
  filter.updateIMU(gx, gy, gz, ax_n, ay_n, az_n);
  
  float qw, qx, qy, qz; filter.getQuaternion(&qw, &qx, &qy, &qz);
  float dq = abs(qw-last_qw) + abs(qx-last_qx) + abs(qy-last_qy) + abs(qz-last_qz);
  last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;

  float az_world = (2*qx*qz - 2*qw*qy)*raw.ax + (2*qy*qz + 2*qw*qx)*raw.ay + (1 - 2*qx*qx - 2*qy*qy)*raw.az;

  if (!gravityReady) {
    if (dq < 0.0008 && sqrt(gx*gx+gy*gy+gz*gz) < 0.05) {
      gravityAccum += az_world; gravitySamples++;
      if (gravitySamples >= 1000) { restingGravity = gravityAccum / 1000; gravityReady = true; transitionTo(STATE_IDLE); }
    }
    return;
  }

  // 3. Stillness & Physics Engine
  float linearZ = (az_world - restingGravity) * 9.81;
  updateStillnessBuffers(linearZ, sqrt(gx*gx+gy*gy+gz*gz));

  bool systemStill = false;
  if (bufferFull) {
    linZVar = computeVariance(linZBuffer);
    gyroVar = computeVariance(gyroBuffer);
    stillStrength = (linZVar / LINZ_VAR_THRESH) + (gyroVar / GYRO_VAR_THRESH);
    if (linZVar < LINZ_VAR_THRESH && gyroVar < GYRO_VAR_THRESH && abs(linearZ) < 0.35) {
      if (stillStart == 0) stillStart = millis();
      if (millis() - stillStart > 50) systemStill = true;
    } else { stillStart = 0; }
  }

  if (systemStill) {
    restingGravity = 0.9995 * restingGravity + 0.0005 * az_world;
    velocityZ = 0; prevLinearZ = 0; v0=v1=v2=0; prevVelocityZ = 0;
  } else {
    velocityZ += 0.5 * (prevLinearZ + linearZ) * dt;
    prevLinearZ = linearZ;
  }

  float velocityFiltered = butterworth(velocityZ);
  velocityFiltered_global = velocityFiltered;
  if (!systemStill) { positionZ += 0.5 * (prevVelocityZ + velocityFiltered) * dt; prevVelocityZ = velocityFiltered; }

  // --------------------------
  // STATE GATEKEEPER
  // --------------------------
  if (currentState == STATE_IDLE) {
    if (!systemStill) {
      if (motionStartTimer == 0) motionStartTimer = millis();
      // Persistent motion check to prevent desk-jitter unrack
      if (millis() - motionStartTimer > 60) { transitionTo(STATE_UNRACKING); motionStartTimer = 0; }
    } else { motionStartTimer = 0; }
  } 
  else if (currentState == STATE_UNRACKING) {
    if (systemStill) { positionZ = 0; transitionTo(STATE_ECCENTRIC); }
  } 
  else if (currentState == STATE_ECCENTRIC) {
    if (systemStill) { positionZ = 0; }
    else if ((concentricSign != 0 ? concentricSign*velocityFiltered : velocityFiltered) > VEL_THRESHOLD_CONCENTRIC) {
       if (concentricCandidateStart == 0) concentricCandidateStart = millis();
       if (millis() - concentricCandidateStart > CONCENTRIC_CONFIRM_MS) { transitionTo(STATE_CONCENTRIC); }
    } else { concentricCandidateStart = 0; }
  } 
  else if (currentState == STATE_CONCENTRIC) {
    float sVel = concentricSign * velocityFiltered;
    
    // Finalize: Stillness
    if (systemStill) {
      if (stillConfirmStart == 0) stillConfirmStart = millis();
      unsigned long cTime = (stillStrength < 0.5 && abs(linearZ) < 0.1) ? 30 : 80;
      if (millis() - stillConfirmStart > cTime) { stillConfirmStart = 0; transitionTo(STATE_ECCENTRIC); }
    } 
    // Finalize: Reversal (Touch & Go)
    else if (sVel < -VEL_THRESHOLD_CONCENTRIC) {
      if (reversalStart == 0) reversalStart = millis();
      if (millis() - reversalStart > REVERSAL_CONFIRM_MS && abs(positionZ-startPosConcentric) > MIN_ROM) { 
        transitionTo(STATE_ECCENTRIC); 
      }
    } 
    // Accumulate: Time-Weighted Integral
    else {
      stillConfirmStart = 0; reversalStart = 0;
      if (sVel > VEL_ACCUMULATE_MIN && !systemStill) {
        concentricArea += (sVel * dt); 
        concentricTime += dt;
        if (sVel > concentricPeak) concentricPeak = sVel;
      }
    }
  }
}
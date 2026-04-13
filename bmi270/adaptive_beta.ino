#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h>

BMI270 imu;
Adafruit_Madgwick filter;

// ==========================
// CONFIGS & ADAPTIVE VARS
// ==========================
float betaStill  = 0.50; 
float betaMotion = 0.30; 
bool idleArmed   = false;
unsigned long unrackSettleStart = 0;

// ==========================
// STRUCTS
// ==========================
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
};

struct RepResult {
  uint8_t  repNumber;
  float    meanVelocity;
  float    peakVelocity;
  float    rom;
  uint16_t durationMs;
  bool     valid;
  uint32_t timestamp;
};

// ==========================
// TIMING & CALIBRATION
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
// VARIANCE BUFFERS
// ==========================
#define WINDOW_SIZE 15
float linZBuffer[WINDOW_SIZE];
float gyroBuffer[WINDOW_SIZE];
int   bufferIndex = 0;
bool  bufferFull  = false;

const float LINZ_VAR_THRESH = 0.004;
const float GYRO_VAR_THRESH = 0.0004;
const float STILL_ACC_THRESH  = 0.15;
const float STILL_GYRO_THRESH = 0.05;
const int   REQUIRED_SAMPLES  = 1000;

unsigned long stillStart = 0;
float linZVar = 0, gyroVar = 0;

// ==========================
// MOTION VARS
// ==========================
float velocityZ = 0;
float prevLinearZ = 0;
float velocityFiltered_global = 0;
float positionZ = 0;
float prevVelocityZ = 0;

// Butterworth 2nd Order
float v0 = 0, v1 = 0, v2 = 0;
const float b0 = 0.0028981946, b1 = 0.0057963892, b2 = 0.0028981946;
const float a1 = -1.8226949, a2 = 0.8371817;

// ==========================
// STATE MACHINE VARS
// ==========================
enum LiftState { STATE_IDLE, STATE_UNRACKING, STATE_ECCENTRIC, STATE_CONCENTRIC, STATE_FAULT };
LiftState currentState = STATE_IDLE;
unsigned long stateEnterTime = 0;
uint8_t repCount = 0;
int8_t concentricSign = 0;

float concentricSum = 0, concentricPeak = 0, startPosConcentric = 0;
int concentricSamples = 0;
unsigned long concentricStart = 0, lastRepTime = 0, stillConfirmStart = 0, concentricCandidateStart = 0;

// THRESHOLDS
const float VEL_THRESHOLD_CONCENTRIC = 0.04; 
const float VEL_ACCUMULATE_MIN = 0.025;
const float MIN_ROM = 0.10;
const float MAX_PEAK_VELOCITY = 3.5;
const float MIN_MEAN_VELOCITY = 0.03;

const unsigned long STILL_CONFIRM_MS = 100;
const unsigned long MIN_CONCENTRIC_MS = 100;
const unsigned long REP_COOLDOWN_MS   = 500;
const unsigned long CONCENTRIC_CONFIRM_MS = 30; 

// Timeouts
const unsigned long TIMEOUT_UNRACKING  = 10000;
const unsigned long TIMEOUT_ECCENTRIC  = 10000;
const unsigned long TIMEOUT_CONCENTRIC = 8000;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 20;

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
  if (imu.beginI2C() != BMI2_OK) { while (1); }
  filter.begin(800);
  lastTime = micros();
  Serial.println("VBT System Online | Feedback-Corrected Pipeline Active");
}

ImuData imu_read() {
  imu.getSensorData();
  ImuData d;
  d.ax = imu.data.accelX; d.ay = imu.data.accelY; d.az = imu.data.accelZ;
  d.gx = imu.data.gyroX * PI / 180.0; d.gy = imu.data.gyroY * PI / 180.0; d.gz = imu.data.gyroZ * PI / 180.0;
  return d;
}

float butterworth(float input) {
  v0 = input - a1 * v1 - a2 * v2;
  float output = b0 * v0 + b1 * v1 + b2 * v2;
  v2 = v1; v1 = v0;
  return output;
}

float computeVariance(float *buf) {
  float mean = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) mean += buf[i];
  mean /= WINDOW_SIZE;
  float var = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) { float d = buf[i] - mean; var += d * d; }
  return var / WINDOW_SIZE;
}

void loop() {
  unsigned long now_u = micros(); 
  unsigned long now_m = millis(); 
  float dt = (now_u - lastTime) / 1000000.0;
  lastTime = now_u;
  
  if (dt <= 0 || dt > 0.01) dt = 0.00125;

  ImuData raw = imu_read();

  // -------------------------------------------------------
  // STEP 1: GYRO CALIBRATION
  // -------------------------------------------------------
  if (!gyroCalibrated) {
    float gyroMagRaw = sqrt(raw.gx*raw.gx + raw.gy*raw.gy + raw.gz*raw.gz);
    if (gyroMagRaw < 0.05) {
      gyroBiasX += raw.gx; gyroBiasY += raw.gy; gyroBiasZ += raw.gz;
      gyroSamples++;
    }
    if (gyroSamples >= 1000) {
      gyroBiasX /= 1000.0; gyroBiasY /= 1000.0; gyroBiasZ /= 1000.0;
      gyroCalibrated = true;
      Serial.println(">>> STEP 1 COMPLETE: Gyro calibrated");
    }
    return; 
  }

  // --- APPLY BIAS & COMPUTE MAGNITUDES ---
  float gx = raw.gx - gyroBiasX;
  float gy = raw.gy - gyroBiasY;
  float gz = raw.gz - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float accelMag = sqrt(raw.ax*raw.ax + raw.ay*raw.ay + raw.az*raw.az);

  // --- FAST STILLNESS (FOR BETA) ---
  bool basicStill = (gyroMag < STILL_GYRO_THRESH) && (abs(accelMag - 1.0) < STILL_ACC_THRESH);

  // --- STEP 2: BETA LOGIC (CLEAN INTERPOLATION) ---
  float beta;
  if (basicStill) {
    beta = betaStill; // 0.50
  } else {
    float motionLevel = constrain(gyroMag / 0.5, 0, 1);
    // Interpolates between motion (0.3) and still (0.5) based on gyro
    beta = betaStill - (betaStill - betaMotion) * motionLevel;
  }
  filter.setBeta(constrain(beta, 0.25, 0.60));

  // --- UPDATE FILTER ---
  float ax_n = raw.ax, ay_n = raw.ay, az_n = raw.az;
  float norm = sqrt(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
  if (norm > 0) { ax_n /= norm; ay_n /= norm; az_n /= norm; }
  filter.updateIMU(gx, gy, gz, ax_n, ay_n, az_n);

  // --- GET WORLD-FRAME DATA ---
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  float az_world = (2*qx*qz - 2*qw*qy) * raw.ax + (2*qy*qz + 2*qw*qx) * raw.ay + (1 - 2*qx*qx - 2*qy*qy) * raw.az;

  // -------------------------------------------------------
  // STEP 2: ORIENTATION CONVERGENCE
  // -------------------------------------------------------
  if (!orientationReady) {
    float dq = abs(qw-last_qw) + abs(qx-last_qx) + abs(qy-last_qy) + abs(qz-last_qz);
    last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
    
    if (dq < 0.0008 && basicStill && abs(az_world) > 0.90) {
      if (stableStart == 0) stableStart = now_m;
      if (now_m - stableStart > 500) {
        orientationReady = true;
        Serial.println(">>> STEP 2 COMPLETE: Orientation converged");
      }
    } else { stableStart = 0; }
    
    if (now_m - lastPrint > PRINT_INTERVAL_MS) {
      lastPrint = now_m;
      Serial.print("WAIT_ORIENT | AccMag:"); Serial.print(accelMag, 3);
      Serial.print(" | az_world:"); Serial.print(az_world, 3);
      Serial.print(" | dq:"); Serial.println(dq, 6);
    }
    return;
  }

  // -------------------------------------------------------
  // LIVE PREP: STILLNESS DEBOUNCE (FOR ZUPT)
  // -------------------------------------------------------
  float linearZ = (az_world - restingGravity) * 9.81;

  linZBuffer[bufferIndex] = linearZ; gyroBuffer[bufferIndex] = gyroMag;
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
  if (bufferIndex == 0) bufferFull = true;
  if (bufferFull) {
    linZVar = computeVariance(linZBuffer);
    gyroVar = computeVariance(gyroBuffer);
  }

  // Re-implemented time confirmation
  bool stillCandidate;

  if (!gravityReady) {
      // PRE-GRAVITY: don't use linearZ
      stillCandidate =
          basicStill &&
          (gyroVar < GYRO_VAR_THRESH * 2.0);
  } else {
      // NORMAL OPERATION
      stillCandidate =
          (linZVar < LINZ_VAR_THRESH * 2.0) &&
          (gyroVar < GYRO_VAR_THRESH * 2.0) &&
          basicStill &&
          (abs(linearZ) < 0.3);
  }

  bool trueStill = false;
  if (stillCandidate) {
    if (stillStart == 0) stillStart = now_m;
    if (now_m - stillStart > 50) trueStill = true;
  } else {
    stillStart = 0;
  }

  // -------------------------------------------------------
  // STEP 3: GRAVITY CALIBRATION
  // -------------------------------------------------------
  if (!gravityReady) {
    if (trueStill) { // Use debounced stillness for calibration safety
      gravityAccum += az_world;
      gravitySamples++;
      if (gravitySamples >= 1000) {
        restingGravity = gravityAccum / 1000.0;
        gravityReady = true;
        Serial.println(">>> STEP 3 COMPLETE: Gravity Ready. SYSTEM LIVE.");
        transitionTo(STATE_IDLE);
      }
    }
    if (now_m - lastPrint > PRINT_INTERVAL_MS) {
      lastPrint = now_m;
      Serial.print("WAIT_GRAV | Samples:"); Serial.print(gravitySamples);
      Serial.print(" | az_world:"); Serial.println(az_world, 4);
    }
    return; 
  }

  // -------------------------------------------------------
  // LIVE PIPELINE
  // -------------------------------------------------------
  velocityZ += 0.5 * (prevLinearZ + linearZ) * dt;
  prevLinearZ = linearZ;
  float vFilt = butterworth(velocityZ);

  if (trueStill) {
    velocityZ = 0; prevLinearZ = 0; v0 = v1 = v2 = 0; prevVelocityZ = 0; vFilt = 0;
    restingGravity = 0.999 * restingGravity + 0.001 * az_world;
  }

  if (trueStill && abs(vFilt) < 0.04) vFilt = 0;
  velocityFiltered_global = vFilt;

  if (!trueStill) positionZ += 0.5 * (prevVelocityZ + vFilt) * dt;
  prevVelocityZ = vFilt;

  // --- STATE MACHINE ---
  switch (currentState) {

  // ==========================
  // IDLE
  // ==========================
  case STATE_IDLE:
    if (trueStill && (millis() - stateEnterTime > 500)) {
      idleArmed = true;
    }

    if (idleArmed && !trueStill && abs(vFilt) > 0.05) {
      transitionTo(STATE_UNRACKING);
    }
    break;

  // ==========================
  // UNRACKING
  // ==========================
  case STATE_UNRACKING:

    // Detect first downward motion → start eccentric
    if (vFilt < -0.05) {
      positionZ = 0;
      transitionTo(STATE_ECCENTRIC);
    }

    // If user settles before moving → still go eccentric
    else if (trueStill) {
      if (unrackSettleStart == 0)
        unrackSettleStart = millis();

      if (millis() - unrackSettleStart > 200) {
        positionZ = 0;
        transitionTo(STATE_ECCENTRIC);
      }
    }

    // Timeout fallback
    else if (millis() - stateEnterTime > 2500) {
      positionZ = 0;
      transitionTo(STATE_ECCENTRIC);
    }

    if (millis() - stateEnterTime > TIMEOUT_UNRACKING)
      transitionTo(STATE_FAULT);

    break;

  // ==========================
  // ECCENTRIC
  // ==========================
  case STATE_ECCENTRIC: {

    // Reset position if user pauses at bottom
    if (trueStill && (millis() - stateEnterTime > 200)) {
      positionZ = 0;
      break;
    }

    // Sign-safe velocity
    float sVel = (concentricSign != 0) ? concentricSign * vFilt : vFilt;

    // Detect upward (concentric) motion
    if (sVel > VEL_THRESHOLD_CONCENTRIC) {

      if (concentricCandidateStart == 0)
        concentricCandidateStart = millis();

      if (millis() - concentricCandidateStart > CONCENTRIC_CONFIRM_MS) {
        transitionTo(STATE_CONCENTRIC);
        concentricCandidateStart = 0;
      }

    } else {
      concentricCandidateStart = 0;
    }

    if (millis() - stateEnterTime > TIMEOUT_ECCENTRIC)
      transitionTo(STATE_FAULT);

    break;
  }

  // ==========================
  // CONCENTRIC
  // ==========================
  case STATE_CONCENTRIC: {

    float curS = concentricSign * vFilt;

    // Lockout detection (top of rep)
    if (trueStill) {
      if (stillConfirmStart == 0)
        stillConfirmStart = millis();

      if (millis() - stillConfirmStart > STILL_CONFIRM_MS) {
        transitionTo(STATE_ECCENTRIC);
      }

    } else {
      stillConfirmStart = 0;

      // Accumulate metrics
      if (curS > VEL_ACCUMULATE_MIN) {
        concentricSum += curS;
        concentricSamples++;

        if (curS > concentricPeak && curS < 3.0) {
          concentricPeak = curS;
        }
      }
    }

    if (millis() - stateEnterTime > TIMEOUT_CONCENTRIC)
      transitionTo(STATE_FAULT);

    break;
  }

  // ==========================
  // FAULT
  // ==========================
  case STATE_FAULT:
    if (trueStill && millis() - stateEnterTime > 3000) {
      transitionTo(STATE_IDLE);
    }
    break;
}

  // --- FINAL DEBUG PRINT ---
  if (now_m - lastPrint > PRINT_INTERVAL_MS) {
    lastPrint = now_m;
    Serial.print("V:"); Serial.print(velocityFiltered_global, 3);
    Serial.print(" | P:"); Serial.print(positionZ, 3);
    Serial.print(" | AccMag:"); Serial.print(accelMag, 3);
    Serial.print(" | linZ:"); Serial.print(linearZ, 3);
    Serial.print(" | Still:"); Serial.print(trueStill);
    Serial.print(" | State:"); Serial.println(currentState);
  }
}

void transitionTo(LiftState newState) {
  if (currentState == STATE_CONCENTRIC) onExitConcentric();
  currentState = newState;
  stateEnterTime = millis();
  unrackSettleStart = 0; idleArmed = false; 
  stillConfirmStart = 0; concentricCandidateStart = 0;

  if (newState == STATE_CONCENTRIC) {
    velocityZ = 0;
    v0 = v1 = v2 = 0;
    prevVelocityZ = 0;
    prevLinearZ = 0;
    concentricSign = (velocityFiltered_global >= 0) ? 1 : -1;
    concentricSum = 0; concentricPeak = 0; concentricSamples = 0;
    concentricStart = millis(); startPosConcentric = positionZ;
  }
  Serial.print(">>> STATE: "); 
  Serial.println(newState == STATE_IDLE ? "IDLE" : newState == STATE_UNRACKING ? "UNRACKING" : newState == STATE_ECCENTRIC ? "ECCENTRIC" : newState == STATE_CONCENTRIC ? "CONCENTRIC" : "FAULT");
}

void onExitConcentric() {
  unsigned long dur = millis() - concentricStart;
  if (dur < MIN_CONCENTRIC_MS || (millis() - lastRepTime < REP_COOLDOWN_MS)) return;
  float meanVel = (concentricSamples > 0) ? concentricSum / concentricSamples : 0;
  float rom = abs(positionZ - startPosConcentric);
  
  RepResult rep = {++repCount, meanVel, concentricPeak, rom, (uint16_t)dur, true, (uint32_t)millis()};
  if (rom < MIN_ROM || meanVel < MIN_MEAN_VELOCITY) rep.valid = false;
  
  lastRepTime = millis();
  emitRep(rep);
}

void emitRep(RepResult &rep) {
  if (!rep.valid) { repCount--; Serial.println(">>> INVALID REP"); return; }
  Serial.print(">>> REP "); Serial.print(rep.repNumber);
  Serial.print(" | MCV: "); Serial.print(rep.meanVelocity, 3);
  Serial.print(" | ROM: "); Serial.print(rep.rom * 100.0, 1); Serial.println(" cm");
}
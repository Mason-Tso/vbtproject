#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h>

BMI270 imu;
Adafruit_Madgwick filter;

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
// TIMING
// ==========================
unsigned long lastTime;

// ==========================
// CALIBRATION STATE
// ==========================
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

const float STILL_ACC_THRESH  = 0.15;
const float STILL_GYRO_THRESH = 0.05;
const int   REQUIRED_SAMPLES  = 1000;

// ==========================
// VARIANCE STILLNESS
// ==========================
#define WINDOW_SIZE 15

float linZBuffer[WINDOW_SIZE];
float gyroBuffer[WINDOW_SIZE];

int  bufferIndex = 0;
bool bufferFull  = false;

const float LINZ_VAR_THRESH = 0.004;
const float GYRO_VAR_THRESH = 0.0004;

unsigned long stillStart = 0;

float linZVar = 0;
float gyroVar = 0;
float stillStrength = 0;

// Adaptive stillness confirmation
unsigned long stillConfirmAdaptiveStart = 0;
// ==========================
// VELOCITY
// ==========================
float velocityZ               = 0;
float prevLinearZ             = 0;
float velocityFiltered_global = 0;

// ==========================
// POSITION
// ==========================
float positionZ     = 0;
float prevVelocityZ = 0;

// ==========================
// BUTTERWORTH
// ==========================
float v0 = 0, v1 = 0, v2 = 0;

const float b0 =  0.0028981946;
const float b1 =  0.0057963892;
const float b2 =  0.0028981946;
const float a1 = -1.8226949;
const float a2 =  0.8371817;

// ==========================
// STATE MACHINE
// ==========================
enum LiftState {
  STATE_IDLE,
  STATE_UNRACKING,
  STATE_ECCENTRIC,
  STATE_CONCENTRIC,
  STATE_FAULT
};

LiftState     currentState   = STATE_IDLE;
unsigned long stateEnterTime = 0;
uint8_t       repCount       = 0;

int8_t concentricSign = 0;

float         concentricSum      = 0;
float         concentricPeak     = 0;
int           concentricSamples  = 0;
unsigned long concentricStart    = 0;
float         startPosConcentric = 0;


float prevSignedVel = 0;

// Concentric entry confirmation
unsigned long concentricCandidateStart  = 0;
const unsigned long CONCENTRIC_CONFIRM_MS = 60;

// Still confirmation inside concentric (top detection)
unsigned long stillConfirmStart        = 0;
const unsigned long STILL_CONFIRM_MS   = 80;

// Rep cooldown
unsigned long lastRepTime           = 0;
const unsigned long REP_COOLDOWN_MS = 500;

// Thresholds
const float VEL_THRESHOLD_CONCENTRIC = 0.08;
const float VEL_ACCUMULATE_MIN       = 0.05;
const float MIN_ROM                  = 0.10;
const float MAX_PEAK_VELOCITY        = 3.0;
const float MIN_MEAN_VELOCITY        = 0.03;

// Time-based validation
const unsigned long MIN_CONCENTRIC_MS = 100;
const unsigned long MAX_CONCENTRIC_MS = 8000;

// State timeouts
const unsigned long TIMEOUT_UNRACKING  = 10000;
const unsigned long TIMEOUT_ECCENTRIC  = 10000;
const unsigned long TIMEOUT_CONCENTRIC = 8000;

// Unrack
const int UNRACK_ZUPTS_REQUIRED = 1;
int unrackZuptCount = 0;

float confirmedBottomPos = 0;

// Reversal confirmation 
unsigned long reversalStart = 0;
const unsigned long REVERSAL_CONFIRM_MS = 40;
// ==========================
// FORWARD DECLARATIONS
// ==========================
void transitionTo(LiftState newState);
void onEnterIdle();
void onEnterUnracking();
void onEnterEccentric();
void onEnterConcentric();
void onExitConcentric(bool byReversal);
void onEnterFault();
void emitRep(RepResult &rep);

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (imu.beginI2C() != BMI2_OK) {
    Serial.println("BMI270 init failed!");
    while (1);
  }

  filter.begin(800);
  filter.setBeta(0.5);

  lastTime = micros();
  Serial.println("Starting calibrated pipeline...");
  Serial.println("timestamp,linearZ,velocity,position,state");
}

// ==========================
// READ IMU
// ==========================
ImuData imu_read() {
  imu.getSensorData();
  ImuData d;
  d.ax = imu.data.accelX;
  d.ay = imu.data.accelY;
  d.az = imu.data.accelZ;
  d.gx = imu.data.gyroX * PI / 180.0;
  d.gy = imu.data.gyroY * PI / 180.0;
  d.gz = imu.data.gyroZ * PI / 180.0;
  return d;
}

// ==========================
// VARIANCE
// ==========================
float computeVariance(float *buf) {
  float mean = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) mean += buf[i];
  mean /= WINDOW_SIZE;

  float variance = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    float d = buf[i] - mean;
    variance += d * d;
  }
  return variance / WINDOW_SIZE;
}

// ==========================
// BUFFER UPDATE
// ==========================
void updateStillnessBuffers(float linZ, float gyroMag) {
  linZBuffer[bufferIndex] = linZ;
  gyroBuffer[bufferIndex] = gyroMag;
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
  if (bufferIndex == 0) bufferFull = true;
}

// ==========================
// BUTTERWORTH
// ==========================
float butterworth(float input) {
  v0 = input - a1 * v1 - a2 * v2;
  float output = b0 * v0 + b1 * v1 + b2 * v2;
  v2 = v1;
  v1 = v0;
  return output;
}

// ==========================
// STATE TRANSITIONS
// ==========================
void transitionTo(LiftState newState) {
  if (currentState == STATE_CONCENTRIC) {
    onExitConcentric(false);
  }

  currentState   = newState;
  stateEnterTime = millis();

  if      (newState == STATE_IDLE)       onEnterIdle();
  else if (newState == STATE_UNRACKING)  onEnterUnracking();
  else if (newState == STATE_ECCENTRIC)  onEnterEccentric();
  else if (newState == STATE_CONCENTRIC) onEnterConcentric();
  else if (newState == STATE_FAULT)      onEnterFault();
}

void onEnterIdle() {
  concentricSign = 0;
  Serial.println(">>> STATE: IDLE");
}

void onEnterUnracking() {
  unrackZuptCount = 0;
  Serial.println(">>> STATE: UNRACKING");
}

void onEnterEccentric() {
  prevSignedVel            = 0;
  concentricCandidateStart = 0;
  Serial.println(">>> STATE: ECCENTRIC");
}

void onEnterConcentric() {
  concentricSum      = 0;
  concentricPeak     = 0;
  concentricSamples  = 0;
  concentricStart    = millis();
  startPosConcentric = positionZ;
  stillConfirmStart  = 0;
  concentricSign     = (velocityFiltered_global > 0) ? 1 : -1;
  Serial.println(">>> STATE: CONCENTRIC");
}

void onExitConcentric(bool byReversal) {
  unsigned long dur = millis() - concentricStart;

  if (dur < MIN_CONCENTRIC_MS) return;
  if (millis() - lastRepTime < REP_COOLDOWN_MS) return;

  float meanVel = (concentricSamples > 0)
                  ? concentricSum / concentricSamples
                  : 0;
  float rom = abs(positionZ - startPosConcentric);

  RepResult rep;
  rep.repNumber    = ++repCount;
  rep.meanVelocity = meanVel;
  rep.peakVelocity = concentricPeak;
  rep.rom          = rom;
  rep.durationMs   = (uint16_t)dur;
  rep.timestamp    = millis();

  rep.valid = true;
  if (rom < MIN_ROM)                      rep.valid = false;
  if (meanVel < MIN_MEAN_VELOCITY)        rep.valid = false;
  if (concentricPeak > MAX_PEAK_VELOCITY) rep.valid = false;
  if (dur > MAX_CONCENTRIC_MS)            rep.valid = false;

  lastRepTime = millis();
  emitRep(rep);
}

void onEnterFault() {
  Serial.println(">>> STATE: FAULT");
}

// ==========================
// EMIT REP
// ==========================
void emitRep(RepResult &rep) {
  if (!rep.valid) {
    repCount--;
    Serial.print(">>> REP INVALID — discarded (rep ");
    Serial.print(rep.repNumber);
    Serial.println(")");
    return;
  }

  Serial.print(">>> REP ");
  Serial.print(rep.repNumber);
  Serial.print(" | MCV: ");
  Serial.print(rep.meanVelocity, 3);
  Serial.print(" m/s | Peak: ");
  Serial.print(rep.peakVelocity, 3);
  Serial.print(" m/s | ROM: ");
  Serial.print(rep.rom * 100.0, 1);
  Serial.print(" cm | Dur: ");
  Serial.print(rep.durationMs);
  Serial.println(" ms");
}

// ==========================
// LOOP
// ==========================
void loop() {

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.01) dt = 0.00125;

  ImuData raw = imu_read();

  // --------------------------
  // GYRO CALIBRATION
  // --------------------------
  if (!gyroCalibrated) {
    float gyroMag = sqrt(raw.gx*raw.gx + raw.gy*raw.gy + raw.gz*raw.gz);
    if (gyroMag < 0.05) {
      gyroBiasX += raw.gx;
      gyroBiasY += raw.gy;
      gyroBiasZ += raw.gz;
      gyroSamples++;
    }
    if (gyroSamples > 1000) {
      gyroBiasX /= gyroSamples;
      gyroBiasY /= gyroSamples;
      gyroBiasZ /= gyroSamples;
      gyroCalibrated = true;
      Serial.println("Gyro calibrated");
    }
    return;
  }

  // APPLY GYRO BIAS
  float gx = raw.gx - gyroBiasX;
  float gy = raw.gy - gyroBiasY;
  float gz = raw.gz - gyroBiasZ;

  // NORMALIZE ACCEL
  float ax_n = raw.ax, ay_n = raw.ay, az_n = raw.az;
  float norm = sqrt(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
  if (norm > 0) { ax_n /= norm; ay_n /= norm; az_n /= norm; }

  // UPDATE FILTER
  filter.updateIMU(gx, gy, gz, ax_n, ay_n, az_n);

  // GET QUATERNION
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  float dq = abs(qw-last_qw) + abs(qx-last_qx) +
             abs(qy-last_qy) + abs(qz-last_qz);
  last_qw = qw; last_qx = qx;
  last_qy = qy; last_qz = qz;

  // WORLD Z
  float az_world =
      (2*qx*qz - 2*qw*qy) * raw.ax +
      (2*qy*qz + 2*qw*qx) * raw.ay +
      (1 - 2*qx*qx - 2*qy*qy) * raw.az;

  float accelMag = sqrt(raw.ax*raw.ax + raw.ay*raw.ay + raw.az*raw.az);
  float gyroMag  = sqrt(gx*gx + gy*gy + gz*gz);

  bool systemStill_basic =
      gyroMag < STILL_GYRO_THRESH &&
      abs(accelMag - 1.0) < STILL_ACC_THRESH;

  // ORIENTATION LOCK
  if (!orientationReady) {
    bool gravityAligned = abs(az_world) > 0.90;
    bool converged = dq < 0.0008 && systemStill_basic && gravityAligned;
    if (converged) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart > 300) {
        orientationReady = true;
        Serial.println("Orientation converged");
      }
    } else {
      stableStart = 0;
    }
    return;
  }

  // GRAVITY CALIBRATION
  if (!gravityReady) {
    if (systemStill_basic) {
      gravityAccum += az_world;
      gravitySamples++;
      if (gravitySamples >= REQUIRED_SAMPLES) {
        restingGravity = gravityAccum / gravitySamples;
        gravityReady   = true;
        Serial.println("Gravity calibrated");
        transitionTo(STATE_IDLE);
      }
    }
    return;
  }

  // LINEAR ACCEL
  float linearZ = (az_world - restingGravity) * 9.81;

  // STILLNESS
  updateStillnessBuffers(linearZ, gyroMag);

  bool systemStill = false;
  if (bufferFull) {
    linZVar = computeVariance(linZBuffer);
    gyroVar = computeVariance(gyroBuffer);

    // compute once per loop (bonus improvement)
    stillStrength =
        (linZVar / LINZ_VAR_THRESH) +
        (gyroVar / GYRO_VAR_THRESH);

    bool stillCandidate = (linZVar < LINZ_VAR_THRESH) &&
                          (gyroVar < GYRO_VAR_THRESH) &&
                          (abs(linearZ) < 0.2);

    if (stillCandidate) {
      if (stillStart == 0) stillStart = millis();
      if (millis() - stillStart > 50) systemStill = true;
    } else {
      stillStart = 0;
    }
  }

  // GRAVITY CORRECTION
  if (systemStill) {
    restingGravity = 0.9995 * restingGravity + 0.0005 * az_world;
  }

  // VELOCITY + ZUPT
  if (systemStill) {
    velocityZ     = 0;
    prevLinearZ   = 0;
    v0 = v1 = v2 = 0;
    prevVelocityZ = 0;
  } else {
    velocityZ += 0.5 * (prevLinearZ + linearZ) * dt;
    prevLinearZ = linearZ;
  }

  float velocityFiltered  = butterworth(velocityZ);
  velocityFiltered_global = velocityFiltered;

  if (systemStill && abs(velocityFiltered) < 0.04) {
    velocityFiltered        = 0;
    velocityFiltered_global = 0;
  }

  // POSITION
  if (!systemStill) {
    positionZ += 0.5 * (prevVelocityZ + velocityFiltered) * dt;
    prevVelocityZ = velocityFiltered;
  }

  // --------------------------
  // STATE MACHINE
  // --------------------------

  // FAULT recovery
  if (currentState == STATE_FAULT) {
    if (systemStill && millis() - stateEnterTime > 3000) {
      transitionTo(STATE_IDLE);
    }
    return;
  }

  // Timeout guards
  if (currentState == STATE_UNRACKING &&
      millis() - stateEnterTime > TIMEOUT_UNRACKING) {
    transitionTo(STATE_FAULT); return;
  }
  if (currentState == STATE_ECCENTRIC &&
      millis() - stateEnterTime > TIMEOUT_ECCENTRIC) {
    transitionTo(STATE_FAULT); return;
  }
  if (currentState == STATE_CONCENTRIC &&
      millis() - stateEnterTime > TIMEOUT_CONCENTRIC) {
    transitionTo(STATE_FAULT); return;
  }

  // --- IDLE ---
  if (currentState == STATE_IDLE) {
    if (!systemStill) {
      transitionTo(STATE_UNRACKING);
    }
  }

  // --- UNRACKING ---
  else if (currentState == STATE_UNRACKING) {
    if (systemStill) {
      unrackZuptCount++;
      if (unrackZuptCount >= UNRACK_ZUPTS_REQUIRED) {
        positionZ = 0;
        transitionTo(STATE_ECCENTRIC);
        Serial.println(">>> Unrack complete, ready");
      }
    }
  }

  // --- ECCENTRIC ---
  else if (currentState == STATE_ECCENTRIC) {
    if (systemStill) {
      positionZ                = 0;
      confirmedBottomPos       = 0;
      prevVelocityZ            = 0;
      concentricCandidateStart = 0; // reset on stillness
    } else {
      float signedVel = (concentricSign != 0)
                        ? concentricSign * velocityFiltered
                        : velocityFiltered;

      if (signedVel > VEL_THRESHOLD_CONCENTRIC) {
        // start or continue confirmation window
        if (concentricCandidateStart == 0)
          concentricCandidateStart = millis();

        if (millis() - concentricCandidateStart > CONCENTRIC_CONFIRM_MS) {
          concentricCandidateStart = 0;
          transitionTo(STATE_CONCENTRIC);
        }
      } else {
        // velocity dropped back — reset candidate
        concentricCandidateStart = 0;
      }
    }
  }

  // --- CONCENTRIC ---
  else if (currentState == STATE_CONCENTRIC) {

    float signedVel = concentricSign * velocityFiltered;

    // --------------------------
    // 1. STILLNESS CONFIRMATION (paused reps)
    // --------------------------
    if (systemStill) {

    bool veryStill  = (stillStrength < 0.5) && (abs(linearZ) < 0.1);
    bool kindaStill = (stillStrength < 1.0);

    if (!kindaStill) {
        stillConfirmStart = 0;
    }

    unsigned long confirmTime;

    if (veryStill)       confirmTime = 30;
    else if (kindaStill) confirmTime = 80;
    else                 confirmTime = 9999;

    if (stillConfirmStart == 0) stillConfirmStart = millis();

    if (millis() - stillConfirmStart > confirmTime) {
        stillConfirmStart = 0;
        reversalStart = 0;

        transitionTo(STATE_ECCENTRIC);
    }
    }

    // --------------------------
    // 2. REVERSAL CONFIRMATION (touch-and-go reps)  <-- NEW FIX
    // --------------------------
    else if (signedVel < -VEL_THRESHOLD_CONCENTRIC) {

      // start reversal candidate
      if (reversalStart == 0)
        reversalStart = millis();

      // confirm reversal persists
      float currentROM = abs(positionZ - startPosConcentric);

    if ((millis() - reversalStart > REVERSAL_CONFIRM_MS) &&
        (millis() - concentricStart > MIN_CONCENTRIC_MS) &&
        (currentROM > MIN_ROM)) {

        stillConfirmStart = 0;

        // finalize rep
        onExitConcentric(true);

        // reset for next rep
        concentricSum      = 0;
        concentricPeak     = 0;
        concentricSamples  = 0;
        concentricStart    = millis();
        startPosConcentric = positionZ;

        reversalStart = 0;

        // transition cleanly
        currentState   = STATE_ECCENTRIC;
        stateEnterTime = millis();
        onEnterEccentric();
      }
    }

    // --------------------------
    // 3. NORMAL MOTION (no end)
    // --------------------------
    else {

      // cancel confirmations if motion resumes
      stillConfirmStart = 0;
      reversalStart     = 0;

      // accumulate velocity
      if (signedVel > VEL_ACCUMULATE_MIN) {
        concentricSum += signedVel;
        concentricSamples++;

        if (signedVel > concentricPeak)
          concentricPeak = signedVel;
      }
    }

    prevSignedVel = signedVel;
  }
 
    
  // --------------------------
  // DEBUG CSV
  // --------------------------
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5) {
    lastPrint = millis();
    Serial.print(millis());            Serial.print(",");
    Serial.print(linearZ, 4);          Serial.print(",");
    Serial.print(velocityFiltered, 4); Serial.print(",");
    Serial.print(positionZ, 4);        Serial.print(",");

    switch (currentState) {
      case STATE_IDLE:       Serial.println("IDLE");       break;
      case STATE_UNRACKING:  Serial.println("UNRACKING");  break;
      case STATE_ECCENTRIC:  Serial.println("ECCENTRIC");  break;
      case STATE_CONCENTRIC: Serial.println("CONCENTRIC"); break;
      case STATE_FAULT:      Serial.println("FAULT");      break;
      default:               Serial.println("UNKNOWN");    break;
    }
  }
}

#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// ================================================================
// ===                       CALIBRATION                        ===
// ================================================================
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
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

// ================================================================
// ===                   PHYSICS & FILTERING                    ===
// ================================================================
static float linearZFiltered = 0;
float alpha = 0.15;
float velDecay = 0.995;

float velocity = 0;
float position = 0;

// ================================================================
// ===                   STATE MACHINE SETUP                    ===
// ================================================================
enum State { SEARCH_BOTTOM, SEARCH_TOP };
State currentState = SEARCH_BOTTOM; 

int repCount = 0;
float currentROMGate = 0.10;
float velocityDeadzone = 0.03;
bool wasDescending = false;
bool wasAscending = false;

// ================================================================
// ===          TOP STABILITY CONFIRMATION + ZUPT (ADDED)        ===
// ================================================================
// TUNE: these are "near rest" thresholds used ONLY at the top
float VEL_STABLE_THRESH = 0.03;       // m/s (near zero velocity)
float ACC_STABLE_THRESH = 0.40;       // m/s^2 (near zero accel)
unsigned long TOP_STABLE_MS = 150;    // ms stable required to confirm top

unsigned long topStableStart = 0;     // 0 means not currently stable
bool topStabilityPrinted = false;     // print once per stability attempt

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
  Serial.println("PHASE 1: Calibrating gyro (5s)...");
  startCalTime = millis();

  filter.begin(100);
  filter.setBeta(0.15);
}

void loop() {
  if (!imu.dataReady()) return;
  imu.getAGMT();

  // --- GYRO CALIBRATION ---
  if (!calibrated) {
    gyroBiasX += imu.gyrX(); gyroBiasY += imu.gyrY(); gyroBiasZ += imu.gyrZ();
    
    float ax = imu.accX() * 0.00981;
    float ay = imu.accY() * 0.00981;
    float az = imu.accZ() * 0.00981;
    
    filter.updateIMU(imu.gyrX()*PI/180, imu.gyrY()*PI/180, imu.gyrZ()*PI/180, ax, ay, az);
    
    if (millis() - startCalTime >= 5000) {
      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;
      calibrated = true;
      lastTime = micros();
      Serial.println("Gyro calibration complete.");
      Serial.println("PHASE 2: Waiting for orientation lock...");
    }
    sampleCount++;
    return;
  }

  // --- TIMING ---
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // --- SENSOR UPDATE ---
  float ax = imu.accX() * 0.00981;
  float ay = imu.accY() * 0.00981;
  float az = imu.accZ() * 0.00981;
  
  float gx = (imu.gyrX() - gyroBiasX) * PI / 180.0;
  float gy = (imu.gyrY() - gyroBiasY) * PI / 180.0;
  float gz = (imu.gyrZ() - gyroBiasZ) * PI / 180.0;
  
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  
  float az_world = (2*qx*qz - 2*qw*qy) * ax + 
                   (2*qy*qz + 2*qw*qx) * ay + 
                   (1 - 2*qx*qx - 2*qy*qy) * az;

  // --- ORIENTATION & GRAVITY CALIBRATION ---
  if (!orientationReady || !gravityCalibrated) {
    float accelMag = sqrt(ax*ax + ay*ay + az*az);
    
    if (abs(accelMag - 9.81) < 0.4) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart > 500) {
        if(!orientationReady) {
          orientationReady = true;
          Serial.println("Orientation locked.");
          Serial.println("PHASE 3: Sampling resting gravity...");
        }
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
          Serial.println("Gravity sampled.");
          Serial.print("Resting Gravity: "); Serial.println(restingGravity);
          Serial.println("SYSTEM READY. Start lifting (Expects Descent First).");
       }
    }
    return;
  }

  // ========================
  // PHASE 4: PHYSICS
  // ========================
  float linearZ = az_world - restingGravity;
  linearZFiltered += alpha * (linearZ - linearZFiltered);

  velocity += linearZFiltered * dt;
  velocity *= velDecay;
  position += velocity * dt;

  // --- Direction Classification ---
  if (velocity < -velocityDeadzone) wasDescending = true;
  if (velocity > velocityDeadzone)  wasAscending  = true;

  // ================================================================
  // SEARCHING FOR BOTTOM (UNCHANGED)
  // ================================================================
  if (currentState == SEARCH_BOTTOM) {

    if (position < -currentROMGate &&
        wasDescending &&
        velocity > velocityDeadzone) {

      Serial.print("--- BOTTOM DETECTED --- Distance: ");
      Serial.println(abs(position));

      velocity = 0;
      position = 0;

      wasDescending = false;
      wasAscending = false;

      // Reset TOP stability tracking when we enter SEARCH_TOP
      topStableStart = 0;
      topStabilityPrinted = false;

      currentState = SEARCH_TOP;
    }
  }

  // ================================================================
  // SEARCHING FOR TOP (SAME GATES + STABILITY + ZUPT)
  // ================================================================
  else if (currentState == SEARCH_TOP) {

    // Keep the same "we only care after real upward travel" conditions:
    //   position > currentROMGate  AND  wasAscending
    if (position > currentROMGate && wasAscending) {

      // Stability check (near rest) at the top
      bool nearRest =
        (abs(velocity) < VEL_STABLE_THRESH) &&
        (abs(linearZFiltered) < ACC_STABLE_THRESH);

      if (nearRest) {

        if (topStableStart == 0) {
          topStableStart = millis();

          // Print once when stability tracking starts
          Serial.println(">>> TOP STABILITY START");
          topStabilityPrinted = true;
        }

        // If we've been stable long enough -> CONFIRM TOP + ZUPT
        if (millis() - topStableStart > TOP_STABLE_MS) {

          repCount++;
          float finalROM = abs(position);
          currentROMGate = finalROM * 0.8;

          Serial.print("--- TOP CONFIRMED (STABLE) --- Rep: ");
          Serial.println(repCount);

          // ZUPT at TOP (safe now because we required stability)
          velocity = 0;
          position = 0;

          wasDescending = false;
          wasAscending = false;

          topStableStart = 0;
          topStabilityPrinted = false;

          currentState = SEARCH_BOTTOM;
        }
      }
      else {
        // If we were trying to stabilize but motion resumed, reset timer
        if (topStableStart != 0) {
          Serial.println(">>> TOP STABILITY RESET (Motion Resumed)");
        }
        topStableStart = 0;
        topStabilityPrinted = false;
      }
    }
    else {
      // If we haven't met the top gates yet, ensure timer stays reset
      topStableStart = 0;
      topStabilityPrinted = false;
    }
  }

  Serial.print(millis());
  Serial.print(",");
  Serial.print(linearZFiltered);
  Serial.print(",");
  Serial.print(velocity);
  Serial.print(",");
  Serial.println(position);
}
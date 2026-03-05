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
float alpha = 0.15;     // Accel filter
float velDecay = 0.995; // Velocity High-Pass (bleeds drift over time)

float velocity = 0;
float position = 0;

// ================================================================
// ===                   STATE MACHINE SETUP                    ===
// ================================================================
enum State { SEARCH_BOTTOM, SEARCH_TOP };
State currentState = SEARCH_BOTTOM; 

int repCount = 0;
float currentROMGate = 0.25; 
float velocityDeadzone = 0.05; // Soft crossing threshold (m/s)

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
  
  // Transform to World Z
  float az_world = (2*qx*qz - 2*qw*qy) * ax + 
                   (2*qy*qz + 2*qw*qx) * ay + 
                   (1 - 2*qx*qx - 2*qy*qy) * az;

  // --- ORIENTATION & GRAVITY CALIBRATION FLAGS ---
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
  velocity *= velDecay; // Bleed off drift
  position += velocity * dt;

  // ================================================================
  // ===                 PHASE 5: SOFT-CROSSING TOGGLE            ===
  // ================================================================

  if (currentState == SEARCH_BOTTOM) {
    // Condition: Traveled down past gate AND velocity is no longer significantly negative
    if (position < -currentROMGate && velocity > -velocityDeadzone) {
      Serial.print("--- BOTTOM DETECTED --- Distance: "); Serial.println(abs(position));
      
      velocity = 0; // ZUPT
      position = 0; 
      currentState = SEARCH_TOP;
    }
  } 

  else if (currentState == SEARCH_TOP) {
    // Condition: Traveled up past gate AND velocity is no longer significantly positive
    if (position > currentROMGate && velocity < velocityDeadzone) {
      repCount++;
      float finalROM = abs(position);

      // Calibrate Gate after Rep 1
      if (repCount == 1) { 
        currentROMGate = finalROM * 0.75; 
        Serial.print("ROM Gate Calibrated to: "); Serial.println(currentROMGate);
      }

      Serial.print("--- TOP DETECTED --- Rep: "); Serial.print(repCount);
    
      velocity = 0; // ZUPT
      position = 0;
      currentState = SEARCH_BOTTOM;
    }
  }
}
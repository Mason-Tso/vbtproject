#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0

ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// ========================
// Gyro bias
// ========================
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

bool calibrated = false;
bool orientationReady = false;

unsigned long startCalTime;
unsigned long lastTime;
unsigned long stableStart = 0;

// ========================
// EMA variables
// ========================
float linearZFiltered = 0;
float alpha = 0.1;   // Acceleration smoothing factor
// ========================

// ========================
// Velocity Variables
// ========================
float velocity = 0;
float prevVelocity = 0;
// ========================

// ========================
// STILLNESS ZUPT VARIABLES (TOP RESET)
// ========================
float accelStillThreshold = 0.15;    // m/s²
int stillTimeRequired = 100;         // ms
unsigned long stillStartTime = 0;
// ========================


// ======================================================
// ===== ADDED FOR HYBRID BOTTOM DETECTION (OPTION 4) ====
// ======================================================

// Movement states
enum MovementState {
  STATE_IDLE,
  STATE_DESCENT,
  STATE_CONCENTRIC
};

MovementState state = STATE_IDLE;

// Thresholds (TUNE THESE)
float descentThreshold = 0.05;       // detect downward motion
float smallVelBand = 0.02;           // near-zero band
float accelUpThreshold = 1.0;        // upward accel needed
float concentricLockoutVel = 0.1;    // hysteresis lockout

// ======================================================


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
  filter.setBeta(0.1);   
}

void loop() {

  if (!imu.dataReady()) return;

  imu.getAGMT();

  // ========================
  // GYRO CALIBRATION
  // ========================
  if (!calibrated) {

    static int sampleCount = 0;

    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();
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
  // NORMAL OPERATION
  // ========================

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

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
  // MADGWICK CONVERGENCE CHECK
  // ========================
  if (!orientationReady) {

    if (abs(az_world - 9.81) < 0.5) {

      if (stableStart == 0)
        stableStart = millis();

      if (millis() - stableStart > 200) {
        orientationReady = true;
        Serial.println("Orientation locked. System ready.");
      }

    } else {
      stableStart = 0;
    }

    return;
  }

  // ========================
  // LINEAR ACCELERATION
  // ========================

  float linearZ = 9.81 - az_world;

  // EMA filtering
  linearZFiltered += alpha * (linearZ - linearZFiltered);

  // Save previous velocity
  prevVelocity = velocity;

  // Integrate acceleration
  velocity += linearZFiltered * dt;


  // ======================================================
  // ===== HYBRID BOTTOM DETECTION STATE MACHINE ==========
  // ======================================================

  switch(state) {

    case STATE_IDLE:

      // Detect start of descent
      if (velocity < -descentThreshold) {
        state = STATE_DESCENT;
      }

      break;


    case STATE_DESCENT:

      // Hybrid bottom detection:
      // 1) Was descending
      // 2) Velocity near zero
      // 3) Strong upward acceleration

      if (
        prevVelocity < -descentThreshold &&
        velocity >= -smallVelBand &&
        linearZFiltered > accelUpThreshold
      ) {
        velocity = 0;                 // HARD CLAMP
        state = STATE_CONCENTRIC;

        Serial.println("BOTTOM (HYBRID)");
      }

      break;


    case STATE_CONCENTRIC:

      // Hysteresis lockout
      // Prevents bounce re-triggering

      if (velocity > concentricLockoutVel) {
        // solid upward motion confirmed
      }

      // Detect next descent
      if (velocity < -descentThreshold) {
        state = STATE_DESCENT;
      }

      break;
  }


  // ======================================================
  // STILLNESS-BASED ZUPT (TOP RESET)
  // ======================================================

  if (abs(linearZFiltered) < accelStillThreshold) {

    if (stillStartTime == 0)
      stillStartTime = millis();

    if (millis() - stillStartTime > stillTimeRequired) {
      velocity = 0;
      state = STATE_IDLE;   // Reset to idle at top
      Serial.println("STILL ZUPT (TOP)");
    }

  } else {
    stillStartTime = 0;
  }

  Serial.println(velocity);
}
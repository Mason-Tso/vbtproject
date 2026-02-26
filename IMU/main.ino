#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_AHRS.h>

#define AD0_VAL 0
ICM_20948_I2C imu;
Adafruit_Madgwick filter;

// --- RE-TUNED PARAMETERS ---
const float ACCEL_THRESHOLD = 0.20;      // Lowered to detect slower starts
const float START_MOVE_THRESHOLD = 0.4;  // Lowered to catch the start of the lift
const float MIN_REP_VELOCITY = 0.15;     // Lowered to catch "grinder" reps
const float EMA_ALPHA = 0.2;             

// --- VARIABLES ---
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool calibrated = false;
unsigned long startCalTime, lastTime, repStartTime;

float velocityZ = 0;
float peakVelocity = 0;
float sumVelocity = 0;  // For calculating Mean
int sampleCount = 0;    // For calculating Mean
float lastFilteredAz = 0;

enum State { IDLE, CONCENTRIC, RESETTING };
State deviceState = IDLE;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) { delay(500); }
  startCalTime = millis();
  filter.begin(100); 
  filter.setBeta(0.1); 
}

void loop() {
  if (!imu.dataReady()) return;
  imu.getAGMT();

  if (!calibrated) {
    calibrateGyro();
    return;
  }

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01; 
  lastTime = currentTime;

  float ax = imu.accX() * 9.80665;
  float ay = imu.accY() * 9.80665;
  float az = imu.accZ() * 9.80665;
  float gx = (imu.gyrX() - gyroBiasX) * PI / 180.0;
  float gy = (imu.gyrY() - gyroBiasY) * PI / 180.0;
  float gz = (imu.gyrZ() - gyroBiasZ) * PI / 180.0;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  float az_world = (2*qx*qz - 2*qw*qy) * ax + (2*qy*qz + 2*qw*qx) * ay + (1 - 2*qx*qx - 2*qy*qy) * az;
  float linearZ = az_world - 9.80665;

  float filteredAz = (EMA_ALPHA * linearZ) + (1.0 - EMA_ALPHA) * lastFilteredAz;
  lastFilteredAz = filteredAz;

  processVBT(filteredAz, dt);
}

void processVBT(float az, float dt) {
  // ZUPT
  if (abs(az) < ACCEL_THRESHOLD && deviceState != CONCENTRIC) {
    velocityZ = 0; 
    if (deviceState == RESETTING) deviceState = IDLE;
  } else {
    velocityZ += az * dt;
  }

  switch (deviceState) {
    case IDLE:
      if (az > START_MOVE_THRESHOLD) {
        deviceState = CONCENTRIC;
        peakVelocity = 0;
        sumVelocity = 0;
        sampleCount = 0;
        repStartTime = millis();
      }
      break;

    case CONCENTRIC:
      // Accumulate velocity for Mean calculation
      if (velocityZ > 0) {
        sumVelocity += velocityZ;
        sampleCount++;
        if (velocityZ > peakVelocity) peakVelocity = velocityZ;
      }
      
      // End rep if velocity drops or time exceeds a realistic rep (e.g. 5s)
      if (velocityZ <= 0.05 || (millis() - repStartTime > 5000)) { 
        if (peakVelocity >= MIN_REP_VELOCITY && sampleCount > 5) { 
          float meanVelocity = sumVelocity / sampleCount;
          
          Serial.println("---------- REP DETECTED ----------");
          Serial.print("MEAN VELOCITY: "); Serial.print(meanVelocity); Serial.println(" m/s");
          Serial.print("PEAK VELOCITY: "); Serial.print(peakVelocity); Serial.println(" m/s");
          Serial.println("----------------------------------");
        }
        deviceState = RESETTING;
      }
      break;

    case RESETTING:
      // Wait for ZUPT to clear the velocity buffer
      break;
  }
}

void calibrateGyro() {
  static int samples = 0;
  gyroBiasX += imu.gyrX();
  gyroBiasY += imu.gyrY();
  gyroBiasZ += imu.gyrZ();
  samples++;
  if (millis() - startCalTime >= 5000) {
    gyroBiasX /= samples; gyroBiasY /= samples; gyroBiasZ /= samples;
    calibrated = true;
    Serial.println("System Ready. Lift now.");
  }
}
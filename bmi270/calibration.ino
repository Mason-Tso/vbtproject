#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h>

BMI270 imu;
Adafruit_Madgwick filter;

// ==========================
// STRUCT
// ==========================
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
};

// ==========================
// TIMING
// ==========================
unsigned long lastTime;

// ==========================
// CALIBRATION STATE
// ==========================
bool gyroCalibrated = false;
int gyroSamples = 0;
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

bool orientationReady = false;
bool gravityReady = false;

float restingGravity = 0;
float gravityAccum = 0;
int gravitySamples = 0;

unsigned long stableStart = 0;

// Quaternion tracking
float last_qw = 1, last_qx = 0, last_qy = 0, last_qz = 0;

// tuning
const float STILL_ACC_THRESH = 0.1;
const float STILL_GYRO_THRESH = 0.05;
const int   REQUIRED_SAMPLES = 1000;

// ==========================
// VARIANCE STILLNESS
// ==========================
#define WINDOW_SIZE 15

float linZBuffer[WINDOW_SIZE];
float gyroBuffer[WINDOW_SIZE];

int bufferIndex = 0;
bool bufferFull = false;

// variance thresholds (squared std)
const float LINZ_VAR_THRESH  = 0.0025;    // (0.05)^2
const float GYRO_VAR_THRESH  = 0.000225;  // (0.015)^2

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
// VARIANCE (NO SQRT)
// ==========================
float computeVariance(float *buf) {
  float mean = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    mean += buf[i];
  }
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

  // --------------------------
  // APPLY GYRO BIAS
  // --------------------------
  float gx = raw.gx - gyroBiasX;
  float gy = raw.gy - gyroBiasY;
  float gz = raw.gz - gyroBiasZ;

  // --------------------------
  // NORMALIZE ACCEL
  // --------------------------
  float ax_n = raw.ax;
  float ay_n = raw.ay;
  float az_n = raw.az;

  float norm = sqrt(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
  if (norm > 0) {
    ax_n /= norm;
    ay_n /= norm;
    az_n /= norm;
  }

  // --------------------------
  // UPDATE FILTER
  // --------------------------
  filter.updateIMU(gx, gy, gz, ax_n, ay_n, az_n);

  // --------------------------
  // GET QUATERNION
  // --------------------------
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  float dq =
    abs(qw - last_qw) +
    abs(qx - last_qx) +
    abs(qy - last_qy) +
    abs(qz - last_qz);

  last_qw = qw;
  last_qx = qx;
  last_qy = qy;
  last_qz = qz;

  // --------------------------
  // WORLD Z
  // --------------------------
  float az_world =
      (2*qx*qz - 2*qw*qy) * raw.ax +
      (2*qy*qz + 2*qw*qx) * raw.ay +
      (1 - 2*qx*qx - 2*qy*qy) * raw.az;

  float accelMag = sqrt(raw.ax*raw.ax + raw.ay*raw.ay + raw.az*raw.az);
  float gyroMag  = sqrt(gx*gx + gy*gy + gz*gz);

  bool systemStill_basic =
      gyroMag < STILL_GYRO_THRESH &&
      abs(accelMag - 1.0) < STILL_ACC_THRESH;

  // --------------------------
  // ORIENTATION LOCK
  // --------------------------
  if (!orientationReady) {

    bool gravityAligned = abs(az_world) > 0.90;

    bool converged =
        dq < 0.0008 &&
        systemStill_basic &&
        gravityAligned;

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

  // --------------------------
  // GRAVITY CALIBRATION
  // --------------------------
  if (!gravityReady) {

    if (systemStill_basic) {
      gravityAccum += az_world;
      gravitySamples++;

      if (gravitySamples >= REQUIRED_SAMPLES) {
        restingGravity = gravityAccum / gravitySamples;
        gravityReady = true;
        Serial.println("Gravity calibrated");
      }
    }

    return;
  }

  // --------------------------
  // LINEAR ACCELERATION
  // --------------------------
  float linearZ = (az_world - restingGravity) * 9.81;

  // --------------------------
  // VARIANCE STILLNESS
  // --------------------------
  updateStillnessBuffers(linearZ, gyroMag);

  bool systemStill = false;

  if (bufferFull) {
    float linZVar = computeVariance(linZBuffer);
    float gyroVar = computeVariance(gyroBuffer);

    systemStill = (linZVar < LINZ_VAR_THRESH) &&
              (gyroVar < GYRO_VAR_THRESH) &&
              (abs(linearZ) < 0.2);
  }

  // --------------------------
  // CONTINUOUS GRAVITY CORRECTION
  // --------------------------
  if (systemStill) {
    restingGravity = 0.999 * restingGravity + 0.001 * az_world;
  }

  // --------------------------
  // DEBUG
  // --------------------------
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 50) {
    lastPrint = millis();

    Serial.print("linZ: ");
    Serial.print(linearZ, 3);
    Serial.print(" | still: ");
    Serial.print(systemStill);
    Serial.print(" | gravity: ");
    Serial.println(restingGravity, 3);
  }
}

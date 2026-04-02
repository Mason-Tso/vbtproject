#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_AHRS.h>

BMI270 imu;
Adafruit_Madgwick filter;

// Portable struct
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
};

unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (imu.beginI2C() != BMI2_OK) {
    Serial.println("BMI270 init failed!");
    while (1);
  }

  filter.begin(200);
  filter.setBeta(0.2);

  lastTime = micros();

  Serial.println("Starting split pipeline test...");
}

// Read raw data ONLY (no normalization here)
ImuData imu_read() {
  imu.getSensorData();

  ImuData d;

  d.ax = imu.data.accelX;  // raw (g)
  d.ay = imu.data.accelY;
  d.az = imu.data.accelZ;

  d.gx = imu.data.gyroX * PI / 180.0;
  d.gy = imu.data.gyroY * PI / 180.0;
  d.gz = imu.data.gyroZ * PI / 180.0;

  return d;
}

void loop() {

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  ImuData raw = imu_read();

  // 🔹 CREATE NORMALIZED COPY (for Madgwick ONLY)
  float ax_n = raw.ax;
  float ay_n = raw.ay;
  float az_n = raw.az;

  float norm = sqrt(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
  if (norm > 0) {
    ax_n /= norm;
    ay_n /= norm;
    az_n /= norm;
  }

  // 🔹 Madgwick uses NORMALIZED accel
  filter.updateIMU(
    raw.gx, raw.gy, raw.gz,
    ax_n, ay_n, az_n
  );

  // Quaternion
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  // 🔹 Use RAW accel for physics transform
  float az_world =
      (2*qx*qz - 2*qw*qy) * raw.ax +
      (2*qy*qz + 2*qw*qx) * raw.ay +
      (1 - 2*qx*qx - 2*qy*qy) * raw.az;

  // 🔹 Print BOTH raw and transformed
  static int count = 0;
  count++;

  if (count % 10 == 0) {
    Serial.print(" | worldZ: ");
    Serial.println(az_world, 3);
  }
}
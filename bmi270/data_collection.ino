#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22); // SDA, SCL

  Serial.println("Initializing BMI270...");

  if (imu.beginI2C() != BMI2_OK) {
    Serial.println("BMI270 init FAILED");
    while (1);
  }

  Serial.println("BMI270 ready!");
}

void loop() {
  imu.getSensorData();

  // Acceleration (m/s^2)
  Serial.print("Accel [m/s^2]  X: ");
  Serial.print(imu.data.accelX, 3);
  Serial.print(" Y: ");
  Serial.print(imu.data.accelY, 3);
  Serial.print(" Z: ");
  Serial.print(imu.data.accelZ, 3);

  // Gyro (deg/s)
  Serial.print("   Gyro [deg/s]  X: ");
  Serial.print(imu.data.gyroX, 3);
  Serial.print(" Y: ");
  Serial.print(imu.data.gyroY, 3);
  Serial.print(" Z: ");
  Serial.println(imu.data.gyroZ, 3);

  delay(100);
}
#include <Wire.h>
#include "ICM_20948.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0

ICM_20948_I2C imu;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

bool calibrated = false;

unsigned long startCalTime;
unsigned long lastTime;
float dt;

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  imu.begin(WIRE_PORT, AD0_VAL);

  while (imu.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.println("IMU not detected...");
    delay(500);
  }

  SERIAL_PORT.println("Keep device completely still...");
  SERIAL_PORT.println("Calibrating gyro for 5 seconds...");

  startCalTime = millis();
}

void loop()
{
  if (!imu.dataReady()) return;

  imu.getAGMT();

  // =========================
  // GYRO CALIBRATION PHASE
  // =========================
  if (!calibrated)
  {
    gyroBiasX += imu.gyrX();
    gyroBiasY += imu.gyrY();
    gyroBiasZ += imu.gyrZ();

    static int sampleCount = 0;
    sampleCount++;

    if (millis() - startCalTime >= 5000)
    {
      gyroBiasX /= sampleCount;
      gyroBiasY /= sampleCount;
      gyroBiasZ /= sampleCount;

      calibrated = true;

      SERIAL_PORT.println("Gyro calibration complete.");
      SERIAL_PORT.print("Bias X: "); SERIAL_PORT.println(gyroBiasX);
      SERIAL_PORT.print("Bias Y: "); SERIAL_PORT.println(gyroBiasY);
      SERIAL_PORT.print("Bias Z: "); SERIAL_PORT.println(gyroBiasZ);

      lastTime = micros();
    }

    return;
  }

  // =========================
  // NORMAL OPERATION
  // =========================

  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  float gx = imu.gyrX() - gyroBiasX;
  float gy = imu.gyrY() - gyroBiasY;
  float gz = imu.gyrZ() - gyroBiasZ;

  SERIAL_PORT.print("Gyro (DPS): ");
  SERIAL_PORT.print(gx); SERIAL_PORT.print(", ");
  SERIAL_PORT.print(gy); SERIAL_PORT.print(", ");
  SERIAL_PORT.println(gz);
}
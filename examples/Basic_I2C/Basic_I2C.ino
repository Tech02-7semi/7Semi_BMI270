/**
 * ------------------------------------------------------------
 * File       : Basic_I2C.ino
 * Device     : BMI270 IMU
 * Platform   : ESP32
 * Interface  : I2C
 * Library    : 7Semi_BMI270
 *
 * Description
 * -----------
 * This example shows how to:
 *  - Initialize the BMI270 IMU over I2C
 *  - Configure accelerometer and gyroscope
 *  - Read acceleration, angular velocity, and temperature
 *  - Print sensor data to the Serial Monitor
 *
 * Wiring (ESP32)
 * --------------
 *  BMI270  ->  ESP32
 *  ------------------
 *  VIN     ->  3.3V
 *  GND     ->  GND
 *  SDA     ->  GPIO 21
 *  SCL     ->  GPIO 22
 *
 * I2C Address
 * -----------
 *  - 0x69 (default when SDO is HIGH)
 * ------------------------------------------------------------
 */

#include <7Semi_BMI270.h>

BMI270_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);

  /* ------------------------------------------------------------
   * BMI270 I2C Configuration
   * ------------------------------------------------------------ */
  BMI270_7Semi::Config cfg;

  cfg.bus  = BMI270_7Semi::Bus::I2C;  // Select I2C interface
  cfg.addr = 0x69;                    // BMI270 I2C address

  // ESP32 I2C pin selection
  cfg.sda   = 21;                    // I2C SDA Pin
  cfg.scl   = 22;                    // I2C SCL Pin
  cfg.i2cHz = 400000;                // I2C speed: 400 kHz

  // Initialize BMI270
  if (!imu.begin(cfg)) {
    Serial.println("ERROR: BMI270 initialization failed!");
    while (1) delay(100);             // Stop execution
  }

  /* ============================================================
   *                  SENSOR CONFIGURATION
   * ============================================================
   *
   * Key terms:
   * ----------
   * ODR   : Output Data Rate (how often data updates)
   * Range : Maximum measurable value
   * BW    : Noise vs latency tradeoff
   * Perf  : Power saving vs performance
   */

  /* ---------------- Accelerometer Configuration ----------------
   *
   * ODR Options (Hz):
   *  BMI2_ACC_ODR_0_78HZ   → ultra low power
   *  BMI2_ACC_ODR_1_56HZ
   *  BMI2_ACC_ODR_3_12HZ
   *  BMI2_ACC_ODR_6_25HZ
   *  BMI2_ACC_ODR_12_5HZ
   *  BMI2_ACC_ODR_25HZ
   *  BMI2_ACC_ODR_50HZ
   *  BMI2_ACC_ODR_100HZ   → recommended default
   *  BMI2_ACC_ODR_200HZ
   *  BMI2_ACC_ODR_400HZ
   *  BMI2_ACC_ODR_800HZ
   *  BMI2_ACC_ODR_1600HZ  → highest speed
   *
   * Range:
   *  BMI2_ACC_RANGE_2G    → (default)
   *  BMI2_ACC_RANGE_4G
   *  BMI2_ACC_RANGE_8G
   *  BMI2_ACC_RANGE_16G   → large motion
   *
   * Bandwidth (noise vs latency):
   *  BMI2_ACC_OSR4_AVG1   → lowest latency
   *  BMI2_ACC_OSR2_AVG2
   *  BMI2_ACC_NORMAL_AVG4 → recommended default
   *  BMI2_ACC_CIC_AVG8
   *  BMI2_ACC_RES_AVG16
   *  BMI2_ACC_RES_AVG32
   *  BMI2_ACC_RES_AVG64
   *  BMI2_ACC_RES_AVG128  → lowest noise
   */

  imu.setAccelConfig(
    BMI2_ACC_ODR_100HZ,        // Output Data Rate
    BMI2_ACC_RANGE_2G,         // Measurement range
    BMI2_ACC_NORMAL_AVG4,      // Filter / bandwidth
    BMI2_PERF_OPT_MODE         // Performance optimized
  );

  /* ---------------- Gyroscope Configuration ----------------
   *
   * ODR (Hz):
   *  BMI2_GYR_ODR_25HZ
   *  BMI2_GYR_ODR_50HZ
   *  BMI2_GYR_ODR_100HZ   → recommended default
   *  BMI2_GYR_ODR_200HZ
   *  BMI2_GYR_ODR_400HZ
   *  BMI2_GYR_ODR_800HZ
   *  BMI2_GYR_ODR_1600HZ
   *  BMI2_GYR_ODR_3200HZ
   *
   * Range (deg/s):
   *  BMI2_GYR_RANGE_125
   *  BMI2_GYR_RANGE_250
   *  BMI2_GYR_RANGE_500
   *  BMI2_GYR_RANGE_1000
   *  BMI2_GYR_RANGE_2000 → fast motion
   *
   * Bandwidth:
   *  BMI2_GYR_OSR4_MODE   → lowest latency
   *  BMI2_GYR_OSR2_MODE
   *  BMI2_GYR_NORMAL_MODE → recommended default
   */

  imu.setGyroConfig(
    BMI2_GYR_ODR_100HZ,        // Output Data Rate
    BMI2_GYR_RANGE_2000,       // Angular velocity range
    BMI2_GYR_NORMAL_MODE,      // Filter / bandwidth
    BMI2_PERF_OPT_MODE         // Performance optimized
  );

  Serial.println("BMI270 initialized successfully\n");
}
void loop() {

  float ax, ay, az;   // Accelerometer values (g)
  float gx, gy, gz;   // Gyroscope values (deg/s)
  float temperature;  // Temperature (°C)

  // Read accelerometer
  if (imu.readAccel(ax, ay, az)) {
    Serial.print("ACC (g): ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.println(az, 3);
  }

  // Read gyroscope
  if (imu.readGyro(gx, gy, gz)) {
    Serial.print("GYR (dps): ");
    Serial.print(gx, 3); Serial.print(", ");
    Serial.print(gy, 3); Serial.print(", ");
    Serial.println(gz, 3);
  }

  // Read temperature
  if (imu.readTemperatureC(temperature)) {
    Serial.print("TEMP (°C): ");
    Serial.println(temperature, 2);
  }
  
  Serial.println();
  delay(10);   // ~100 Hz loop rate
}

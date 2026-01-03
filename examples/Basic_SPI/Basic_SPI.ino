/**
 * ------------------------------------------------------------
 * File       : Basic_SPI.ino
 * Device     : BMI270 IMU
 * Platform   : ESP32
 * Interface  : SPI
 * Library    : 7Semi_BMI270
 *
 * Description
 * -----------
 * This example demonstrates how to bring up the BMI270 IMU
 * using SPI on an ESP32.
 *
 * Important SPI Notes (BMI270)
 * ----------------------------
 * - BMI270 selects SPI or I2C at POWER-UP
 * - CS must be held LOW during power-up to force SPI mode
 * - SPI mode: MODE 0
 * - Max SPI clock:
 *     - Init  : ≤ 1 MHz (recommended)
 *     - Normal: ≤ 10 MHz (after init)
 *
 * ESP32 VSPI Wiring
 * -----------------
 *  BMI270  ->  ESP32
 *  ------------------
 *  CS      ->  GPIO 5
 *  SCK     ->  GPIO 18
 *  MISO    ->  GPIO 19
 *  MOSI    ->  GPIO 23
 *  VCC     ->  3.3V
 *  GND     ->  GND
 * ------------------------------------------------------------
 */

#include <7Semi_BMI270.h>

constexpr int PIN_CS   = 5;    // Chip Select (CS)

/* ESP32 VSPI default pins */
constexpr int PIN_SCK  = 18;
constexpr int PIN_MISO = 19;
constexpr int PIN_MOSI = 23;

SPIClass spi(VSPI);          // Use ESP32 VSPI peripheral
BMI270_7Semi imu;            // BMI270 IMU instance

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n[BMI270 SPI Bring-Up | ESP32]");

  /* ------------------------------------------------------------
   * Force SPI mode at power-up
   *
   * BMI270 samples CS during boot:
   *  - CS LOW  → SPI mode
   *  - CS HIGH → I2C mode
   * ------------------------------------------------------------ */
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, LOW);     // Hold CS LOW
  delay(50);                     // Ensure stable level
  digitalWrite(PIN_CS, HIGH);    // Release CS
  delay(100);                    // BMI270 power-up time

  /* ------------------------------------------------------------
   * BMI270 SPI Configuration
   * ------------------------------------------------------------ */
  BMI270_7Semi::Config cfg;

  cfg.bus  = BMI270_7Semi::Bus::SPI;   // REQUIRED for SPI
  cfg.spi  = &spi;                     // SPI peripheral
  cfg.cs   = PIN_CS;                   // Chip Select pin

  cfg.sck  = PIN_SCK;
  cfg.miso = PIN_MISO;
  cfg.mosi = PIN_MOSI;

  cfg.spiHz = 1 * 1000 * 1000;          // 1 MHz (safe init speed)

  /* ------------------------------------------------------------
   * Initialize BMI270
   * ------------------------------------------------------------ */
  if (!imu.begin(cfg)) {
    Serial.println("ERROR: BMI270 SPI initialization failed");

    uint8_t err;
    imu.getErrorStatus(err);
    Serial.print("Driver error status: ");
    Serial.println(err);

    // Attempt to read CHIP_ID anyway (debug aid)
    uint8_t id = imu.chipId();
    Serial.print("CHIP_ID read = 0x");
    Serial.println(id, HEX);

    while (true) delay(1000);
  }

  // Accelerometer:
  //  - 100 Hz update rate
  //  - ±2g range (best resolution)
  //  - Normal bandwidth (balanced noise/latency)
  imu.setAccelConfig(
    BMI2_ACC_ODR_100HZ,
    BMI2_ACC_RANGE_2G,
    BMI2_ACC_NORMAL_AVG4,
    BMI2_PERF_OPT_MODE
  );

  // Gyroscope:
  //  - 100 Hz update rate
  //  - ±2000 dps range
  //  - Normal bandwidth
  imu.setGyroConfig(
    BMI2_GYR_ODR_100HZ,
    BMI2_GYR_RANGE_2000,
    BMI2_GYR_NORMAL_MODE,
    BMI2_PERF_OPT_MODE
  );

  Serial.println("BMI270 initialized successfully (SPI)");
}
void loop() {

  float ax, ay, az;     // Accelerometer (g)
  float gx, gy, gz;     // Gyroscope (deg/s)
  float temperatureC;   // Temperature (°C)

  // Read accelerometer
  if (imu.readAccel(ax, ay, az)) {
    Serial.print("ACC (g): ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.print(az, 3);
    Serial.print(" | ");
  }

  // Read gyroscope
  if (imu.readGyro(gx, gy, gz)) {
    Serial.print("GYR (dps): ");
    Serial.print(gx, 2); Serial.print(", ");
    Serial.print(gy, 2); Serial.print(", ");
    Serial.print(gz, 2);
    Serial.print(" | ");
  }

  // Read temperature
  if (imu.readTemperatureC(temperatureC) == BMI2_OK) {
    Serial.print("TEMP (°C): ");
    Serial.print(temperatureC, 2);
  }

  Serial.println();
  delay(1000);
}

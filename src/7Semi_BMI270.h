#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

extern "C" {
#include "bmi2.h"
#include "bmi270.h"
}

/**
 * 7Semi BMI270 Arduino Library (I2C + SPI)
 *
 * Copyright (c) 2025 7Semi
 *
 * This library is a thin Arduino/C++ wrapper around the official Bosch Sensortec
 * BMI2/BMI270 driver (C API). It provides Arduino-friendly initialization and
 * helper functions while delegating the sensor register/feature handling to the
 * Bosch driver.
 *
 * Bosch Sensortec notice (important):
 * - This library uses the Bosch Sensortec BMI2/BMI270 driver source and headers
 *   (e.g., bmi2.h, bmi270.h and corresponding .c files) which are copyrighted by
 *   Bosch Sensortec GmbH.
 * - The Bosch driver is distributed under the BSD 3-Clause license (as provided
 *   by Bosch). MUST keep Bosch’s copyright notice, license text, and
 *   disclaimers intact in any redistributed Bosch source/header files.
 * - This wrapper does not replace or modify Bosch’s license terms for the Bosch
 *   driver; it only adds Arduino integration around it.
 */

/**
 * BMI270_7Semi
 *
 * - Arduino-friendly wrapper for BMI270 using Bosch Sensortec BMI2/BMI270 API
 * - Supports I2C and SPI
 * - Exposes common configuration, FIFO, interrupts, calibration/FOC and feature
 *   helper calls
 *
 * Notes:
 * - Most functions return Bosch driver status codes (BMI2_OK on success).
 * - For advanced/feature APIs, make sure the correct BMI270 config file/feature
 *   sets are used per Bosch documentation.
 */
class BMI270_7Semi {
public:
  /** - Bus selection for device communication */
  enum class Bus : uint8_t {
    I2C = 0,
    SPI = 1
  };

  /**
   * Config
   * - Select bus and pass bus settings
   * - Defaults match common BMI270 modules
   */
  struct Config {
    Bus bus = Bus::I2C;

    /** - I2C bus instance and address (0x68 or 0x69 depending on SA0) */
    TwoWire* i2c = &Wire;
    uint8_t addr = 0x68;

    /** - Optional custom pins (only for platforms that support pin remap) */
    int8_t sda = -1;
    int8_t scl = -1;

    /** - I2C clock */
    uint32_t i2cHz = 400000;

    /** - SPI bus instance and chip select pin */
    SPIClass* spi = &SPI;
    uint8_t cs = 5;

    /** - Optional custom pins (only for platforms that support pin remap) */
    int8_t sck = -1;
    int8_t miso = -1;
    int8_t mosi = -1;

    /** - SPI clock */
    uint32_t spiHz = 10000000;
  };

  BMI270_7Semi();

  /**
   * begin
   * - Initializes selected bus (I2C/SPI)
   * - Wires Bosch callbacks to Arduino bus functions
   * - Runs BMI270 init sequence and caches chip id
   */
  bool begin(const Config& cfg);

  /**
   * end
   * - Stops/ends bus where applicable
   * - Does not guarantee sensor power-down (board dependent)
   */
  void end();

  /** - Returns chip id from successful initialization */
  uint8_t chipId() const {
    return chip_id;
  }

  /** - Soft reset device and return status */
  int8_t softReset();

  /** - Reads device status byte (Bosch-defined bits) */
  int8_t getStatus(uint8_t& status);

  int8_t getErrorStatus(uint8_t& status);

  /** - Enables or disables accelerometer in BMI2 power control */
  int8_t enableAccel(bool enable);

  /** - Enables or disables gyroscope in BMI2 power control */
  int8_t enableGyro(bool enable);

  /** - Enables or disables temperature sensor */
  int8_t enableTemp(bool enable);

  /**
   * setAdvancedPowerSave
   * - Toggles APS mode
   * - Some modes/ODRs/features may have restrictions under APS
   */
  int8_t setPowerSave(bool enable);

  /** - Reads current APS mode state */
  int8_t getPowerSave(bool& enable);

  /** - Writes accelerometer configuration (range/odr/bw/filter) */
  int8_t setAccelConfig(uint8_t odr, uint8_t range, uint8_t bwp, uint8_t filter_perf);

  /** - Reads accelerometer configuration (range/odr/bw/filter) */
  int8_t getAccelConfig(uint8_t& odr, uint8_t& range, uint8_t& bwp, uint8_t& filter_perf);

  /** - Writes gyroscope configuration (range/odr/bw/filter) */
  int8_t setGyroConfig(uint8_t odr, uint8_t range, uint8_t bwp, uint8_t filter_perf);
  /** - Reads gyroscope configuration (range/odr/bw/filter) */
  int8_t getGyroConfig(uint8_t& odr, uint8_t& range, uint8_t& bwp, uint8_t& filter_perf);

  /** - Change accel ODR */
  int8_t setAccelOdr(uint8_t odr);

  /** - Read accel ODR */
  int8_t getAccelOdr(uint8_t& odr);

  /** -  Change gyro ODR */
  int8_t setGyroOdr(uint8_t odr);

  /** - Read gyro ODR */
  int8_t getGyroOdr(uint8_t& odr);

  /**
   * readSample
   * - Reads latest sensor data into bmi2_sens_data struct
   * - Which fields are valid depends on enabled sensors
   */
  int8_t readSample(bmi2_sens_data& s);

  /** - Reads raw temperature value (Bosch scaling) */
  int8_t readTemperatureRaw(int16_t& t);

  /** - Reads temperature and converts to degrees C */
  int8_t readTemperatureC(float& c);

  /**
   * readGyro
   * - Reads gyro and converts to dps using current gyro range
   */
  bool readGyro(float& gx_dps, float& gy_dps, float& gz_dps);

  /**
   * readAccel
   * - Reads accel and converts to g using current accel range
   */
  bool readAccel(float& ax_g, float& ay_g, float& az_g);

  /**
   * fifoEnable
   * - Enables FIFO streams and optional header mode
   * - Header mode is recommended when mixing multiple streams
   */
  int8_t fifoEnable(bool accel, bool gyro, bool aux, bool temp, bool header = true);

  /** - Disables all FIFO streams */
  int8_t fifoDisableAll();

  /** - Flushes FIFO content */
  int8_t fifoFlush();

  /** - Sets FIFO watermark level */
  int8_t fifoSetWatermark(uint16_t wm);

  /** - Reads FIFO fill level in bytes */
  int8_t fifoGetLength(uint16_t& bytes);

  /**
   * fifoRead
   * - Reads FIFO bytes into user buffer
   * - Updates fifo frame object for extraction functions
   */
  int8_t fifoRead(bmi2_fifo_frame& fifo, uint8_t* buffer, uint16_t bufferLen);

  /** - Extracts accel frames from FIFO buffer into output array */
  int8_t fifoExtractAccel(bmi2_sens_axes_data* out, uint16_t& frames, bmi2_fifo_frame& fifo);

  /** - Extracts gyro frames from FIFO buffer into output array */
  int8_t fifoExtractGyro(bmi2_sens_axes_data* out, uint16_t& frames, bmi2_fifo_frame& fifo);

  /** - Extracts AUX frames from FIFO buffer into output array */
  int8_t fifoExtractAux(bmi2_aux_fifo_data* out, uint16_t& frames, bmi2_fifo_frame& fifo);

  /** - Configures FIFO downsampling for selected sensor stream */
  int8_t fifoSetDownsampling(uint8_t sensor, uint8_t down);

  /** - Reads FIFO downsampling configuration for selected sensor stream */
  int8_t fifoGetDownsampling(uint8_t sensor, uint8_t& down);

  /** - Enables/disables FIFO self wake-up behavior (Bosch feature) */
  int8_t fifoSetSelfWakeup(bool enable);

  /** - Reads FIFO self wake-up configuration */
  int8_t fifoGetSelfWakeup(bool& enable);

  /** - Maps data interrupt sources to selected hardware pin */
  int8_t mapDataInterrupt(uint8_t data_int_mask, bmi2_hw_int_pin pin);

  /** - Maps feature interrupt configs to hardware interrupt routing */
  int8_t mapFeatureInterrupt(const bmi2_sens_int_config* cfg, uint8_t n);

  /** - Reads interrupt status bitmask */
  int8_t getInterruptStatus(uint16_t& int_status);

  /** - Enables/disables accelerometer offset compensation */
  int8_t enableAccelOffset(bool enable);

  /** - Reads accelerometer offset registers (raw LSB values) */
  int8_t getAccelOffset(int8_t& x, int8_t& y, int8_t& z);

  /**
   * performAccelFOC
   * - Runs Bosch accel fast offset calibration
   * - Sensor must be stationary in expected orientation
   */
  int8_t performAccelFOC(int8_t x, int8_t y, int8_t z);

  /** - Writes gyro offset registers (manual offsets) */
  int8_t setGyroOffset(int16_t x, int16_t y, int16_t z);

  /** - Enables/disables gyro offset compensation */
  int8_t setGyroOffsetComp(bool enable);

  /** - Reads gyro offset compensation state */
  int8_t getGyroOffsetComp(bool& enabled);

  /** - Alternate gyro offset setter using Bosch axis struct */
  int8_t setGyroOffset(const bmi2_sens_axes_data& off);

  /**
   * performGyroFOC
   * - Runs Bosch gyro fast offset calibration
   * - Sensor must be stationary during calibration
   */
  int8_t performGyroFOC();

  /** - Writes axis remap configuration for board orientation alignment */
  int8_t setRemap(const bmi2_remap& r);

  /** - Reads current axis remap configuration */
  int8_t getRemap(bmi2_remap& r);

  /** - Writes AUX interface configuration */
  int8_t setAuxConfig(const bmi2_aux_config& cfg);

  /** - Reads AUX interface configuration */
  int8_t getAuxConfig(bmi2_aux_config& cfg);

  /** - Reads register(s) from AUX connected device */
  int8_t auxRead(uint8_t reg, uint8_t* data, uint16_t len);

  /** - Writes register(s) to AUX connected device */
  int8_t auxWrite(uint8_t reg, const uint8_t* data, uint16_t len);

  /** - Runs gyro CRT/self-test sequence through Bosch API */
  int8_t gyroCRT(uint8_t gyro_st_crt, bmi2_gyro_self_test_status& out);

  /** - Aborts gyro CRT/self-test sequence */
  int8_t abortGyroCRT();

  /** - Enables BMI270 features using Bosch feature ID list */
  int8_t enableFeatures(const uint8_t* featList, uint8_t n);

  /** - Disables BMI270 features using Bosch feature ID list */
  int8_t disableFeatures(const uint8_t* featList, uint8_t n);

  /** - Reads step counter output (feature must be enabled) */
  int8_t readStepCount(uint32_t& steps);

  /** - Resets step counter */
  int8_t resetStepCount();

  /** - Reads activity output (Bosch-defined value) */
  int8_t readStepActivity(uint8_t& activity);

  /** - Reads wrist gesture output (Bosch-defined value) */
  int8_t readWristGesture(uint8_t& gesture);

  /** - Programs NVM (only if supported by device/variant) */
  int8_t nvmProgram();

private:
  /**
   * BusConfig
   * - Stored bus context for Bosch callbacks
   * - Keeps callback interface simple and stable
   */
  struct BusConfig {
    Bus bus = Bus::I2C;

    TwoWire* i2c = nullptr;
    uint8_t addr = 0;

    SPIClass* spi = nullptr;
    uint8_t cs = 0;
    uint32_t spiHz = 0;
  };

  BusConfig busConfig;

  /** - Bosch device struct holding callbacks and runtime state */
  bmi2_dev bme270;

  /** - Cached sample storage for convenience paths */
  bmi2_sens_data sens_data;

  /** - Cached chip id after init */
  uint8_t chip_id = 0;

  /** - Cached last Bosch status for debug */
  int8_t last_status = BMI2_OK;

  /**
   * readRegs
   * - Raw register read helper
   */
  int8_t readRegs(uint8_t reg, uint8_t* data, uint16_t len);

  /**
   * writeRegs
   * - Raw register write helper
   */
  int8_t writeRegs(uint8_t reg, const uint8_t* data, uint16_t len);

  /**
   * read
   * - Bosch driver register read hook
   * - Routes register reads to I2C/SPI based on stored busConfig
   */
  static int8_t read(uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr);

  /**
   * write
   * - Bosch driver register write hook
   * - Routes register writes to I2C/SPI based on stored busConfig
   */
  static int8_t write(uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr);

  /**
   * delayUs
   * - Bosch driver delay hook
   * - Used for init timing and internal wait requirements
   */
  static void delayUs(uint32_t period, void* intf_ptr);

  /**
   * initBMI270
   * - Bosch init sequence wrapper
   * - Loads BMI270 configuration and verifies communication
   */
  bool initBMI270();

  /** - Returns accel scale (LSB per g) for configured range */
  float accLsbPerG_(uint8_t range) const;

  /** - Returns gyro scale (LSB per dps) for configured range */
  float gyrLsbPerDps_(uint8_t range) const;
};
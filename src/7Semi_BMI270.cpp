#include "7Semi_BMI270.h"

// ================== Constructor ==================
/**
- Create driver object
- Clears bus config + Bosch device struct + cached sens data
*/
BMI270_7Semi::BMI270_7Semi() {
  memset(&busConfig, 0, sizeof(busConfig));
  memset(&bme270, 0, sizeof(bme270));
  memset(&sens_data, 0, sizeof(sens_data));
}

// ================== Begin ==================
/**
- Initialize bus and start BMI270
- cfg.bus : selects I2C or SPI
- cfg.i2c/cfg.spi : bus instance (optional)
- cfg.addr/cfg.cs : I2C address / SPI chip select
- cfg.sda/scl/sck/miso/mosi : optional pin remap (platform dependent)
- return : true if init ok
*/
bool BMI270_7Semi::begin(const Config& cfg) {
  busConfig.bus = cfg.bus;

  if (cfg.bus == Bus::I2C) {
    busConfig.i2c = cfg.i2c ? cfg.i2c : &Wire;
    busConfig.addr = cfg.addr;

    // ----- I2C begin (pins only where supported) -----
    if ((cfg.sda >= 0) && (cfg.scl >= 0)) {
#if defined(ESP32)
      busConfig.i2c->begin((int)cfg.sda, (int)cfg.scl);
#else
      busConfig.i2c->begin();
#endif
    } else {
      busConfig.i2c->begin();
    }

    // ----- I2C clock -----
    busConfig.i2c->setClock(cfg.i2cHz);
  } else {
    busConfig.spi = cfg.spi ? cfg.spi : &SPI;
    busConfig.cs = cfg.cs;
    busConfig.spiHz = cfg.spiHz;

    // ----- CS idle high -----
    pinMode(busConfig.cs, OUTPUT);
    digitalWrite(busConfig.cs, HIGH);

    // ----- SPI begin (pins only where supported) -----
    if ((cfg.sck >= 0) && (cfg.miso >= 0) && (cfg.mosi >= 0)) {
#if defined(ESP32)
      busConfig.spi->begin((int)cfg.sck, (int)cfg.miso, (int)cfg.mosi, -1);
#else
      busConfig.spi->begin();
#endif
    } else {
      busConfig.spi->begin();
    }
  }

  return initBMI270();
}

// ================== End ==================
/**
- End wrapper
- Bosch driver has no deinit
- User may end Wire/SPI if required
*/
void BMI270_7Semi::end() {
  /** Nothing to de-init for Bosch driver. User can end Wire/SPI if required. */
}

// ================== RAW register access ==================
/**
- Read N bytes from register address
- reg : start register
- data : out buffer
- len : number of bytes
- return : Bosch status
*/
int8_t BMI270_7Semi::readRegs(uint8_t reg, uint8_t* data, uint16_t len) {
  last_status = bmi2_get_regs(reg, data, len, &bme270);
  return last_status;
}

/**
- Write N bytes to register address
- reg : start register
- data : input buffer
- len : number of bytes
- return : Bosch status
*/
int8_t BMI270_7Semi::writeRegs(uint8_t reg, const uint8_t* data, uint16_t len) {
  last_status = bmi2_set_regs(reg, data, len, &bme270);
  return last_status;
}

// ================== Reset / status ==================
/**
- Issue soft reset
- return : Bosch status
*/
int8_t BMI270_7Semi::softReset() {
  last_status = bmi2_soft_reset(&bme270);
  return last_status;
}

/**
- Read status register bits
- status : out status
- return : Bosch status
*/
int8_t BMI270_7Semi::getStatus(uint8_t& status) {
  last_status = bmi2_get_status(&status, &bme270);
  return last_status;
}

/**
- Read internal status / error bits
- istat : out internal status
- return : Bosch status
*/
int8_t BMI270_7Semi::getErrorStatus(uint8_t& istat) {
  last_status = bmi2_get_internal_status(&istat, &bme270);
  return last_status;
}

// ================== Enable accel / gyro / temp ==================
/**
- Enable/disable accelerometer (PWR_CTRL bit)
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::enableAccel(bool enable) {
  uint8_t reg_data[1] = { 0 };
  if (enable)
    reg_data[0] |= 1 << 2;
  else
    reg_data[0] |= 0 << 2;
  last_status = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, reg_data, 1, &bme270);
  return last_status;
}

/**
- Enable/disable gyroscope (PWR_CTRL bit)
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::enableGyro(bool enable) {
  uint8_t reg_data[1] = { 0 };
  if (enable)
    reg_data[0] |= 1 << 1;
  else
    reg_data[0] |= 0 << 1;
  last_status = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, reg_data, 1, &bme270);
  return last_status;
}

/**
- Enable/disable temperature sensor path (PWR_CTRL bit)
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::enableTemp(bool enable) {
  uint8_t reg_data[1] = { 0 };
  if (enable)
    reg_data[0] |= 1 << 3;
  else
    reg_data[0] |= 0 << 3;
  last_status = bmi2_set_regs(BMI2_PWR_CTRL_ADDR, reg_data, 1, &bme270);
  return last_status;
}

// ================== Power save ==================
/**
- Enable/disable advanced power save
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::setPowerSave(bool enable) {
  last_status = bmi2_set_adv_power_save(enable ? BMI2_ENABLE : BMI2_DISABLE, &bme270);
  return last_status;
}

/**
- Read advanced power save state
- enable : out true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::getPowerSave(bool& enable) {
  uint8_t en = 0;
  last_status = bmi2_get_adv_power_save(&en, &bme270);
  if (last_status == BMI2_OK) enable = (en == BMI2_ENABLE);
  return last_status;
}

// ================== Accel config ==================
/**
- Set accelerometer config using Bosch sensor config
- odr : output data rate setting
- range : accel range setting
- bwp : bandwidth parameter
- filter_perf : performance/lowpower mode selector
- return : Bosch status
*/
int8_t BMI270_7Semi::setAccelConfig(uint8_t odr, uint8_t range, uint8_t bwp, uint8_t filter_perf) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_ACCEL;

  /** Read current first (safer across versions) */
  last_status = bmi2_get_sensor_config(&sc, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  sc.cfg.acc.odr = odr;
  sc.cfg.acc.range = range;
  sc.cfg.acc.bwp = bwp;
  sc.cfg.acc.filter_perf = filter_perf;

  last_status = bmi2_set_sensor_config(&sc, 1, &bme270);
  return last_status;
}

/**
- Read accelerometer config using Bosch sensor config
- odr,range,bwp,filter_perf : out fields
- return : Bosch status
*/
int8_t BMI270_7Semi::getAccelConfig(uint8_t& odr, uint8_t& range, uint8_t& bwp, uint8_t& filter_perf) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_ACCEL;

  last_status = bmi2_get_sensor_config(&sc, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  odr = sc.cfg.acc.odr;
  range = sc.cfg.acc.range;
  bwp = sc.cfg.acc.bwp;
  filter_perf = sc.cfg.acc.filter_perf;

  return last_status;
}

// ================== Gyro config ==================
/**
- Set gyroscope config using Bosch sensor config
- odr : output data rate setting
- range : gyro range setting
- bwp : bandwidth parameter
- filter_perf : performance/lowpower mode selector
- return : Bosch status
*/
int8_t BMI270_7Semi::setGyroConfig(uint8_t odr, uint8_t range, uint8_t bwp, uint8_t filter_perf) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_GYRO;

  last_status = bmi2_get_sensor_config(&sc, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  sc.cfg.gyr.odr = odr;
  sc.cfg.gyr.range = range;
  sc.cfg.gyr.bwp = bwp;
  sc.cfg.gyr.filter_perf = filter_perf;

  last_status = bmi2_set_sensor_config(&sc, 1, &bme270);
  return last_status;
}

/**
- Read gyroscope config using Bosch sensor config
- odr,range,bwp,filter_perf : out fields
- return : Bosch status
*/
int8_t BMI270_7Semi::getGyroConfig(uint8_t& odr, uint8_t& range, uint8_t& bwp, uint8_t& filter_perf) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_GYRO;

  last_status = bmi2_get_sensor_config(&sc, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  odr = sc.cfg.gyr.odr;
  range = sc.cfg.gyr.range;
  bwp = sc.cfg.gyr.bwp;
  filter_perf = sc.cfg.gyr.filter_perf;

  return last_status;
}

// ================== ODR helpers ==================
/**
- Update only accel ODR (keeps other accel fields)
- odr : new odr value
- return : Bosch status
*/
int8_t BMI270_7Semi::setAccelOdr(uint8_t odr) {
  uint8_t cur_odr = 0, range = 0, bwp = 0, perf = 0;

  last_status = getAccelConfig(cur_odr, range, bwp, perf);
  if (last_status != BMI2_OK) return last_status;

  return setAccelConfig(odr, range, bwp, perf);
}

/**
- Read accel ODR only
- odr : out odr value
- return : Bosch status
*/
int8_t BMI270_7Semi::getAccelOdr(uint8_t& odr) {
  uint8_t range = 0, bwp = 0, perf = 0;

  last_status = getAccelConfig(odr, range, bwp, perf);
  return last_status;
}

/**
- Update only gyro ODR (keeps other gyro fields)
- odr : new odr value
- return : Bosch status
*/
int8_t BMI270_7Semi::setGyroOdr(uint8_t odr) {
  uint8_t cur_odr = 0, range = 0, bwp = 0, perf = 0;

  last_status = getGyroConfig(cur_odr, range, bwp, perf);
  if (last_status != BMI2_OK) return last_status;

  return setGyroConfig(odr, range, bwp, perf);
}

/**
- Read gyro ODR only
- odr : out odr value
- return : Bosch status
*/
int8_t BMI270_7Semi::getGyroOdr(uint8_t& odr) {
  uint8_t range = 0, bwp = 0, perf = 0;

  last_status = getGyroConfig(odr, range, bwp, perf);
  return last_status;
}

// ================== Read sample ==================
/**
- Read latest enabled sensor sample
- s : out bmi2_sens_data
- return : Bosch status
*/
int8_t BMI270_7Semi::readSample(bmi2_sens_data& s) {
  // uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  last_status = bmi2_get_sensor_data(&s, &bme270);
  return last_status;
}

// ================== Temperature ==================
/**
- Read temperature raw value
- t : out raw
- return : Bosch status
*/
int8_t BMI270_7Semi::readTemperatureRaw(int16_t& t) {
  last_status = bmi2_get_temperature_data(&t, &bme270);
  return last_status;
}

/**
- Read temperature in degC
- c : out degrees C
- return : Bosch status
*/
int8_t BMI270_7Semi::readTemperatureC(float& c) {
  int16_t raw = 0;
  last_status = readTemperatureRaw(raw);
  if (last_status != BMI2_OK) return false;

  /** BMI2 driver raw conversion as per your tuning */
  c = ((float)((int16_t)raw) / 512.0f) + 20.230f;
  // c = ((float)raw) / 100.0f;
  return true;
}

// ================== FIFO ==================
/**
- Enable FIFO streams
- accel,gyro,aux,temp : stream enable
- header : header mode enable
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoEnable(bool accel, bool gyro, bool aux, bool temp, bool header) {
  uint16_t fifoCfg = 0;

  if (accel) fifoCfg |= BMI2_FIFO_ACC_EN;
  if (gyro) fifoCfg |= BMI2_FIFO_GYR_EN;
  if (aux) fifoCfg |= BMI2_FIFO_AUX_EN;
  // if (temp)  fifoCfg |= BMI2_FIFO_TEMP;

  if (header) fifoCfg |= BMI2_FIFO_HEADER_EN;

  last_status = bmi2_set_fifo_config(fifoCfg, BMI2_ENABLE, &bme270);
  return last_status;
}

/**
- Disable all FIFO streams
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoDisableAll() {
  last_status = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &bme270);
  return last_status;
}

/**
- Flush FIFO buffer
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoFlush() {
  /** BMI2 provides a FIFO flush command via command register */
  last_status = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &bme270);
  return last_status;
}

/**
- Set FIFO watermark
- wm : watermark value
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoSetWatermark(uint16_t wm) {
  last_status = bmi2_set_fifo_wm(wm, &bme270);
  return last_status;
}

/**
- Read FIFO length in bytes
- bytes : out bytes
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoGetLength(uint16_t& bytes) {
  last_status = bmi2_get_fifo_length(&bytes, &bme270);
  return last_status;
}

/**
- Read FIFO bytes into buffer
- fifo : fifo frame struct (updated)
- buffer : raw byte buffer
- bufferLen : buffer size
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoRead(bmi2_fifo_frame& fifo, uint8_t* buffer, uint16_t bufferLen) {
  if (!buffer || !bufferLen) return BMI2_E_NULL_PTR;

  memset(&fifo, 0, sizeof(fifo));
  fifo.data = buffer;
  fifo.length = bufferLen;

  last_status = bmi2_read_fifo_data(&fifo, &bme270);
  return last_status;
}

/**
- Extract accel frames from FIFO
- out : output frames array
- frames : in/out frame count
- fifo : fifo frame struct
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoExtractAccel(bmi2_sens_axes_data* out, uint16_t& frames, bmi2_fifo_frame& fifo) {
  if (!out) return BMI2_E_NULL_PTR;
  last_status = bmi2_extract_accel(out, &frames, &fifo, &bme270);
  return last_status;
}

/**
- Extract gyro frames from FIFO
- out : output frames array
- frames : in/out frame count
- fifo : fifo frame struct
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoExtractGyro(bmi2_sens_axes_data* out, uint16_t& frames, bmi2_fifo_frame& fifo) {
  if (!out) return BMI2_E_NULL_PTR;
  last_status = bmi2_extract_gyro(out, &frames, &fifo, &bme270);
  return last_status;
}

/**
- Extract AUX frames from FIFO
- out : output frames array
- frames : in/out frame count
- fifo : fifo frame struct
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoExtractAux(bmi2_aux_fifo_data* out, uint16_t& frames, bmi2_fifo_frame& fifo) {
  if (!out) return BMI2_E_NULL_PTR;
  last_status = bmi2_extract_aux(out, &frames, &fifo, &bme270);
  return last_status;
}

/**
- Set FIFO downsampling
- sensor : Bosch selector
- down : downsample setting
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoSetDownsampling(uint8_t sensor, uint8_t down) {
  last_status = bmi2_set_fifo_down_sample(sensor, down, &bme270);
  return last_status;
}

/**
- Read FIFO downsampling
- sensor : Bosch selector
- down : out downsample setting
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoGetDownsampling(uint8_t sensor, uint8_t& down) {
  last_status = bmi2_get_fifo_down_sample(sensor, &down, &bme270);
  return last_status;
}

/**
- Enable/disable FIFO self wakeup
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoSetSelfWakeup(bool enable) {
  last_status = bmi2_set_fifo_self_wake_up(enable ? BMI2_ENABLE : BMI2_DISABLE, &bme270);
  return last_status;
}

/**
- Read FIFO self wakeup enable state
- enable : out true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::fifoGetSelfWakeup(bool& enable) {
  uint8_t en = 0;
  last_status = bmi2_get_fifo_self_wake_up(&en, &bme270);
  if (last_status == BMI2_OK) enable = (en == BMI2_ENABLE);
  return last_status;
}

// ================== Interrupts ==================
/**
- Map data interrupt to INT pin
- data_int_mask : Bosch data interrupt mask
- pin : INT1/INT2
- return : Bosch status
*/
int8_t BMI270_7Semi::mapDataInterrupt(uint8_t data_int_mask, bmi2_hw_int_pin pin) {
  last_status = bmi2_map_data_int(data_int_mask, pin, &bme270);
  return last_status;
}

/**
- Map feature interrupts
- cfg : Bosch sens int config array
- n : array length
- return : Bosch status
*/
int8_t BMI270_7Semi::mapFeatureInterrupt(const bmi2_sens_int_config* cfg, uint8_t n) {
  last_status = bmi270_map_feat_int(cfg, n, &bme270);
  return last_status;
}

/**
- Read interrupt status
- int_status : out status mask
- return : Bosch status
*/
int8_t BMI270_7Semi::getInterruptStatus(uint16_t& int_status) {
  last_status = bmi2_get_int_status(&int_status, &bme270);
  return last_status;
}

// ================== Offsets / FOC ==================
/**
- Enable/disable accel offset compensation
- enable : true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::enableAccelOffset(bool enable) {
  last_status = bmi2_set_accel_offset_comp(enable ? BMI2_ENABLE : BMI2_DISABLE, &bme270);
  return last_status;
}

/**
- Read accel offset registers
- x,y,z : out offsets
- return : Bosch status
*/
int8_t BMI270_7Semi::getAccelOffset(int8_t& x, int8_t& y, int8_t& z) {
  uint8_t offset[3] = { 0 };

  last_status = bmi2_get_accel_offset_comp(offset, &bme270);
  if (last_status != BMI2_OK) {
    return last_status;
  }

  x = (int8_t)offset[0];
  y = (int8_t)offset[1];
  z = (int8_t)offset[2];

  return last_status;
}

/**
- Perform accel FOC
- x,y,z : target gravity direction (Bosch-defined)
- return : Bosch status
*/
int8_t BMI270_7Semi::performAccelFOC(int8_t x, int8_t y, int8_t z) {
  // Bosch struct from user arguments
  struct bmi2_accel_foc_g_value gravity;
  gravity.x = x;
  gravity.y = y;
  gravity.z = z;

  last_status = bmi2_perform_accel_foc(&gravity, &bme270);
  return last_status;
}

/**
- Write gyro offset registers
- x,y,z : raw offsets
- return : Bosch status
*/
int8_t BMI270_7Semi::setGyroOffset(int16_t x, int16_t y, int16_t z) {
  bmi2_sens_axes_data offset;

  offset.x = x;
  offset.y = y;
  offset.z = z;

  last_status = bmi2_write_gyro_offset_comp_axes(&offset, &bme270);
  return last_status;
}

/**
- Read gyro offset compensation enable
- enabled : out true/false
- return : Bosch status
*/
int8_t BMI270_7Semi::getGyroOffsetComp(bool& enabled) {
  uint8_t en = 0;
  last_status = bmi2_get_gyro_offset_comp(&en, &bme270);
  if (last_status == BMI2_OK) {
    enabled = (en != 0);
  }
  return last_status;
}

/**
- Perform gyro FOC
- return : Bosch status
*/
int8_t BMI270_7Semi::performGyroFOC() {
  last_status = bmi2_perform_gyro_foc(&bme270);
  return last_status;
}

// ================== Remap ==================
/**
- Set axis remap
- r : remap struct
- return : Bosch status
*/
int8_t BMI270_7Semi::setRemap(const bmi2_remap& r) {
  last_status = bmi2_set_remap_axes(&r, &bme270);
  return last_status;
}

/**
- Read axis remap
- r : out remap struct
- return : Bosch status
*/
int8_t BMI270_7Semi::getRemap(bmi2_remap& r) {
  last_status = bmi2_get_remap_axes(&r, &bme270);
  return last_status;
}

// ================== AUX ==================
/**
- Set AUX configuration
- cfg : aux config struct
- return : Bosch status
*/
int8_t BMI270_7Semi::setAuxConfig(const bmi2_aux_config& cfg) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_AUX;
  sc.cfg.aux = cfg;
  last_status = bmi2_set_sensor_config(&sc, 1, &bme270);
  return last_status;
}

/**
- Read AUX configuration
- cfg : out aux config
- return : Bosch status
*/
int8_t BMI270_7Semi::getAuxConfig(bmi2_aux_config& cfg) {
  bmi2_sens_config sc;
  memset(&sc, 0, sizeof(sc));
  sc.type = BMI2_AUX;
  last_status = bmi2_get_sensor_config(&sc, 1, &bme270);
  if (last_status == BMI2_OK) cfg = sc.cfg.aux;
  return last_status;
}

/**
- AUX manual mode read
- reg : start address
- data : out buffer
- len : number of bytes
- return : Bosch status
*/
int8_t BMI270_7Semi::auxRead(uint8_t reg, uint8_t* data, uint16_t len) {
  last_status = bmi2_read_aux_man_mode(reg, data, len, &bme270);
  return last_status;
}

/**
- AUX manual mode write
- reg : start address
- data : input buffer
- len : number of bytes
- return : Bosch status
*/
int8_t BMI270_7Semi::auxWrite(uint8_t reg, const uint8_t* data, uint16_t len) {
  last_status = bmi2_write_aux_man_mode(reg, data, len, &bme270);
  return last_status;
}

// ================== Gyro CRT ==================
/**
- Run gyro CRT/self-test
- gyro_st_crt : Bosch selector
- out : out status struct
- return : Bosch status
*/
int8_t BMI270_7Semi::gyroCRT(uint8_t gyro_st_crt, bmi2_gyro_self_test_status& out) {
  // NOTE: current implementation calls gyro FOC
  // Keep as-is to match existing behavior
  (void)gyro_st_crt;
  (void)out;
  last_status = bmi2_perform_gyro_foc(&bme270);
  return last_status;
}

/**
- Abort gyro CRT/self-test
- return : Bosch status
*/
int8_t BMI270_7Semi::abortGyroCRT() {
  last_status = bmi2_abort_crt_gyro_st(&bme270);
  return last_status;
}

// ================== Feature engine ==================
/**
- Enable BMI270 features list
- featList : feature id list
- n : number of entries
- return : Bosch status
*/
int8_t BMI270_7Semi::enableFeatures(const uint8_t* featList, uint8_t n) {
  last_status = bmi270_sensor_enable(featList, n, &bme270);
  return last_status;
}

/**
- Disable BMI270 features list
- featList : feature id list
- n : number of entries
- return : Bosch status
*/
int8_t BMI270_7Semi::disableFeatures(const uint8_t* featList, uint8_t n) {
  last_status = bmi270_sensor_disable(featList, n, &bme270);
  return last_status;
}

/**
- Read step counter output
- steps : out step count
- return : Bosch status
*/
int8_t BMI270_7Semi::readStepCount(uint32_t& steps) {
  bmi2_feat_sensor_data fd;
  memset(&fd, 0, sizeof(fd));
  fd.type = BMI2_STEP_COUNTER;

  last_status = bmi270_get_feature_data(&fd, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  steps = fd.sens_data.step_counter_output;
  return last_status;
}

/**
- Reset step counter
- return : Bosch status
*/
int8_t BMI270_7Semi::resetStepCount() {
  /** Workaround: reset by disable/enable step counter feature */
  uint8_t feat[1] = { BMI2_STEP_COUNTER };

  int8_t r = bmi270_sensor_disable(feat, 1, &bme270);
  if (r != BMI2_OK) {
    last_status = r;
    return r;
  }

  bme270.delay_us(2000, bme270.intf_ptr);

  r = bmi270_sensor_enable(feat, 1, &bme270);
  last_status = r;
  return r;
}

/**
- Read step activity output
- activity : out activity value
- return : Bosch status
*/
int8_t BMI270_7Semi::readStepActivity(uint8_t& activity) {
  bmi2_feat_sensor_data fd;
  memset(&fd, 0, sizeof(fd));
  fd.type = BMI2_STEP_ACTIVITY;

  last_status = bmi270_get_feature_data(&fd, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  activity = fd.sens_data.activity_output;
  return last_status;
}

/**
- Read wrist gesture output
- gesture : out gesture value
- return : Bosch status
*/
int8_t BMI270_7Semi::readWristGesture(uint8_t& gesture) {
  bmi2_feat_sensor_data fd;
  memset(&fd, 0, sizeof(fd));
  fd.type = BMI2_WRIST_GESTURE;

  last_status = bmi270_get_feature_data(&fd, 1, &bme270);
  if (last_status != BMI2_OK) return last_status;

  gesture = fd.sens_data.wrist_gesture_output;
  return last_status;
}

// ================== NVM ==================
/**
- Program NVM (if supported by device/variant)
- return : Bosch status
*/
int8_t BMI270_7Semi::nvmProgram() {
  last_status = bmi2_nvm_prog(&bme270);
  return last_status;
}

// ================== Bosch callbacks ==================
/**
- Bosch read
- reg : register address
- data : out buffer
- len : number of bytes
- intf_ptr : BusConfig pointer
- return : Bosch status
*/
int8_t BMI270_7Semi::read(uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr) {
  BusConfig* Bus_config = (BusConfig*)intf_ptr;
  if (!Bus_config || !data) return BMI2_E_NULL_PTR;

  if (Bus_config->bus == Bus::I2C) {
    Bus_config->i2c->beginTransmission(Bus_config->addr);
    Bus_config->i2c->write(reg);
    if (Bus_config->i2c->endTransmission(false) != 0) return BMI2_E_COM_FAIL;

    uint32_t got = Bus_config->i2c->requestFrom((int)Bus_config->addr, (int)len);
    if (got != len) return BMI2_E_COM_FAIL;

    for (uint32_t i = 0; i < len; i++) data[i] = (uint8_t)Bus_config->i2c->read();
    return BMI2_OK;
  }

  Bus_config->spi->beginTransaction(SPISettings(Bus_config->spiHz, MSBFIRST, 0));
  digitalWrite(Bus_config->cs, LOW);
  delayMicroseconds(2);
  Bus_config->spi->transfer(reg | 0x80);
  for (uint32_t i = 0; i < len; i++) data[i] = Bus_config->spi->transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(Bus_config->cs, HIGH);
  Bus_config->spi->endTransaction();

  return BMI2_OK;
}

/**
- Bosch write
- reg : register address
- data : input buffer
- len : number of bytes
- intf_ptr : BusConfig pointer
- return : Bosch status
*/
int8_t BMI270_7Semi::write(uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr) {
  BusConfig* Bus_config = (BusConfig*)intf_ptr;
  if (!Bus_config || !data) return BMI2_E_NULL_PTR;

  if (Bus_config->bus == Bus::I2C) {
    Bus_config->i2c->beginTransmission(Bus_config->addr);
    Bus_config->i2c->write(reg);
    for (uint32_t i = 0; i < len; i++) Bus_config->i2c->write(data[i]);
    if (Bus_config->i2c->endTransmission(true) != 0) return BMI2_E_COM_FAIL;
    return BMI2_OK;
  }

  Bus_config->spi->beginTransaction(SPISettings(Bus_config->spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(Bus_config->cs, LOW);

  Bus_config->spi->transfer(reg & 0x7F);
  for (uint32_t i = 0; i < len; i++) Bus_config->spi->transfer(data[i]);

  digitalWrite(Bus_config->cs, HIGH);
  Bus_config->spi->endTransaction();

  return BMI2_OK;
}

/**
- Bosch delay callback
- period : delay in microseconds
*/
void BMI270_7Semi::delayUs(uint32_t period, void* /*intf_ptr*/) {
  delayMicroseconds(period);
}

// ================== Internal init ==================
/**
- Setup Bosch device struct and run bmi270_init()
- Sets default configs and enables accel+gyro
- return : true if init ok
*/
bool BMI270_7Semi::initBMI270() {
  // ----- callback wiring -----
  bme270.read = read;
  bme270.write = write;
  bme270.delay_us = delayUs;
  bme270.intf_ptr = &busConfig;

  // ----- interface select -----
  if (busConfig.bus == Bus::I2C) {
    bme270.intf = BMI2_I2C_INTF;
    bme270.read_write_len = 32;
    // bme270.dev_id = busConfig.addr;
  } else {
    bme270.intf = BMI2_SPI_INTF;
    bme270.read_write_len = 32;
  }

  // ----- Bosch init -----
    last_status = bmi270_init(&bme270);
  if (last_status != BMI2_OK) return false;

  // ----- cache chip id -----
  uint8_t id = 0;
  last_status = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &id, 1, &bme270);
  Serial.println("ID:" + (String)id);
  if (last_status == BMI2_OK) chip_id = id;

  // ----- default power mode -----
  last_status = bmi2_set_adv_power_save(BMI2_DISABLE, &bme270);
  if (last_status != BMI2_OK) return false;

  // ----- default accel config -----
  last_status = setAccelConfig(BMI2_ACC_ODR_100HZ,
                               BMI2_ACC_RANGE_2G,
                               BMI2_ACC_NORMAL_AVG4,
                               BMI2_PERF_OPT_MODE);
  if (last_status != BMI2_OK) return false;

  // ----- default gyro config -----
  last_status = setGyroConfig(BMI2_GYR_ODR_100HZ,
                              BMI2_GYR_RANGE_2000,
                              BMI2_GYR_NORMAL_MODE,
                              BMI2_PERF_OPT_MODE);
  if (last_status != BMI2_OK) return false;

  // ----- enable accel + gyro -----
  uint8_t list[2] = { BMI2_ACCEL, BMI2_GYRO };
  last_status = bmi2_sensor_enable(list, 2, &bme270);
  if (last_status != BMI2_OK) return false;

  return true;
}

// ================== Scale helpers ==================
/**
- Accel scale lookup
- range : BMI2_ACC_RANGE_*
- return : lsb per g
*/
float BMI270_7Semi::accLsbPerG_(uint8_t range) const {
  switch (range) {
    case BMI2_ACC_RANGE_2G: return 16384.0f;
    case BMI2_ACC_RANGE_4G: return 8192.0f;
    case BMI2_ACC_RANGE_8G: return 4096.0f;
    case BMI2_ACC_RANGE_16G: return 2048.0f;
    default: return 16384.0f;
  }
}

/**
- Gyro scale lookup
- range : BMI2_GYR_RANGE_*
- return : lsb per dps
*/
float BMI270_7Semi::gyrLsbPerDps_(uint8_t range) const {
  switch (range) {
    case BMI2_GYR_RANGE_2000: return 16.4f;
    case BMI2_GYR_RANGE_1000: return 32.8f;
    case BMI2_GYR_RANGE_500: return 65.6f;
    case BMI2_GYR_RANGE_250: return 131.2f;
    case BMI2_GYR_RANGE_125: return 262.4f;
    default: return 16.4f;
  }
}

// ================== Convenience float reads ==================
/**
- Read accel and convert to g
- ax_g,ay_g,az_g : out values
- return : true if read ok
*/
bool BMI270_7Semi::readAccel(float& ax_g, float& ay_g, float& az_g) {
  bmi2_sens_data data;
  memset(&data, 0, sizeof(data));

  last_status = readSample(data);
  if (last_status != BMI2_OK) return false;

  uint8_t odr = 0, range = 0, bwp = 0, perf = 0;
  last_status = getAccelConfig(odr, range, bwp, perf);
  if (last_status != BMI2_OK) return false;

  float lsb_per_g = accLsbPerG_(range);

  ax_g = (float)data.acc.x / lsb_per_g;
  ay_g = (float)data.acc.y / lsb_per_g;
  az_g = (float)data.acc.z / lsb_per_g;

  return true;
}

/**
- Read gyro and convert to dps
- gx_dps,gy_dps,gz_dps : out values
- return : true if read ok
*/
bool BMI270_7Semi::readGyro(float& gx_dps, float& gy_dps, float& gz_dps) {
  bmi2_sens_data data;
  memset(&data, 0, sizeof(data));

  last_status = readSample(data);
  if (last_status != BMI2_OK) return false;

  uint8_t odr = 0, range = 0, bwp = 0, perf = 0;
  last_status = getGyroConfig(odr, range, bwp, perf);
  if (last_status != BMI2_OK) return false;

  float lsb_per_dps = gyrLsbPerDps_(range);

  gx_dps = (float)data.gyr.x / lsb_per_dps;
  gy_dps = (float)data.gyr.y / lsb_per_dps;
  gz_dps = (float)data.gyr.z / lsb_per_dps;

  return true;
}

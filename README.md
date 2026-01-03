# 📘 BMI270 7Semi Library

This repository contains basic, well-documented examples for using the Bosch BMI270 IMU with an ESP32, using the 7Semi_BMI270 library.  

**Supported interfaces:** I2C / SPI  

---

## 🔌 Connection Guide

### 🟢 I2C Connection (Default & Recommended)

| BMI270 Pin | ESP32 Pin | Notes             |
|------------|-----------|-----------------|
| VCC        | 3.3V      | check your board's tolerance   |
| GND        | GND       | Common ground   |
| SDA        | GPIO 21   | Can be changed  |
| SCL        | GPIO 22   | Can be changed  |
| SDO        | LOW       | Sets I2C address to 0x68 |

**I2C Address:**  

- 0x68 → SDO = LOW  
- 0x69 → SDO = HIGH  

**I2C Speed:**  

- Recommended: 400 kHz  
- Supported: up to 1 MHz  

---

### 🔵 SPI Connection (Advanced)

⚠️ **Important:** BMI270 selects SPI or I2C at power-up. To force SPI mode, CS must be LOW during power-up.  

**ESP32 VSPI Wiring:**  

| BMI270 Pin | ESP32 Pin | Notes                |
|------------|-----------|--------------------|
| CS         | GPIO 5    | Must be LOW at boot |
| SCK        | GPIO 18   | VSPI clock          |
| MISO       | GPIO 19   | VSPI MISO           |
| MOSI       | GPIO 23   | VSPI MOSI           |
| VCC        | 3.3V      | Required            |
| GND        | GND       | Required            |

**SPI Configuration:**  

- Mode: SPI MODE 0  
- Init speed: ≤ 1 MHz  
- Normal speed: ≤ 10 MHz  

---

## ⚙️ Sensor Configuration

### 🧭 Accelerometer

**Output Data Rate (ODR):**  

- BMI2_ACC_ODR_0_78HZ   (ultra low power)  
- BMI2_ACC_ODR_12_5HZ  
- BMI2_ACC_ODR_25HZ  
- BMI2_ACC_ODR_50HZ  
- BMI2_ACC_ODR_100HZ   ← recommended  
- BMI2_ACC_ODR_200HZ  
- BMI2_ACC_ODR_400HZ  
- BMI2_ACC_ODR_800HZ  
- BMI2_ACC_ODR_1600HZ  

**Measurement Range:**  

- BMI2_ACC_RANGE_2G    ← best resolution  
- BMI2_ACC_RANGE_4G  
- BMI2_ACC_RANGE_8G  
- BMI2_ACC_RANGE_16G   (large motion)  

**Bandwidth (Noise vs Latency):**  

- BMI2_ACC_OSR4_AVG1   (lowest latency)  
- BMI2_ACC_OSR2_AVG2  
- BMI2_ACC_NORMAL_AVG4 ← recommended  
- BMI2_ACC_CIC_AVG8  
- BMI2_ACC_RES_AVG16  
- BMI2_ACC_RES_AVG32  
- BMI2_ACC_RES_AVG64  
- BMI2_ACC_RES_AVG128  (lowest noise)  

---

### 🔄 Gyroscope Options

**Output Data Rate (ODR):**  

- BMI2_GYR_ODR_25HZ  
- BMI2_GYR_ODR_50HZ  
- BMI2_GYR_ODR_100HZ   ← recommended  
- BMI2_GYR_ODR_200HZ  
- BMI2_GYR_ODR_400HZ  
- BMI2_GYR_ODR_800HZ  
- BMI2_GYR_ODR_1600HZ  
- BMI2_GYR_ODR_3200HZ  

**Measurement Range (deg/s):**  

- BMI2_GYR_RANGE_125  
- BMI2_GYR_RANGE_250  
- BMI2_GYR_RANGE_500  
- BMI2_GYR_RANGE_1000  
- BMI2_GYR_RANGE_2000  ← fast motion  

**Bandwidth:**  

- BMI2_GYR_OSR4_MODE   (lowest latency)  
- BMI2_GYR_OSR2_MODE  
- BMI2_GYR_NORMAL_MODE ← recommended  

---

### ✅ Recommended Default Configuration

| Sensor        | ODR   | Range       | Bandwidth      |
|---------------|-------|------------|----------------|
| Accelerometer | 100 Hz | ±2G        | NORMAL_AVG4    |
| Gyroscope     | 100 Hz | ±2000 dps  | NORMAL_MODE    |

### 📈 Output Units

| Sensor        | Unit                  |
|---------------|---------------------|
| Accelerometer | g                   |
| Gyroscope     | degrees/second (dps)|
| Temperature   | °C                  |

---

## 🧪 Debug Tips

- **Wrong CHIP_ID?**  
  Expected value: 0x24. Check wiring and power.  

- **SPI not working?**  
  CS must be LOW at power-up.  

- **No data?**  
  Confirm ODR is enabled and check loop timing.  

---

## 🚀 Next Steps / Advanced Features

- FIFO burst reads  
- Interrupt-driven data ready  
- Low-power mode  
- Sensor fusion (external)  
- Calibration routines  

---

## 📄 License

**MIT License** – This Arduino/C++ wrapper, examples, and documentation are released under the MIT License.  

**Bosch Sensortec Driver:**  

- This library uses the official Bosch Sensortec BMI2/BMI270 C driver, which is copyright © Bosch Sensortec GmbH and licensed under the BSD 3-Clause License.  
- Bosch license terms must be retained when redistributing Bosch source/header files.  
- This wrapper does not modify or replace Bosch’s license; it only adds Arduino integration.

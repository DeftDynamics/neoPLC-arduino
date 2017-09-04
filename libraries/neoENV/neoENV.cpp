// neoPLC-ENV environmental sensor library
// adapted from official BME280 code (Bosch) and Adafruit
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "neoENV.h"

neoENV::neoENV(uint8_t addr)
{
  // constructor
  _i2caddr = addr;
}

bool neoENV::begin()
{
    Wire.begin();

    // check if sensor, i.e. the chip ID is correct
    if (read8(BME280_REGISTER_CHIPID) != 0x60)
    {
        return false;
    } else {
        write8(BME280_REGISTER_SOFTRESET, 0xB6); // reset
        delay(200); // wait for chip to wake up
        while (isReadingCalibration()) {delay(50);}
        readCoefficients(); // see DS 4.2.2
        setSampling();      // use defaults
        return true;
    }
}


void neoENV::setSampling(sensor_mode       mode,
                            		  sensor_sampling   tempSampling,
                            		  sensor_sampling   pressSampling,
                            		  sensor_sampling   humSampling,
                            		  sensor_filter     filter,
                            		  standby_duration  duration) 
{
    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;
    
    _humReg.osrs_h    = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb   = duration;

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    write8(BME280_REGISTER_CONFIG, _configReg.get());
    write8(BME280_REGISTER_CONTROL, _measReg.get());
}


void neoENV::write8(byte reg, byte value) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)value);
        Wire.endTransmission();
}


uint8_t neoENV::read8(byte reg) {
    uint8_t value;
    
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
        value = Wire.read();

    return value;
}

uint16_t neoENV::read16(byte reg)
{
    uint16_t value;

        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
        value = (Wire.read() << 8) | Wire.read();

    return value;
}


uint16_t neoENV::read16_LE(byte reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t neoENV::readS16(byte reg)
{
    return (int16_t)read16(reg);
}

int16_t neoENV::readS16_LE(byte reg)
{
    return (int16_t)read16_LE(reg);
}

uint32_t neoENV::read24(byte reg)
{
    uint32_t value;
    
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)3);

    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

    return value;
}

void neoENV::takeForcedMeasurement()
{   
  if (_measReg.mode == MODE_FORCED) {
    write8(BME280_REGISTER_CONTROL, _measReg.get());
    while (read8(BME280_REGISTER_STATUS) & 0x08)
	  delay(1);
  }
}

void neoENV::readCoefficients(void)
{
    _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

bool neoENV::isReadingCalibration(void)
{
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

float neoENV::readTemperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
            ((int32_t)_bme280_calib.dig_T2)) >> 11;
             
    var2 = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
              ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
            ((int32_t)_bme280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float neoENV::readPressure(void) {
    int64_t var1, var2, p;

    readTemperature(); // must be done first to get t_fine

    int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bme280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3)>>8) +
           ((var1 * (int64_t)_bme280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7)<<4);
    return (float)p/256.0;
}

float neoENV::readHumidity(void) {
    readTemperature();

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;
        
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                    (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)_bme280_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0;
}

float neoENV::readAltitude()
{
    float seaLevel = SEALEVELPRESSURE_HPA;
    float atmospheric = readPressure()/100.0;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903)); // see page 16
}


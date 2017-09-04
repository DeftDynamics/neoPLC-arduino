// neoPLC-IMU library
// Deft Dynamics

#include "arduino.h"
#include "neoIMU.h"
#include <Wire.h>

neoIMU::neoIMU(uint8_t addr_a, uint8_t addr_g, uint8_t addr_m)
{
  // constructor
    uint8_t _addr_acc = addr_a;
    uint8_t _addr_gyr = addr_g;
    uint8_t _addr_mag = addr_m;
}

bool neoIMU::begin(int acc_range, int acc_bw, int gyro_range, int gyro_bw, int mag_odr)
{
  Wire.begin();
  Wire.setClock(250000);
  uint8_t a=0;
  uint8_t b=0;
  uint8_t c=0;
  
  // start with all sensors in default mode with all registers reset
   writeByte(_addr_acc,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
   delay(500); // Wait for all registers to reset 

  a = readByte(_addr_acc, BMX055_ACC_WHOAMI) == 0xFA;  // Read acc WHO_AM_I register and check value
  b = readByte(_addr_gyr, BMX055_GYRO_WHOAMI) == 0x0F;  // Read gyro WHO_AM_I register and check value
      writeByte(_addr_mag, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer
      delay(10);
  c = readByte(_addr_mag, BMX055_MAG_WHOAMI) == 0x32 ;  // Read mag WHO_AM_I register and check value

  if ( a & b & c ) // if all 3 sensors connect, complete the setup and return success flag
  {
    setupAcc(acc_range, acc_bw);
    setupGyro(gyro_range, gyro_bw);
    setupMag(mag_odr);
    return 1;
  }
  else             // if any of 3 sensors doesn't connect, return failure flag
  {
    return 0;
  }
}

void neoIMU::pollAccel()
{
  uint8_t rawData[6];
  int16_t raw_ax;
  int16_t raw_ay;
  int16_t raw_az;
  readBytes(_addr_acc, BMX055_ACC_D_X_LSB, 6, &rawData[0]);      
  if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) 
  { 
    raw_ax = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
    raw_ay = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;  
    raw_az = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4; 
    ax = (float)raw_ax*_acc_res;
    ay = (float)raw_ay*_acc_res;
    az = (float)raw_az*_acc_res;
  }
}

void neoIMU::pollGyro()
{
  uint8_t rawData[6];
  int16_t raw_gx;
  int16_t raw_gy;
  int16_t raw_gz;
  readBytes(_addr_gyr, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    raw_gx = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    raw_gy = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    raw_gz = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
    gx = (float)raw_gx*_gyro_res;
    gy = (float)raw_gy*_gyro_res;
    gz = (float)raw_gz*_gyro_res;
}

void neoIMU::pollMag()
{
  int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
  uint16_t data_r = 0;
  uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
  int16_t raw_mx=0;
  int16_t raw_my=0;
  int16_t raw_mz=0;
  
  readBytes(_addr_mag, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
    if(rawData[6] & 0x01) // data ready
    {
      mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
      mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
      mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
      data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
  
     // calculate temperature compensated 16-bit magnetic fields
     temp = ((int16_t)(((uint16_t)((((int32_t)_dig_xyz1) << 14)/(data_r != 0 ? data_r : _dig_xyz1))) - ((uint16_t)0x4000)));
     raw_mx = ((int16_t)((((int32_t)mdata_x) *
          ((((((((int32_t)_dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
             (((int32_t)temp) * ((int32_t)(((int16_t)_dig_xy1) << 7)))) >> 9) +
           ((int32_t)0x100000)) * ((int32_t)(((int16_t)_dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) + (((int16_t)_dig_x1) << 3);
     raw_my = ((int16_t)((((int32_t)mdata_y) *
          ((((((((int32_t)_dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) + 
             (((int32_t)temp) * ((int32_t)(((int16_t)_dig_xy1) << 7)))) >> 9) +
                 ((int32_t)0x100000)) * ((int32_t)(((int16_t)_dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
        (((int16_t)_dig_y1) << 3);
     raw_mz = (((((int32_t)(mdata_z - _dig_z4)) << 15) - ((((int32_t)_dig_z3) * ((int32_t)(((int16_t)data_r) -
           ((int16_t)_dig_xyz1))))>>2))/(_dig_z2 + ((int16_t)(((((int32_t)_dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
    mx = (float)raw_mx*_mag_res;
    my = (float)raw_my*_mag_res;
    mz = (float)raw_mz*_mag_res;
    
   }
}

float neoIMU::readTemp()
{
  int c =  readByte(_addr_acc, BMX055_ACC_D_TEMP);  // Read the raw data register (8-bit = 1-byte)
  int16_t t = ((int16_t)((int16_t)c << 8)) >> 8;
  float temp = ((float)t/1.42+23); // read accelerometer temp and convert to deg C
  return temp;  // Turn the byte into a signed 8-bit integer
}

void neoIMU::pollAll()
{
  pollAccel(); // update acceleration in Gs IF new data is available
  pollGyro(); // update rotation rate in deg/s 
  pollMag(); // update magnetic flux in mGauss IF new data is available
}

void neoIMU::setupAcc(int acc_range, int acc_bw)
{
  uint8_t Ascale = 0x03;
  switch (acc_range)
  {
    case 2:
      _acc_res = 2.0/2048.0; // 2 G
      Ascale = 0x03; break;
    case 4:
      _acc_res = 4.0/2048.0; // 4 G
      Ascale = 0x05; break;
    case 8:
      _acc_res = 8.0/2048.0; // 8 G
      Ascale = 0x08; break;
    case 16:
      _acc_res = 16.0/2048.0; // 16 G
      Ascale = 0x0C; break;
    default:
      _acc_res = 2.0/2048.0; // default: 2 G
      Ascale = 0x03; break;
  }

  uint8_t Abw;
  switch (acc_bw)
  {
    case 8:
      Abw = 0x00; break; // 7.81 Hz,  64 ms update time
    case 16:
      Abw = 0x01; break; // 15.63 Hz, 32 ms update time
    case 32:
      Abw = 0x02; break; // 31.25 Hz, 16 ms update time
    case 64:
      Abw = 0x03; break; // 62.5  Hz,  8 ms update time
    case 125:
      Abw = 0x04; break; // 125   Hz,  4 ms update time
    case 250:
      Abw = 0x05; break; // 250   Hz,  2 ms update time
    case 500:
      Abw = 0x06; break; // 500   Hz,  1 ms update time
    case 1000:
      Abw = 0x07; break; // 1000  Hz,  0.5 ms update time
    default:
      Abw = 0x04; break; // default to: 125  Hz,  4 ms update time
  }
  
   writeByte(_addr_acc, BMX055_ACC_PMU_RANGE, Ascale & 0x0F);    // Set accelerometer full range
   writeByte(_addr_acc, BMX055_ACC_PMU_BW, (0x08 | Abw) & 0x0F); // Set accelerometer bandwidth
   writeByte(_addr_acc, BMX055_ACC_D_HBW, 0x00);                  // Use filtered data

    _dig_x1 = readByte(_addr_acc, BMM050_DIG_X1);
    _dig_x2 = readByte(_addr_acc, BMM050_DIG_X2);
    _dig_y1 = readByte(_addr_acc, BMM050_DIG_Y1);
    _dig_y2 = readByte(_addr_acc, BMM050_DIG_Y2);
    _dig_xy1 = readByte(_addr_acc, BMM050_DIG_XY1);
    _dig_xy2 = readByte(_addr_acc, BMM050_DIG_XY2);
}

void neoIMU::setupGyro(int gyro_range, int gyro_bw)
{
  uint8_t Gscale = 0x03;
  switch (gyro_range)
  {
    case 125:
      _gyro_res = 124.87/32768.0; 
      Gscale = 0x04; break;
    case 250:
      _gyro_res = 249.75/32768.0; 
      Gscale = 0x03; break;
    case 500:
      _gyro_res = 499.5/32768.0; 
      Gscale = 0x02; break;
    case 1000:
      _gyro_res = 999.0/32768.0; 
      Gscale = 0x01; break;
    case 2000:
      _gyro_res = 1998.0/32768.0; 
      Gscale = 0x00; break;
    default:
      _gyro_res = 249.75/32768.0; 
      Gscale = 0x03; break;
  }

  uint8_t Gbw;
  switch (gyro_bw)
  {
    case 523:
      Gbw = 0x00; break; // 2000 Hz ODR, 523 Hz BW
    case 230:
      Gbw = 0x01; break; // 2000 Hz ODR, 230 Hz BW
    case 116:
      Gbw = 0x02; break; // 1000 Hz ODR, 116 Hz BW
    case 47:
      Gbw = 0x03; break; //  400 Hz ODR,  47 Hz BW
    case 23:
      Gbw = 0x04; break; //  200 Hz ODR,  23 Hz BW
    case 12:
      Gbw = 0x05; break; //  100 Hz ODR,  12 Hz BW
    case 64:
      Gbw = 0x06; break; //  200 Hz ODR,  64 Hz BW
    case 32:
      Gbw = 0x07; break; //  100 Hz ODR,  32 Hz BW
    default:
      Gbw = 0x02; break; // default to: 1000 Hz ODR, 116 Hz BW
  }
  
  writeByte(_addr_gyr, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
  writeByte(_addr_gyr, BMX055_GYRO_BW, Gbw);     // set GYRO ODR and Bandwidth

}

void neoIMU::setupMag(int mag_odr)
{
    uint8_t Mbw;
  switch (mag_odr)
  {
    case 10:
      Mbw = 0x00; break; // 10 Hz ODR
    case 2:
      Mbw = 0x01; break; //  2 Hz ODR
    case 6:
      Mbw = 0x02; break; //  6 Hz ODR
    case 8:
      Mbw = 0x03; break; //  8 Hz ODR
    case 15:
      Mbw = 0x04; break; // 15 Hz ODR
    case 20:
      Mbw = 0x05; break; // 20 Hz ODR
    case 25:
      Mbw = 0x06; break; // 25 Hz ODR
    case 30:
      Mbw = 0x07; break; // 30 Hz ODR
    default:
      Mbw = 0x07; break; // default to: 30 Hz ODR
  }
  
  //writeByte(BMX055_MAG_ADDR, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
  //writeByte(BMX055_MAG_ADDR, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
  writeByte(_addr_mag, BMX055_MAG_PWR_CNTL2, Mbw << 3); // Normal mode
  
         // Low-power
       //   writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
       //   writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
          // Regular
       //   writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
       //   writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
          // Enhanced Regular
          writeByte(_addr_mag, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
          writeByte(_addr_mag, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
          // High Accuracy
        //  writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
        //  writeByte(BMX055_MAG_ADDR, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)


      
        uint8_t rawData[2];  //placeholder for 2-byte trim data
          readBytes(_addr_mag, BMM050_DIG_Z1_LSB, 2, &rawData[0]);   
        _dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);  
          readBytes(_addr_mag, BMM050_DIG_Z2_LSB, 2, &rawData[0]);   
        _dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
          readBytes(_addr_mag, BMM050_DIG_Z3_LSB, 2, &rawData[0]);   
        _dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
          readBytes(_addr_mag, BMM050_DIG_Z4_LSB, 2, &rawData[0]);   
        _dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
          readBytes(_addr_mag, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);   
        _dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);  

        _mag_res =  1.0/1.6;
}

void neoIMU::writeByte(uint8_t ic_address , uint8_t reg_address, uint8_t byte_to_send)
{
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.write(byte_to_send);
  Wire.endTransmission();
}

uint8_t neoIMU::readByte(uint8_t ic_address, uint8_t reg_address)
{
  uint8_t message;
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.endTransmission(0);        
  Wire.requestFrom(ic_address, (size_t) 1);
  message = Wire.read();
  return message;
}

void neoIMU::readBytes(uint8_t ic_address, uint8_t reg_address, uint8_t num_bytes, uint8_t *output_location)
{
  uint8_t i = 0;
  Wire.beginTransmission(ic_address);
  Wire.write(reg_address);
  Wire.endTransmission(0);
  Wire.requestFrom(ic_address, (size_t) num_bytes);
  while (Wire.available()) 
        {
          output_location[i++] = Wire.read();
        }  
}




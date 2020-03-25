/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  Extended functionality by Ron Driessen for Philibert Couwenbergh (april 2017).
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BMP280_H__
#define __BMP280_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

/*=========================================================================
    TEMPERATURE OVERSAMPLING                                          Phi
    Only bits[7,6,5] are relevant (bit group is named: osrs_t)
    -----------------------------------------------------------------------*/
    #define BMP280_TOVERSAMPLING_1     (B00100000)
    #define BMP280_TOVERSAMPLING_2     (B01000000)
    #define BMP280_TOVERSAMPLING_4     (B01100000)
    #define BMP280_TOVERSAMPLING_8     (B10000000)
    #define BMP280_TOVERSAMPLING_16    (B10100000)
/*=========================================================================*/

/*=========================================================================
    PRESSURE OVERSAMPLING                                             Phi
    Only bits[4,3,2] are relevant (bit group is named: osrs_p)
    -----------------------------------------------------------------------*/
    #define BMP280_POVERSAMPLING_1     (B00000100)
    #define BMP280_POVERSAMPLING_2     (B00001000)
    #define BMP280_POVERSAMPLING_4     (B00001100)
    #define BMP280_POVERSAMPLING_8     (B00010000)
    #define BMP280_POVERSAMPLING_16    (B00010100)
/*=========================================================================*/

/*=========================================================================
    POWER MODES                                                       Phi
    Only bits[1,0] are relevant (bit group is named: mode)
    In sleep mode no measurements are performed.
      You will read old data (from the last performed measurement).
    In forced mode a single measurement is performed.
      You will read current data.
      In between reads, (can be any duration), the sensor will fall back to
      a power saving mode that uses a tiny bit more than the real sleep mode.
    In normal mode measurements are made continuously.
      The time in between measurements, (the so called Standby Time),
      is always the same (but can be set).
    -----------------------------------------------------------------------*/
    #define BMP280_POWERMODE_SLEEP     (B00000000)
    #define BMP280_POWERMODE_FORCED    (B00000001)
    #define BMP280_POWERMODE_NORMAL    (B00000011)
/*=========================================================================*/

/*=========================================================================
    STANDBY TIME                                                      Phi
    Only bits[7,6,5] are relevant (bit group is named: t_sb)
    The Standby Time is used only while operating in the NORMAL power mode.
    It's the time in between two measurements (in units of milliseconds).
    -----------------------------------------------------------------------*/
    #define BMP280_STANDBYTIME_0_5     (B00000000)     //    0.5 ms
    #define BMP280_STANDBYTIME_62_5    (B00100000)     //   62.5 ms
    #define BMP280_STANDBYTIME_125     (B01000000)     //  125   ms
    #define BMP280_STANDBYTIME_250     (B01100000)     //  250   ms
    #define BMP280_STANDBYTIME_500     (B10000000)     //  500   ms
    #define BMP280_STANDBYTIME_1000    (B10100000)     // 1000   ms
    #define BMP280_STANDBYTIME_2000    (B11000000)     // 2000   ms
    #define BMP280_STANDBYTIME_4000    (B11100000)     // 4000   ms
/*=========================================================================*/

/*=========================================================================
    FILTER COEFFICIENT                                                Phi
    Only bits[4,3,2] are relevant (bit group is named: filter)
    The value represents the number of samples needed to reach >= 75% of step response.
    -----------------------------------------------------------------------*/
    #define BMP280_FILTER_OFF          (B00000000)     //  1 sample
    #define BMP280_FILTER_2            (B00000100)     //  2 samples
    #define BMP280_FILTER_4            (B00001000)     //  5 samples
    #define BMP280_FILTER_8            (B00001100)     // 11 samples
    #define BMP280_FILTER_16           (B00010000)     // 22 samples
/*=========================================================================*/

/*=========================================================================
    3-WIRE SPI                                                        Phi
    Only bit[0] is relevant (bit group is named: spi3w_en)
    -----------------------------------------------------------------------*/
    #define BMP280_SPI_DEFAULT         (B00000000)     // 4-wire SPI
    #define BMP280_SPI_3WIRE           (B00000001)     // 3-wire SPI
/*=========================================================================*/



/*
class Adafruit_BMP280_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP280_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/

class Adafruit_BMP280
{
  public:
    Adafruit_BMP280(void);
    Adafruit_BMP280(int8_t cspin);
    Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);

    // Phi extended functionality
    void  powerMode(uint8_t mode);
    void  overSampling(uint8_t osTemperature, uint8_t osPressure);
    void  standbyTime(uint8_t duration);
    void  filter(uint8_t filter);
    float calcSealevelPressure(float altitude);
/*
//  i did not check out Kevin's original SPI code.
//  so i just keep this next declaration commented out.
    void  wiresSPI(uint8_t wires);
*/
  private:

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t t_fine;

    int8_t _cs, _mosi, _miso, _sck;

    bmp280_calib_data _bmp280_calib;

    // Phi extended functionality
    uint8_t _ctrl_meas;	// register 0xF4, BMP280_REGISTER_CONTROL
    uint8_t _config;    // register 0xF5, BMP280_REGISTER_CONFIG
};

#endif

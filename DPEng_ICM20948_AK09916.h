/*!
 * @file DPEng_ICM20948_AK09916.h
 *
 * This header file is part of DPEng's ICM20948 driver for the Arduino platform.  It is
 * designed specifically to work with the DPEng ICM20948 breakout board which can be 
 * purchased from eBay here:
 * https://www.ebay.co.uk/itm/323724746939
 * or Amazon here:
 * https://www.amazon.co.uk/DP-Eng-ICM-20948-Breakout-obsolete/dp/B07PDTKK3Y
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout. SPI is also possible with <2 MBIT/s speeds.
 *
 * DP Engineering invests time and resources providing this open source code,
 * please support DPEng by purchasing this breakout board from DPEng
 *
 * Written by David Pattison for DP Engineering Ltd.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
/** \file DPEng_ICM20948_AK09916.h */
#ifndef __DPEng_ICM20948_AK09916_H__
#define __DPEng_ICM20948_AK09916_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    /** 7-bit I2C address for this sensor */
    #define ICM20948_ACCELGYRO_ADDRESS           (0x69)     // 0110 1001
    /** Device ID for this sensor (used as sanity check during init) */
    #define ICM20948_ACCELGYRO_ID                (0xEA)     // 1110 1010
	/** 7-bit address for this sensor */
    #define ICM20948_MAG_ADDRESS       			 (0x0C)       // 0000001
	/** Device ID for this sensor (used as a sanity check during init) */
    #define ICM20948_MAG_ID           			 (0x09)       // 0000 1001
	
	/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
	#define ACCEL_MG_LSB_2G (0.000061035F)
	/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
	#define ACCEL_MG_LSB_4G (0.000122070F)
	/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
	#define ACCEL_MG_LSB_8G (0.000244141F)
	/** Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */
	#define ACCEL_MG_LSB_16G (0.000488281F)
	
	/** Gyroscope sensitivity at 250dps */
    #define GYRO_SENSITIVITY_250DPS  (0.0076336F) // Table 1 of datasheet
    /** Gyroscope sensitivity at 500dps */
    #define GYRO_SENSITIVITY_500DPS  (0.0152672F)
    /** Gyroscope sensitivity at 1000dps */
    #define GYRO_SENSITIVITY_1000DPS (0.0304878F)
    /** Gyroscope sensitivity at 2000dps */
    #define GYRO_SENSITIVITY_2000DPS (0.0609756F)
	
	/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
	#define MAG_UT_LSB      (0.15F)
	
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    /*!
        Raw register addresses used to communicate with the sensor.
    */
    typedef enum
    {
		WHO_AM_I_ICM20948  		= 0x00, // Should return 0xEA
		USER_CTRL         	 	= 0x03,  // Bit 7 enable DMP, bit 3 reset DMP
		LP_CONFIG		   		= 0x05, 
		PWR_MGMT_1        		= 0x06, // Device defaults to the SLEEP mode
		PWR_MGMT_2        		= 0x07,
		INT_PIN_CFG       		= 0x0F,
		INT_ENABLE        		= 0x10,
		INT_ENABLE_1	   		= 0x11, 
		INT_ENABLE_2	   		= 0x12,
		INT_ENABLE_3	   		= 0x13,
		I2C_MST_STATUS     		= 0x17,
		INT_STATUS         		= 0x19,
		INT_STATUS_1	   		= 0x1A,
		INT_STATUS_2	   		= 0x1B, 
		INT_STATUS_3	   		= 0x1C, 
		DELAY_TIMEH				= 0x28,	
		DELAY_TIMEL				= 0x29,	
		ACCEL_XOUT_H       		= 0x2D,
		ACCEL_XOUT_L      		= 0x2E,
		ACCEL_YOUT_H       		= 0x2F,
		ACCEL_YOUT_L       		= 0x30,
		ACCEL_ZOUT_H      	 	= 0x31,
		ACCEL_ZOUT_L      		= 0x32,
		GYRO_XOUT_H        		= 0x33,
		GYRO_XOUT_L        		= 0x34,
		GYRO_YOUT_H        		= 0x35,
		GYRO_YOUT_L        		= 0x36,
		GYRO_ZOUT_H        		= 0x37,
		GYRO_ZOUT_L        		= 0x38,
		TEMP_OUT_H         		= 0x39,
		TEMP_OUT_L         		= 0x3A,
		EXT_SENS_DATA_00   		= 0x3B,
		EXT_SENS_DATA_01   		= 0x3C,
		EXT_SENS_DATA_02   		= 0x3D,
		EXT_SENS_DATA_03   		= 0x3E,
		EXT_SENS_DATA_04   		= 0x3F,
		EXT_SENS_DATA_05   		= 0x40,
		EXT_SENS_DATA_06   		= 0x41,
		EXT_SENS_DATA_07   		= 0x42,
		EXT_SENS_DATA_08   		= 0x43,
		EXT_SENS_DATA_09   		= 0x44,
		EXT_SENS_DATA_10   		= 0x45,
		EXT_SENS_DATA_11   		= 0x46,
		EXT_SENS_DATA_12   		= 0x47,
		EXT_SENS_DATA_13   		= 0x48,
		EXT_SENS_DATA_14   		= 0x49,
		EXT_SENS_DATA_15   		= 0x4A,
		EXT_SENS_DATA_16  		= 0x4B,
		EXT_SENS_DATA_17  		= 0x4C,
		EXT_SENS_DATA_18   		= 0x4D,
		EXT_SENS_DATA_19   		= 0x4E,
		EXT_SENS_DATA_20   		= 0x4F,
		EXT_SENS_DATA_21   		= 0x50,
		EXT_SENS_DATA_22   		= 0x51,
		EXT_SENS_DATA_23   		= 0x52,
		FIFO_EN_1          		= 0x66,
		FIFO_EN_2          		= 0x67, 
		FIFO_RST		   		= 0x68, 
		FIFO_MODE		   		= 0x69,
		FIFO_COUNTH        		= 0x70,
		FIFO_COUNTL        		= 0x71,
		FIFO_R_W           		= 0x72,
		DATA_RDY_STATUS			= 0x74, 
		FIFO_CFG		   		= 0x76, 
		REG_BANK_SEL	   		= 0x7F, 

		// USER BANK 1 REGISTER MAP
		SELF_TEST_X_GYRO  			= 0x02,
		SELF_TEST_Y_GYRO  			= 0x03,
		SELF_TEST_Z_GYRO  			= 0x04,
		SELF_TEST_X_ACCEL 			= 0x0E,
		SELF_TEST_Y_ACCEL 			= 0x0F,
		SELF_TEST_Z_ACCEL 			= 0x10,
		XA_OFFSET_H       			= 0x14,
		XA_OFFSET_L       			= 0x15,
		YA_OFFSET_H       			= 0x17,
		YA_OFFSET_L       			= 0x18,
		ZA_OFFSET_H       			= 0x1A,
		ZA_OFFSET_L       			= 0x1B,
		TIMEBASE_CORRECTION_PLL		= 0x28,

		// USER BANK 2 REGISTER MAP
		GYRO_SMPLRT_DIV        		= 0x00, 
		GYRO_CONFIG_1      			= 0x01, 
		GYRO_CONFIG_2      			= 0x02, 
		XG_OFFSET_H       			= 0x03,  // User-defined trim values for gyroscope
		XG_OFFSET_L       			= 0x04,
		YG_OFFSET_H       			= 0x05,
		YG_OFFSET_L       			= 0x06,
		ZG_OFFSET_H       			= 0x07,
		ZG_OFFSET_L       			= 0x08,
		ODR_ALIGN_EN				= 0x09, 
		ACCEL_SMPLRT_DIV_1     		= 0x10, 
		ACCEL_SMPLRT_DIV_2     		= 0x11, 
		ACCEL_INTEL_CTRL			= 0x12, 
		ACCEL_WOM_THR				= 0x13, 
		ACCEL_CONFIG      			= 0x14,
		ACCEL_CONFIG_2     			= 0x15, 
		FSYNC_CONFIG				= 0x52, 
		TEMP_CONFIG					= 0x53, 
		MOD_CTRL_USR				= 0x54, 

		// USER BANK 3 REGISTER MAP
		I2C_MST_ODR_CONFIG		= 0x00, 
		I2C_MST_CTRL       		= 0x01,
		I2C_MST_DELAY_CTRL 		= 0x02,
		I2C_SLV0_ADDR      		= 0x03,
		I2C_SLV0_REG       		= 0x04,
		I2C_SLV0_CTRL      		= 0x05,
		I2C_SLV0_DO        		= 0x06,
		I2C_SLV1_ADDR      		= 0x07,
		I2C_SLV1_REG       		= 0x08,
		I2C_SLV1_CTRL      		= 0x09,
		I2C_SLV1_DO        		= 0x0A,
		I2C_SLV2_ADDR      		= 0x0B,
		I2C_SLV2_REG       		= 0x0C,
		I2C_SLV2_CTRL      		= 0x0D,
		I2C_SLV2_DO        		= 0x0E,
		I2C_SLV3_ADDR      		= 0x0F,
		I2C_SLV3_REG       		= 0x10,
		I2C_SLV3_CTRL      		= 0x11,
		I2C_SLV3_DO        		= 0x12,
		I2C_SLV4_ADDR      		= 0x13,
		I2C_SLV4_REG       		= 0x14,
		I2C_SLV4_CTRL      		= 0x15,
		I2C_SLV4_DO        		= 0x16,
		I2C_SLV4_DI        		= 0x17
    } icm20948Registers_t;
	
	typedef enum
    {
		// ICM20948 Init Registers
		
      //Magnetometer Registers 
		WHO_AM_I_AK09916 		= 0x01,  // (AKA WIA2) should return 0x09
		AK09916_ST1      		= 0x10,  // data ready status bit 0
		AK09916_XOUT_L   		= 0x11,  // data
		AK09916_XOUT_H   		= 0x12,
		AK09916_YOUT_L   		= 0x13,
		AK09916_YOUT_H   		= 0x14,
		AK09916_ZOUT_L   		= 0x15,
		AK09916_ZOUT_H   		= 0x16,
		AK09916_ST2      		= 0x18,  // Data overflow bit 3 and data read error status bit 2
		AK09916_CNTL     		= 0x31,  // Power down (0000), single-measurement (0001), self-test (1000) modes on bits 3:0
		AK09916_CNTL2    		= 0x32   // Normal (0), Reset (1)
    } magRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    /*!
        Range settings for the accelerometer sensor.
    */
    typedef enum
    {
      ICM20948_ACCELRANGE_2G                = (0b00 << 1),
      ICM20948_ACCELRANGE_4G                = (0b01 << 1),
      ICM20948_ACCELRANGE_8G                = (0b10 << 1),
	  ICM20948_ACCELRANGE_16G               = (0b11 << 1)
    } icm20948AccelRange_t;
	
	/*!
        Enum to define valid gyroscope range values
    */
    typedef enum
    {
      GYRO_RANGE_250DPS  = 250,     /**< 250dps */
      GYRO_RANGE_500DPS  = 500,     /**< 500dps */
      GYRO_RANGE_1000DPS = 1000,    /**< 1000dps */
      GYRO_RANGE_2000DPS = 2000     /**< 2000dps */
    } icm20948GyroRange_t;

    /*!
        Lowpass settings for the accelerometer sensor.
    */
    typedef enum
    {
      ICM20948_ACCELLOWPASS_473_0_HZ        = (0b111 << 3),
      ICM20948_ACCELLOWPASS_246_0_HZ        = (0b001 << 3),
      ICM20948_ACCELLOWPASS_111_4_HZ        = (0b010 << 3),
      ICM20948_ACCELLOWPASS_50_4_HZ         = (0b011 << 3),
      ICM20948_ACCELLOWPASS_23_9_HZ         = (0b100 << 3),
      ICM20948_ACCELLOWPASS_11_5_HZ         = (0b101 << 3),
      ICM20948_ACCELLOWPASS_5_7_HZ          = (0b110 << 3)
    } icm20948AccelLowpass_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    /*!
        @brief  Raw (integer) values from the gyroscope sensor.
    */
    typedef struct
    {
      int16_t x;    /**< Raw int16_t value from the x axis */
      int16_t y;    /**< Raw int16_t value from the y axis */
      int16_t z;    /**< Raw int16_t value from the z axis */
    } icm20948RawData_t;
/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the DPEng ICM-20948 breakout.
*/
/**************************************************************************/
class DPEng_ICM20948 : public Adafruit_Sensor
{
  public:
    DPEng_ICM20948(int32_t accelSensorID = -1, int32_t gyroSensorID = -1, int32_t magSensorID = -1);

    bool begin           ( icm20948AccelRange_t rngAccel = ICM20948_ACCELRANGE_2G, icm20948GyroRange_t rngGyro = GYRO_RANGE_250DPS, icm20948AccelLowpass_t lowpassAccel = ICM20948_ACCELLOWPASS_50_4_HZ, uint8_t accelgyro_address = ICM20948_ACCELGYRO_ADDRESS );
    bool getEventAcc     ( sensors_event_t* accel );
    bool getEventMag     ( sensors_event_t* mag );
    bool getEvent        ( sensors_event_t* accel );
    void getSensor       ( sensor_t* accel );
    bool getEvent        ( sensors_event_t* accel, sensors_event_t* gyro, sensors_event_t* mag );
    void getSensor       ( sensor_t* accel, sensor_t* gyro, sensor_t* mag );
    void standby         ( boolean standby );

    /*! Raw accelerometer values from last sucsessful sensor read */
    icm20948RawData_t accel_raw;
    /*! Raw gyroscope values from last successful sensor read */
    icm20948RawData_t gyro_raw;
	/*! Raw magnetometer values from last successful sensor read */
    icm20948RawData_t mag_raw;

  private:
	void        write8  ( byte address, byte reg, byte value );
    void        write8  ( byte reg, byte value );
	byte		read8	( byte address, byte reg );
    byte        read8   ( byte reg );

    icm20948AccelRange_t 	_rangeAccel;
    icm20948GyroRange_t 	_rangeGyro;
    int32_t              	_accelSensorID;
    int32_t              	_gyroSensorID;
	int32_t              	_magSensorID;
	uint8_t					_accelgyro_address;
};

#endif

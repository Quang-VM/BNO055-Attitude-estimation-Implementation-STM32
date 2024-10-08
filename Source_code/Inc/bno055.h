/*
 * bno055.h
 *
 *  Created on: Sep 10, 2024
 *      Author: HQLap
 */

#ifndef BNO055_H_
#define BNO055_H_

#include "main.h"


/*
 * I2C
 */
extern I2C_HandleTypeDef hi2c2;
#define bno_i2c 		 (hi2c2)
#define bno055_timeout   2000


/** ----------------------------------------------------------------------------------------------------
  * 			 			   BNO055 Handle structure and macros definition
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct
{
	uint8_t		Frame;
	uint8_t 	Unit_Sel;
	uint8_t 	OP_Mode;
	uint8_t 	PWR_Mode;
	uint8_t 	Clock_Source;
	uint8_t 	ACC_Range;
	uint8_t 	GYR_Range;
}BNO055_Handle_t;

/** ----------------------------------------------------------------------------------------------------
  * 			 			  	Madgwick algorithm handle struct
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct
{
	uint8_t		frame;
	float		deltat;
	float 		beta;
}Madgwick_Handle_t;

/** ----------------------------------------------------------------------------------------------------
  * 			 			   BNO055 OPERATION MODES structure and macros definition
  * ----------------------------------------------------------------------------------------------------
  */
typedef enum
{
	//Non-fusion modes
	CONFIG_MODE 	=	0x00,	//This is the only mode in which all the writable register map entries can be changed (Except INT , INT_MASK, OPR_MODE)
	ACC_ONLY 		=	0x01,
	MAG_ONLY 		=	0x02,
	GYRO_ONLY 		=	0x03,
	ACC_MAG 		=	0x04,
	ACC_GYRO 		=	0x05,
	MAG_GYRO 		=	0x06,
	AMG				= 	0x07,
	//Fusion modes
	IMU				= 	0x08,
	COMPASS 		=	0x09,
	M4G				= 	0x0A,
	NDOF_FMC_OFF 	=	0x0B,
	NDOF			= 	0x0C
}OP_Mode_t;


/** ----------------------------------------------------------------------------------------------------
  * 			 					  BNO055 status macros definition
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct{
	uint8_t STresult;	//First 4 bit[0:3] indicates self test resulst 3:ST_MCU, 2:ST_GYRO, 1:ST_MAG, 0:ST_ACCEL
	uint8_t SYSError;	//Contains system error type If SYSStatus is System_Error(0x01)
	uint8_t SYSStatus;
}BNO_Status_t;

typedef struct{
	uint8_t System;
	uint8_t Gyro;
	uint8_t Acc;
	uint8_t MAG;
}Calib_status_t;

/** ----------------------------------------------------------------------------------------------------
  * 			 BNO055 Data output structures and  macros definition for ReadData function
  * ----------------------------------------------------------------------------------------------------
  */
typedef struct{ //SENSOR DATA AXIS X, Y and Z
	float X;
	float Y;
	float Z;
}BNO055_Data_XYZ_t;

typedef struct{ //SENSOR DATA AXIS W, X, Y and Z (Only for quaternion data)
	float W;
	float X;
	float Y;
	float Z;
}BNO055_QuaData_WXYZ_t;


typedef struct{ //SENSOR DATAS
	BNO055_Data_XYZ_t Accel;
	BNO055_Data_XYZ_t Gyro;
	BNO055_Data_XYZ_t Magneto;
	BNO055_Data_XYZ_t Magneto_CAL;
	BNO055_Data_XYZ_t Euler;
	BNO055_Data_XYZ_t LineerAcc;
	BNO055_Data_XYZ_t Gravity;
	BNO055_QuaData_WXYZ_t Quaternion;

	//algorithm compute
	BNO055_QuaData_WXYZ_t Quaternion_MAD;
	BNO055_QuaData_WXYZ_t Quaternion_MY;

	BNO055_Data_XYZ_t Euler_MAD;
	BNO055_Data_XYZ_t Euler_MY;

	//position estimation
	BNO055_Data_XYZ_t Linear_ACC;
	BNO055_Data_XYZ_t Linear_Vel;
	BNO055_Data_XYZ_t Position;

	//Stance detecter
	bool Stance;
	float AngRate_Eng;
}BNO055_Sensors_t;

typedef enum {
    SENSOR_GRAVITY     = 0x01,
    SENSOR_QUATERNION  = 0x02,
    SENSOR_LINACC      = 0x04,
    SENSOR_GYRO        = 0x08,
    SENSOR_ACCEL       = 0x10,
    SENSOR_MAG         = 0x20,
    SENSOR_EULER       = 0x40,
} BNO055_Sensor_Type;



//Base Addresses of output data register
#define BNO_ACCEL 					ACCEL_DATA_BASEADDR
#define BNO_GYRO 					GYRO_DATA_BASEADDR
#define BNO_MAG 					MAG_DATA_BASEADDR
#define BNO_EULER 					EUL_DATA_BASEADDR
#define BNO_LINACC 					LIA_DATA_BASEADDR
#define BNO_GRAVITY					GRV_DATA_BASEADDR
#define BNO_QUATERNION				QUA_DATA_BASEADDR

//===========================================================================================================================

//PWR_MODE
#define BNO055_NORMAL_MODE			0
#define BNO055_LOWPOWER_MODE		1
#define BNO055_SUSPEND_MODE			2

/*
 * CLK Source
 */
#define CLOCK_EXTERNAL				(1 << 7)
#define CLOCK_INTERNAL				(0 << 7)

/*
 * Page macros
 */
#define PAGE_0 						0x00
#define PAGE_1 						0x01

/*
 * Operating mode switching time (table 3-6)
 */
#define BNO_CONFIG_TIME_DELAY 		7   //(ms)  From CONFIGMODE mode to any operation mode
#define BNO_ANY_TIME_DELAY    		19  //(ms)  From any operation mode to CONFIGMODE

//Frame
#define NED_FRAME					0
#define ENU_FRAME					1

//Axis remap and Axis sign
#define DEFAULT_AXIS_REMAP			0x24
#define DEFAULT_AXIS_SIGN			0x00

#define P5_AXIS_REMAP				0x21
#define P5_AXIS_SIGN				0x01

/*
 * UNIT_SELECT register macros definition
 */
#define UNIT_ORI_ANDROID 			(1 << 7)
#define UNIT_ORI_WINDOWS 			(0 << 7)
#define UNIT_TEMP_CELCIUS 			(0 << 4)
#define UNIT_TEMP_FAHRENHEIT 		(1 << 4)
#define UNIT_EUL_DEG 				(0 << 2)
#define UNIT_EUL_RAD 				(1 << 2)
#define UNIT_GYRO_DPS 				(0 << 1)
#define UNIT_GYRO_RPS 				(1 << 1)
#define UNIT_ACC_MS2				(0 << 0)
#define UNIT_ACC_MG 				(1 << 0)


/*
 * ACCEL range macros definition for ACC_CONFIG register
 */
#define ACC_RANGE_2G 					0x00
#define ACC_RANGE_4G 					0x01
#define ACC_RANGE_8G 					0x02
#define ACC_RANGE_16G 					0x03

/*
 * GYRO range macros definition for GYR_CONFIG register
 */
#define GYR_RANGE_2000_DPS 					0x00
#define GYR_RANGE_1000_DPS  				0x01
#define GYR_RANGE_500_DPS  					0x02
#define GYR_RANGE_250_DPS  					0x03
#define GYR_RANGE_125_DPS  					0x04


///////Scale resolutions per LSB for the sensors////
//Accelerometer
#define ACC_MS2_RES 					100.0
#define ACC_MG_RES   					1.0
//Magnetometer
#define MAG_UT_RES						16.0
//Gyroscope
#define GYRO_DPS_RES 					16.0
#define GYRO_RPS_RES   					900.0
//Euler angles
#define EUL_DEG_RES 					16.0
#define EUL_RAD_RES   					900.0
//Quaternion
#define QUAT_RES						(1<<14) //1 Quaternion = 2^14 LSB
//Linear Acceleration
#define LIA_MS2_RES 					100.0
#define LIA_MG_RES   					1.0
//Gravity Vector
#define GRV_MS2_RES 					100.0
#define GRV_MG_RES   					1.0


/*
 * BNO055 API declaration
 */
void BNO055_Init(BNO055_Handle_t *pBNO055Handle);
void ReadData( BNO055_Handle_t *pBNO055Handle, BNO055_Sensors_t *sensorData,BNO055_Sensor_Type sensors);
/*
 * BNO055 helper function
 */
void Set_Operation_Mode(OP_Mode_t OP_Mode);
void SelectPage(uint8_t page);
void ResetBno055(void);
void Clock_Source(uint8_t source);
void SetPowerMODE(uint8_t power_mode);
void Check_Status(BNO_Status_t *result);
OP_Mode_t getCurrentMode(void);

//estimation
void Magwick_Setup(Madgwick_Handle_t *hMadgwick);
void Madgwick_Update(BNO055_Sensors_t *sensorData, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ecompass(BNO055_Sensors_t *sensorData, float ax, float ay, float az, float mx, float my, float mz);

//position estimation
void stance_detector(BNO055_Sensors_t *sensorData);

void getCalibration(Calib_status_t *calib);
_Bool isFullyCalibrated(void);
void setSensorOffsets(const uint8_t *calibData) ;
void getSensorOffsets(uint8_t *calibData);
_Bool Calibrate_BNO055(void);
#endif /* BNO055_H_ */

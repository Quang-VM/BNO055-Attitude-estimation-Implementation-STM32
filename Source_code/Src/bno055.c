/*
 * bno055.c
 *
 *  Created on: Sep 10, 2024
 *      Author: HQLap
 */

#include "main.h"

//Global variable
float deltat;
float beta;
uint8_t frame;


float slide_window[WINDOW_SIZE];
int current_index = 0;
int num_samples = 0;
float cumulative_sum_energy = 0.0f;


// Magnetic bias vector
float Mag_Bias[3] = {69.127276f, -59.899396f, 161.283932f};

// Magnetic scaling factor matrix
float Mag_ScFactor[3][3] = {
    {1.102381f, 0.021149f, 0.032939f},
    {0.021149f, 1.007658f, -0.108953f},
    {0.032939f, -0.108953f, 1.119184f}
};

float Gyro_Bias[3] = {-0.002371f, -0.003778f, 0.003331f};


/**
  * @brief  Initialization of BNO055
  *
  * @param  Init argument to a BNO055_Init_t structure that contains
  *         the configuration information for the BNO055 device.
  *
  * @retval None
  */
void BNO055_Init(BNO055_Handle_t *pBNO055Handle){

	uint8_t chip_id = 0;

	// Check Chip ID
	if ( HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, CHIP_ID_ADDR, 1, &chip_id, 1, bno055_timeout) != HAL_OK) {
		SERIAL_Printf("Error with the Bno055 I2C Address!\r\n");
		Error_handler();
	}else{
		while(chip_id != BNO055_ID) {
			SERIAL_Printf("Undefined chip ID! \r\n");
			Error_handler();
		}
	}

	// Set operation mode to config_mode for initialize all register
	Set_Operation_Mode(CONFIG_MODE);

	/*   Set register page number to 1
	 * 	 Configure Accelerometer, Gyroscope range
	 */
	SelectPage(PAGE_1);
	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, ACC_CONFIG_ADDR, 1, &pBNO055Handle->ACC_Range, 1, bno055_timeout);
	HAL_Delay(100);

	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, GYRO_CONFIG_0_ADDR, 1, &pBNO055Handle->GYR_Range, 1, bno055_timeout);
	HAL_Delay(100);

	// Read clock status. If status=0 then it is free to configure the clock source
	SelectPage(PAGE_0);
	HAL_Delay(50);

	uint8_t status;
	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, SYS_CLK_STATUS_ADDR, 1, &status, 1, 100);
	HAL_Delay(50);
	//Checking if the status bit is 0
	if(status == 0)
	{
		//Changing clock source
		Clock_Source(pBNO055Handle->Clock_Source);
		HAL_Delay(1000);
	}

	// Configure axis remapping and signing
	if (pBNO055Handle->Frame == NED_FRAME) {
	    uint8_t p5_axis_remap = P5_AXIS_REMAP; // Create a pointer for the remap value
	    HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, AXIS_MAP_CONFIG_ADDR, 1, &p5_axis_remap, 1, bno055_timeout);
	    HAL_Delay(100);

	    uint8_t p5_axis_sign = P5_AXIS_SIGN; // Create a pointer for the sign value
	    HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, AXIS_MAP_SIGN_ADDR, 1, &p5_axis_sign, 1, bno055_timeout);
	    HAL_Delay(20);
	} else if (pBNO055Handle->Frame == ENU_FRAME) {
	    uint8_t default_axis_remap = DEFAULT_AXIS_REMAP; // Create a pointer for the remap value
	    HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, AXIS_MAP_CONFIG_ADDR, 1, &default_axis_remap, 1, bno055_timeout);
	    HAL_Delay(100);

	    uint8_t default_axis_sign = DEFAULT_AXIS_SIGN; // Create a pointer for the sign value
	    HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, AXIS_MAP_SIGN_ADDR, 1, &default_axis_sign, 1, bno055_timeout);
	    HAL_Delay(20);
	} else {
	    SERIAL_Printf("Unknown frame type ");
	    Error_handler();
	}

	//Configure data output format and the measurement unit
	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, UNIT_SEL_ADDR, 1, &pBNO055Handle->Unit_Sel, sizeof(pBNO055Handle->Unit_Sel), bno055_timeout);
    HAL_Delay(100);

	//Set power mode
	SetPowerMODE(pBNO055Handle->PWR_Mode);
	HAL_Delay(100);

	//Set operation mode
	Set_Operation_Mode(pBNO055Handle->OP_Mode);
	HAL_Delay(100);

	SERIAL_Printf("BNO055 Initialization process is done! \r\n");
}


/*!
 *   @brief  Reads various data measured by BNO055
 *
 *   @param  Register base address of the data to be read
 * 			Possible arguments
 * 			[SENSOR_ACCEL
 *			 SENSOR_GYRO
 * 			 SENSOR_MAG
 *			 SENSOR_EULER
 *			 SENSOR_LINACC
 *			 SENSOR_GRAVITY
 *			 SENSOR_QUATERNION]
 *
 *   @retval Structure containing the values ​​of the read data
 */
void ReadData(BNO055_Handle_t *pBNO055Handle, BNO055_Sensors_t *sensorData, BNO055_Sensor_Type sensors){

	uint8_t buffer[8];

	if (sensors & SENSOR_GRAVITY) {
		HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_GRAVITY, 1, buffer, 6, bno055_timeout );
		sensorData->Gravity.X = (float)((int16_t)((buffer[1] << 8) | (buffer[0])) / GRV_MS2_RES);
		sensorData->Gravity.Y = (float)((int16_t)((buffer[3] << 8) | (buffer[2])) / GRV_MS2_RES);
		sensorData->Gravity.Z = (float)((int16_t)((buffer[5] << 8) | (buffer[4])) / GRV_MS2_RES);
		memset(buffer, 0, sizeof(buffer));
	}

    if (sensors & SENSOR_QUATERNION) {

    	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_QUATERNION, 1, buffer, 8, bno055_timeout);
        sensorData->Quaternion.W = (float)(((int16_t)((buffer[1] << 8) | buffer[0])) / QUAT_RES);
        sensorData->Quaternion.X = (float)(((int16_t)((buffer[3] << 8) | buffer[2])) / QUAT_RES);
        sensorData->Quaternion.Y = (float)(((int16_t)((buffer[5] << 8) | buffer[4])) / QUAT_RES);
        sensorData->Quaternion.Z = (float)(((int16_t)((buffer[7] << 8) | buffer[6])) / QUAT_RES);
        memset(buffer, 0, sizeof(buffer));
    }

    if (sensors & SENSOR_LINACC) {

    	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_LINACC, 1, buffer, 6, bno055_timeout);
        sensorData->LineerAcc.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0])) / LIA_MS2_RES);
        sensorData->LineerAcc.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2])) / LIA_MS2_RES);
        sensorData->LineerAcc.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4])) / LIA_MS2_RES);
        memset(buffer, 0, sizeof(buffer));
    }

    if (sensors & SENSOR_GYRO) {

    	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_GYRO, 1, buffer, 6, bno055_timeout);
        sensorData->Gyro.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0])) / GYRO_RPS_RES);
        sensorData->Gyro.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2])) / GYRO_RPS_RES);
        sensorData->Gyro.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4])) / GYRO_RPS_RES);

        sensorData->Gyro.X = sensorData->Gyro.X - Gyro_Bias[0];
        sensorData->Gyro.Y = sensorData->Gyro.Y - Gyro_Bias[1];
        sensorData->Gyro.Z = sensorData->Gyro.Z - Gyro_Bias[2];

        memset(buffer, 0, sizeof(buffer));

        //sensorData->AngRate_Eng = sensorData->Gyro.X * sensorData->Gyro.X + sensorData->Gyro.Y * sensorData->Gyro.Y + sensorData->Gyro.Z * sensorData->Gyro.Z;
        //stance_detector(BNO055_Sensors_t *sensorData);
    }

    if (sensors & SENSOR_ACCEL) {

    	if (pBNO055Handle->Frame == NED_FRAME) {
            HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_ACCEL, 1, buffer, 6, bno055_timeout);
            sensorData->Accel.X = -(float)(((int16_t) ((buffer[1] << 8) | buffer[0])) / ACC_MS2_RES);
            sensorData->Accel.Y = -(float)(((int16_t) ((buffer[3] << 8) | buffer[2])) / ACC_MS2_RES); //NED frame, negative the x & y axis
            sensorData->Accel.Z = -(float)(((int16_t) ((buffer[5] << 8) | buffer[4])) / ACC_MS2_RES);
    	}else {
    		sensorData->Accel.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0])) / ACC_MS2_RES);
    		sensorData->Accel.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2])) / ACC_MS2_RES);
    		sensorData->Accel.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4])) / ACC_MS2_RES);
    	}

        memset(buffer, 0, sizeof(buffer));
    }

    if (sensors & SENSOR_MAG) {

    	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_MAG, 1, buffer, 6, bno055_timeout);
        sensorData->Magneto.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0])) / MAG_UT_RES);
        sensorData->Magneto.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2])) / MAG_UT_RES);
        sensorData->Magneto.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4])) / MAG_UT_RES);

        // axulirary variables to avoid reapeated calcualtions
        float mxsubmbx = sensorData->Magneto.X - Mag_Bias[0];
        float mysubmby = sensorData->Magneto.Y - Mag_Bias[1];
        float mzsubmbz = sensorData->Magneto.Z - Mag_Bias[2];

        sensorData->Magneto_CAL.X = Mag_ScFactor[0][0] * mxsubmbx
                                  + Mag_ScFactor[0][1] * mysubmby
                                  + Mag_ScFactor[0][2] * mzsubmbz;
        sensorData->Magneto_CAL.Y = Mag_ScFactor[1][0] * mxsubmbx
                                  + Mag_ScFactor[1][1] * mysubmby
                                  + Mag_ScFactor[1][2] * mzsubmbz;
        sensorData->Magneto_CAL.Z = Mag_ScFactor[2][0] * mxsubmbx
                                  + Mag_ScFactor[2][1] * mysubmby
                                  + Mag_ScFactor[2][2] * mzsubmbz;

        memset(buffer, 0, sizeof(buffer));
    }
    if (sensors & SENSOR_EULER) {

    	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, BNO_EULER, 1, buffer, 6, bno055_timeout);
        sensorData->Euler.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0])) / EUL_DEG_RES);
        sensorData->Euler.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2])) / EUL_DEG_RES);
        sensorData->Euler.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4])) / EUL_DEG_RES);
        memset(buffer, 0, sizeof(buffer));
    }
}


// Function to determine the "stance" state based on angular rate energy
// Input: Pointer to the BNO055_Sensors_t structure containing sensor data
void stance_detector(BNO055_Sensors_t *sensorData) {
    // Retrieve the current angular energy from the sensor data
    float current_energy = sensorData->AngRate_Eng;

    // If the number of samples is less than the window size
    if (num_samples < WINDOW_SIZE) {
        // Accumulate the current energy into the cumulative energy total
        cumulative_sum_energy += current_energy;
        num_samples++;  // Increment the sample count
    } else {
        // If the required number of samples is reached, update the cumulative energy by replacing the old value
        float old_energy = slide_window[current_index];
        cumulative_sum_energy = cumulative_sum_energy - old_energy + current_energy;
    }

    // Update the current energy value in the sliding window
    slide_window[current_index] = current_energy;
    // Update the current index for the sliding window
    current_index = (current_index + 1) % WINDOW_SIZE;

    // Calculate the value T based on the variance and cumulative energy
    float T = (1.0f / (SIGMA_OMEGA * SIGMA_OMEGA * WINDOW_SIZE)) * cumulative_sum_energy;

    // Determine the "stance" state based on the value of T
    if (T < GAMMA_OMEGA) {
        sensorData->Stance = true;  // If T is less than the threshold, set the state to true
    } else {
        sensorData->Stance = false; // Otherwise, set the state to false
    }
}

void position_estimation(BNO055_Sensors_t *sensorData) {
	//Rotate body accelerations to Earth frame

}




/*!
 *   @brief  Gets the latest system status info
 *
 *   @param  BNO_status_t structure that contains status information
 *           STresult, SYSError and SYSStatus
 *
 *   @retval None
 */
void Check_Status(BNO_Status_t *result){

	HAL_StatusTypeDef status;
	uint8_t value;

	  /* Self Test Results
	     1 = test passed, 0 = test failed

	     Bit 0 = Accelerometer self test
	     Bit 1 = Magnetometer self test
	     Bit 2 = Gyroscope self test
	     Bit 3 = MCU self test

	     0x0F = all good!
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, ST_RESULT_ADDR, 1, &value, 1, bno055_timeout);
	if (status != HAL_OK) {
	    SERIAL_Printf("I2C Read Error: ST_RESULT_ADDR \r\n");
	}
	HAL_Delay(50);
	result->STresult = value;
	value=0;

	  /* System Status (see section 4.3.58)
	     0 = Idle
	     1 = System Error
	     2 = Initializing Peripherals
	     3 = System Iniitalization
	     4 = Executing Self-Test
	     5 = Sensor fusio algorithm running
	     6 = System running without fusion algorithms
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, SYS_STATUS_ADDR, 1, &value, 1, bno055_timeout);
	if (status != HAL_OK) {
	    SERIAL_Printf("I2C Read Error: SYS_STATUS_ADDR \r\n");
	}
	HAL_Delay(50);
	result->SYSStatus = value;
	value=0;
	  /* System Error (see section 4.3.59)
	     0 = No error
	     1 = Peripheral initialization error
	     2 = System initialization error
	     3 = Self test result failed
	     4 = Register map value out of range
	     5 = Register map address out of range
	     6 = Register map write error
	     7 = BNO low power mode not available for selected operation mode
	     8 = Accelerometer power mode not available
	     9 = Fusion algorithm configuration error
	     A = Sensor configuration error
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, SYS_ERR_ADDR, 1, &value, 1, bno055_timeout);
	if (status != HAL_OK) {
	    SERIAL_Printf("I2C Read Error: SYS_ERR_ADDR \r\n");
	}
	HAL_Delay(50);
	result->SYSError = value;
}


/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  Operation modes
 *          Mode Values;
 *          CONFIG_MODE
 *          ACC_ONLY
 *          MAG_ONLY
 *          GYR_ONLY
 *          ACC_MAG
 *          ACC_GYRO
 *          MAG_GYRO
 *          AMG
 *          IMU
 *          COMPASS
 *          M4G
 *          NDOF_FMC_OFF
 *          NDOF
 *  @retval None
 */
void Set_Operation_Mode(OP_Mode_t OP_Mode){

	// 1. Select Page 0
	SelectPage(PAGE_0);

	// 2. Switch mode
	if(	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, OPR_MODE_ADDR, 1, (uint8_t*)&OP_Mode, 1, bno055_timeout) !=HAL_OK){
		SERIAL_Printf("Operation mode could not be set \r\n");
		Error_handler();
	}

	// 3. Wait for switching
	if(OP_Mode == CONFIG_MODE) HAL_Delay(BNO_ANY_TIME_DELAY + 5);
	else HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
}


/*!
 *   @brief  Changes register page
 *
 *   @param  Page number
 *   		Possible Arguments
 * 			[PAGE_0
 * 			 PAGE_1]
 *
 * 	 @retval None
 */
void SelectPage(uint8_t page){
	if ( HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, PAGE_ID_ADDR, 1, &page, 1, bno055_timeout) != HAL_OK ) {
		SERIAL_Printf("Register page replacement could not be set \r\n");
		Error_handler();
	}
}


/**
  * @brief  Software Reset to BNO055
  *
  * @param  None
  *
  * @retval None
  */
void ResetBno055(void) {

	uint8_t reset = 0x20;
	if ( HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, SYS_TRIGGER_ADDR, 1, &reset, 1, bno055_timeout) != HAL_OK) {
		SERIAL_Printf("Can't reset Bno055 \r\n");
		Error_handler();
	}
	HAL_Delay(1000);
}

/*!
 *  @brief  Selects the chip's clock source
 *  @param  Source
 *          possible values
 *           [CLOCK_EXTERNAL
 *            CLOCK_INTERNAL]
 *
 *  @retval None
 */
void Clock_Source(uint8_t source) {

	SelectPage(PAGE_0);

	//7th bit: External Crystal=1; Internal Crystal=0
	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, SYS_TRIGGER_ADDR, 1, &source, sizeof(source), bno055_timeout);
}

/*!
 *  @brief  Set the power mode of BNO055
 *  @param  power modes
 *          possible values
 *           [BNO055_NORMAL_MODE
 *            BNO055_LOWPOWER_MODE
 *            BNO055_SUSPEND_MODE]
 *
 *  @retval None
 */
void SetPowerMODE(uint8_t power_mode){

	SelectPage(PAGE_0);

	if(	HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, PWR_MODE_ADDR, 1, &power_mode, 1, bno055_timeout) != HAL_OK)
	{
		SERIAL_Printf("Power mode could not be set!\r\n");
		Error_handler();
	}
	HAL_Delay(50);
}


/**
  * @brief  Calibrates BNO055
  *
  * @param  None
  *
  * @retval None
  *
  */
_Bool Calibrate_BNO055(void) {

		char msg[100];

		Calib_status_t calib={0};
        SERIAL_Printf("Calibrating BNO055 sensor...\r\n");

        // Set operation mode to FUSION_MODE or appropriate mode for calibration
        Set_Operation_Mode(NDOF);
    	HAL_Delay(100);
        // Gyroscope calibration
        SERIAL_Printf("Calibrating gyroscope...\r\n");
        SERIAL_Printf("Place the device in a single stable position in 10s\r\n");
        HAL_Delay(10000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
        	sprintf(msg, "GYRO CAL_STAT: %d\n", calib.Gyro);
        	SERIAL_Printf(msg);
		} while (calib.Gyro !=3);
        SERIAL_Printf("Gyroscope calibration complete.\r\n");

        // Accelerometer calibration
        SERIAL_Printf("Calibrating accelerometer...\r\n");
        SERIAL_Printf("Place the device in 6 different stable positions (each position in 20s)\r\n");
        for (int i = 0; i < 6; i++) {
        	sprintf(msg, "Position: %d\n", i + 1);
        	SERIAL_Printf(msg);
            HAL_Delay(20000);  // Simulated accelerometer calibration time
        }

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
        	sprintf(msg, "ACC CAL_STAT: %d\n", calib.Acc);
        	SERIAL_Printf(msg);
		} while (calib.Acc !=3);
        SERIAL_Printf("Accelerometer calibration complete.\r\n");

        // Magnetometer calibration
        SERIAL_Printf("Calibrating magnetometer...\r\n");
        SERIAL_Printf("Make some random movements in 35s \r\n");
        HAL_Delay(35000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
        	sprintf(msg, "Mag CAL_STAT: %d\n", calib.MAG);
        	SERIAL_Printf(msg);
		} while (calib.MAG !=3);
        SERIAL_Printf("Magnetometer calibration complete. \r\n");

        // System calibration
        SERIAL_Printf("Calibrating system...\r\n");
        SERIAL_Printf("Keep the device stationary until system calibration reaches level 3\r\n");
        do {
            getCalibration(&calib);
        	HAL_Delay(500);
        	sprintf(msg, "SYS CAL_STAT: %d\n", calib.System);
        	SERIAL_Printf(msg);
		} while (calib.System !=3);
        HAL_Delay(500);

        // Check calibration status
        while(!isFullyCalibrated()) HAL_Delay(500);
        SERIAL_Printf("Sensor is fully calibrated.\r\n");

    	sprintf(msg, "System: %d      Gyro: %d       Accel: %d       MAG: %d\n", calib.System,calib.Gyro , calib.Acc, calib.MAG);
    	SERIAL_Printf(msg);

        if(isFullyCalibrated()) return true;
        else return false;
}

/**
  * @brief  Gets calibration status of accel, gyro, mag and system
  *
  * @param  None
  *
  * @retval Calib_status_t structure that contains
  *         the calibration status of accel, gyro, mag and system.
  */
void getCalibration(Calib_status_t *calib) {
    uint8_t calData;

    // Read calibration status register using I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR, CALIB_STAT_ADDR, 1, &calData, 1, HAL_MAX_DELAY);

    // Check if read was successful
    if (status == HAL_OK) {

        // Extract calibration status values

        	calib->System= (calData >> 6) & 0x03;


        	calib->Gyro = (calData >> 4) & 0x03;


        	calib->Acc = (calData >> 2) & 0x03;


        	calib->MAG = calData & 0x03;

    } else {
    	SERIAL_Printf("Failed to read calibration status register.\r\n");
    }
}


/**
  * @brief  Sets sensor offsets
  *
  * @param  22 byte long buffer containing offset data
  *
  * @retval None
  *
  */
void setSensorOffsets(const uint8_t *calibData) {
    uint8_t lastMode = getCurrentMode();

    // Switch to CONFIG mode
    Set_Operation_Mode(CONFIG_MODE);
    SERIAL_Printf("Switched to CONFIG mode.\n");

    // Write calibration data to the sensor's offset registers using memory write
    HAL_I2C_Mem_Write(&bno_i2c, BNO055_ADDR, ACC_OFFSET_X_LSB_ADDR, 1, (uint8_t *)calibData, 22, 100);
    SERIAL_Printf("Wrote calibration data to sensor's offset registers.\n");

    // Restore the previous mode
    Set_Operation_Mode(lastMode);
    SERIAL_Printf("Restored to previous mode.\n");
}

/**
  * @brief  Checks the calibration status of the sensor
  *
  * @param  None
  *
  * @retval True of False
  *
  */
_Bool isFullyCalibrated(void) {
//    Calib_status_t calib ={0};
    Calib_status_t calib ={0};
    getCalibration(&calib);


    switch (getCurrentMode()) {
        case ACC_ONLY:
            return (calib.Acc == 3);
        case MAG_ONLY:
            return (calib.MAG == 3);
        case GYRO_ONLY:
        case M4G: /* No magnetometer calibration required. */
            return (calib.Gyro == 3);
        case ACC_MAG:
        case COMPASS:
            return (calib.Acc == 3 && calib.MAG == 3);
        case ACC_GYRO:
        case IMU:
            return (calib.Acc == 3 && calib.Gyro == 3);
        case MAG_GYRO:
            return (calib.MAG == 3 && calib.Gyro == 3);
        default:
            return (calib.System == 3 && calib.Gyro == 3 && calib.Acc == 3 && calib.MAG == 3);
    }
}

/**
  * @brief  Gets the current operating mode of the chip
  *
  * @param  None
  *
  * @retval Operating mode
  *
  */
OP_Mode_t getCurrentMode(void) {

	OP_Mode_t mode;

	HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, OPR_MODE_ADDR, 1, &mode, 1, 100);

    return mode;
}

/**
  * @brief  Gets sensor offsets
  *
  * @param  22 byte long buffer to hold offset data
  *
  * @retval None
  *
  */
void getSensorOffsets(uint8_t *calibData) {

        // Save the current mode
        uint8_t lastMode = getCurrentMode();

        // Switch to CONFIG mode
        Set_Operation_Mode(CONFIG_MODE);
        SERIAL_Printf("Switched to CONFIG mode.\r\n");

        // Read the offset registers
        HAL_I2C_Mem_Read(&bno_i2c, BNO055_ADDR, ACC_OFFSET_X_LSB_ADDR, 1, calibData, 22, 100);
        SERIAL_Printf("Calibration data obtained.\r\n");

        // Restore the previous mode
        Set_Operation_Mode(lastMode);
        SERIAL_Printf("Restored to previous mode.\r\n");
}


void Magwick_Setup(Madgwick_Handle_t *hMadgwick) {

	frame = hMadgwick->frame;
	deltat = hMadgwick->deltat;
	beta = hMadgwick->beta;
}

void Madgwick_Update(BNO055_Sensors_t *sensorData, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){

	float q1 = sensorData->Quaternion_MAD.W, q2 = sensorData->Quaternion_MAD.X, q3 = sensorData->Quaternion_MAD.Y, q4 = sensorData->Quaternion_MAD.Z;   // short name local variable for readability
    float norm;
    float qm1, qm2, qm3, qm4; //quaternion product of q and m
	float h0, hx, hy, hz;
	float bx, by, bz;
	float f1, f2, f3, f4, f5, f6;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;


	 // axulirary variables to avoid reapeated calcualtions
	 float halfq1 = 0.5f * q1;
	 float halfq2 = 0.5f * q2;
	 float halfq3 = 0.5f * q3;
	 float halfq4 = 0.5f * q4;
	 float twoq1 = 2.0f * q1;
	 float twoq2 = 2.0f * q2;
	 float twoq3 = 2.0f * q3;
	 float twoq4 = 2.0f * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    q_prod(q1, q2, q3, q4, 0, mx, my, mz, &qm1, &qm2, &qm3, &qm4);
    q_prod(qm1, qm2, qm3, qm4, q1, -q2, -q3, -q4, &h0, &hx, &hy, &hz);

    if (frame == NED_FRAME) {
        bx = sqrtf(hx * hx + hy * hy);
        bz = hz;

        // axulirary variables to avoid reapeated calcualtions
		float twobx = 2.0f * bx;
		float twobz = 2.0f * bz;
		float twobxq1 = twobx * q1;
		float twobxq2 = twobx * q2;
		float twobxq3 = twobx * q3;
		float twobxq4 = twobx * q4;
		float twobzq1 = twobz * q1;
		float twobzq2 = twobz * q2;
		float twobzq3 = twobz * q3;
		float twobzq4 = twobz * q4;

		float J11, J12orJ23, J13, J14orJ21,
		      J22, J24,
		      J31orJ34, J32, J33,
		      J41, J42, J43, J44,
		      J51, J52, J53, J54,
		      J61, J62, J63, J64;

        // compute the objective function
        f1 = twoq2 * q4 - twoq1 * q3 - ax;
        f2 = twoq1 * q2 + twoq3 * q4 - ay;
        f3 = 1.0f - twoq2 * q2 - twoq3 * q3 - az;
        f4 = 0.5f * twobx - twobxq3 * q3 - twobxq4 * q4 + twobzq2 * q4 - twobzq1 * q3 - mx;
        f5 = twobxq2 * q3 - twobxq4 + twobzq1 * q2 + twobzq3 * q4 - my;
        f6 = twobxq1 * q3 + twobxq2 * q4 + 0.5f * twobz - twobzq2 * q2 - twobzq3 * q3 - mz;

        // compute the Jacobian
        J11 = -twoq3;
        J12orJ23 = twoq4;
        J13 = -twoq1;
        J14orJ21 = twoq2;    //(common name for J14 and J21)

        // J21 has been computed above as J14orJ21
        J22 = twoq1;
        // J23 has been computed above as J12orJ23
        J24 = twoq3;

        J31orJ34 = 0.0f;
        J32 = -4.0f * q2;
        J33 = -4.0f * q3;

        J41 = -twobzq3;
        J42 = twobzq4;
        J43 = -2.0f * twobxq3 - twobzq1;
        J44 = -2.0f * twobxq4 + twobzq2;

        J51 = -twobxq4 + twobzq2;
        J52 = twobxq3 + twobzq1;
        J53 = twobxq2 + twobzq4;
        J54 = -twobxq1 + twobzq3;

        J61 = twobxq3;
        J62 = twobxq4 - 2.0f * twobzq2;
        J63 = twobxq1 - 2.0f * twobzq3;
        J64 = twobxq2;

        // compute the gradient (matrix multiplication)
        s1 = J11 * f1 		+ 		J14orJ21 * f2 		+ J31orJ34 * f3 		+ J41 * f4 		+ J51 * f5 		+ J61 * f6;
        s2 = J12orJ23 * f1 	+ 		J22 * f2 			+ J32 * f3 				+ J42 * f4 		+ J52 * f5 		+ J62 * f6;
        s3 = J13 * f1 		+ 		J12orJ23 * f2 		+ J33 * f3 				+ J43 * f4 		+ J53 * f5 		+ J63 * f6;
        s4 = J14orJ21 * f1 	+ 		J24 * f2 			+ J31orJ34 * f3 		+ J44 * f4 		+ J54 * f5 		+ J64 * f6;

        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

    } else if (frame == ENU_FRAME) {
    	by = sqrtf(hx * hx + hy * hy);
    	bz = hz;

        // axulirary variables to avoid reapeated calcualtions
		float twoby = 2.0f * by;
		float twobz = 2.0f * bz;
		float twobyq1 = twoby * q1;
		float twobyq2 = twoby * q2;
		float twobyq3 = twoby * q3;
		float twobyq4 = twoby * q4;
		float twobzq1 = twobz * q1;
		float twobzq2 = twobz * q2;
		float twobzq3 = twobz * q3;
		float twobzq4 = twobz * q4;

		// compute the objective function
		f1 = twoq1 * q3 - twoq2 * q4 - ax;
		f2 = -twoq1 * q2 - twoq3 * q4 - ay;
		f3 = -1.0f + twoq2 * q2 + twoq3 * q3 -az;
		f4 = twobyq1 * q4 + twobyq2 * q3 - twobzq1 * q3 + twobzq2 * q4 - mx;
		f5 = 0.5f * twoby - twobyq2 * q2 - twobyq4 * q4 + twobzq1 * q2 + twobzq3 * q4 - my;
		f6 = twobyq3 * q4 - twobyq1 * q2 + 0.5f * twobz - twobzq2 * q2 - twobzq3 * q3 - mz;

		// Compute the Jacobian for ENU frame
		float J11 = twoq3;
		float J12or23 = -twoq4;
		float J13 = twoq1;
		float J14or21 = -twoq2;

		//float J21 = -2.0f * q2;
		float J22 = -twoq1;
		//float J23 = -2.0f * q4;
		float J24 = -twoq3;

		float J31or34 = 0.0f;
		float J32 = 2.0f * twoq2;
		float J33 = 2.0f * twoq3;
		//float J34 = 0.0f;

		float J41 = twobyq4 - twobzq3;
		float J42 = twobyq3 + twobzq4;
		float J43 = twobyq2 - twobzq1;
		float J44 = twobyq1 + twobzq2;

		float J51 = twobzq2;
		float J52 = -by * J32 + twobzq1;
		float J53 = twobzq4;
		float J54 = -2.0f * twobyq4 + twobzq3;

		float J61 = -twobyq2;
		float J62 = -twobyq1 - bz * J32;
		float J63 = twobyq4 - bz * J33;
		float J64 = twobyq3;

		s1 = J11 * f1 + J14or21 * f2 + J31or34 * f3 + J41 * f4 + J51 * f5 + J61 * f6;
		s2 = J12or23 * f1 + J22 * f2 + J32 * f3 + J42 * f4 + J52 * f5 + J62 * f6;
		s3 = J13 * f1 + J12or23 * f2 + J33 * f3 + J43 * f4 + J53 * f5 + J63 * f6;
		s4 = J14or21 * f1 + J24 * f2 + J31or34 * f3 + J44 * f4 + J54 * f5 + J64 * f6;

        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

    }

    // Compute rate of change of quaternion
    qDot1 = -halfq2 * gx - halfq3 * gy - halfq4 * gz - beta * s1;
    qDot2 =  halfq1 * gx + halfq3 * gz - halfq4 * gy - beta * s2;
    qDot3 =  halfq1 * gy - halfq2 * gz + halfq4 * gx - beta * s3;
    qDot4 =  halfq1 * gz + halfq2 * gy - halfq3 * gx - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    sensorData->Quaternion_MAD.W = q1 * norm;
    sensorData->Quaternion_MAD.X = q2 * norm;
    sensorData->Quaternion_MAD.Y = q3 * norm;
    sensorData->Quaternion_MAD.Z = q4 * norm;
}


/*
 * Orientation from accelerometer and magnetometer readings

    Parameters
    ----------
    a : Sample of tri-axial accelerometer, in m/s^2.
    m : Sample of tri-axial magnetometer, in uT.
 * */
void ecompass(BNO055_Sensors_t *sensorData, float ax, float ay, float az, float mx, float my, float mz) {
	float q0, q1, q2, q3;
	float RyX, RyY, RyZ;
	float RxX, RxY, RxZ;
	float norm;

	// Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    if (frame == NED_FRAME){
        // Ry = cross(Rz, m)
        RyX = ay * mz - az * my;
        RyY = az * mx - ax * mz;
        RyZ = ax * my - ay * mx;
        norm = sqrtf(RyX * RyX + RyY * RyY + RyZ * RyZ);
        if (norm == 0.0f) return; // handle NaN or degenerate case
        norm = 1.0f/norm;
        RyX *= norm;
        RyY *= norm;
        RyZ *= norm;

        // Rx = cross(Ry, Rz)
        RxX = RyY * az - RyZ * ay;
        RxY = RyZ * ax - RyX * az;
        RxZ = RyX * ay - RyY * ax;
        norm = sqrtf(RxX * RxX + RxY * RxY + RxZ * RxZ);
        if (norm == 0.0f) return; // handle NaN or degenerate case
        norm = 1.0f/norm;
        RxX *= norm;
        RxY *= norm;
        RxZ *= norm;

    } else {
        // Rx = cross(m, Rz);
        RxX = my * az - mz * ay;
        RxY = mz * ax - mx * az;
        RxZ = mx * ay - my * ax;
        norm = sqrtf(RxX * RxX + RxY * RxY + RxZ * RxZ);
        if (norm == 0.0f) return; // handle NaN or degenerate case
        norm = 1.0f/norm;
        RxX *= norm;
        RxY *= norm;
        RxZ *= norm;

        // Ry = cross(Rz, Rx);
        RyX = ay * RxZ - az * RxY;
        RyY = az * RxX - ax * RxZ;
        RyZ = ax * RxY - ay * RxX;
        norm = sqrtf(RyX * RyX + RyY * RyY + RyZ * RyZ);
        if (norm == 0.0f) return; // handle NaN or degenerate case
        norm = 1.0f/norm;
        RyX *= norm;
        RyY *= norm;
        RyZ *= norm;
    }

    // Convert to quaternion
	dcm2quat(RxX, RxY, RxZ,
			 RyX, RyY, RyZ,
			 ax,  ay,  az,
			 &q0, &q1, &q2, &q3);

	//store
	sensorData->Quaternion_MAD.W = q0;
	sensorData->Quaternion_MAD.X = q1;
	sensorData->Quaternion_MAD.Y = q2;
	sensorData->Quaternion_MAD.Z = q3;
}







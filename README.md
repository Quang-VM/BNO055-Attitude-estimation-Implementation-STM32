# Madgwick Algorithm implementation with Bosch's Bno055 sensor
Quaternion Based Attitude Estimation Using Bno055 and STM32F407
![](./img/Proposal.gif)

[DEMO](https://www.youtube.com/watch?v=e6xahf8quOc)

# Hardware
- [STM32F407G-DISC1](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)

- [ESP32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf) streaming data over serial instead of using USB-to-Serial Converter

- [BNO055](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) (probably made in China)

# Connections

| BNO055            | STM32 Pin   |   | STM32 Pin         | ESP32       |
|-------------------|-------------|---|-------------------|-------------|
| SCL               | PB10        |   | PA2 (UART2 TX)    | GPIO3 (RX)  |
| SDA               | PB11        |   | PA3 (UART2 RX)    | GPIO1 (TX)  |
| GND               | GND         |   |                   |             |
| VIN               | 3V          |   |                   |             |

# Frame Representation
To change the frame to ENU Frame, in Sensor_Init() function set:
```
handle_bno055.Frame = ENU_FRAME;				
```
in order to config the frame of BNO055 and Madgwick Algorithm also.

# Madgwick Algorithm
* The first estimation was calculated by ecompass() function (Accelerometer and Magnetometer Fusion)
* Delta time (default 0.0125s (80 Hz) -> use a timer interrupt)
    - To calculate the required delta time use the `timer_period_calculation.xlsx` file and change the parameters in TIMER6_Init() with this results
* Set up the filter Beta gain (default 0.68)

# Raw Measurement and Calibration
By default, this implementation uses raw accelerometer and gyroscope data along with custom calibrated magnetometer data.
To manually calibrate the magnetometer, follow these steps:
* In the Timer interrupt callback function, modify it as follows:
```
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htimer6){

		ReadData(&BNO055,  SENSOR_MAG);

	    len = snprintf(msg, sizeof(msg),
	        "%f,%f,%f\n",
			BNO055.Magneto.X, BNO055.Magneto.Y, BNO055.Magneto.Z);  
	    SERIAL_Printf(msg);
    }
}
```
* On the PC side, run the `log-mag.py` script and rotate the sensor as much as possible to collect the raw magnetometer data.
* Open [Magneto12.exe](https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html) software to find the Bias & Scale factor Matrices

![](./img/magneto-sf.png)

* Copy the 3x1 bias matrix (B) and the 3x3 inverse scale factor matrix (A⁻¹) into `plot-calibration-data.py` to get the comparison

![](./img/raw_cali_magnetometer.png)

* Update the 3x1 bias matrix (B) and the 3x3 inverse scale factor matrix (A⁻¹) on the STM32 side
```
// Magnetic bias vector
float Mag_Bias[3] = {69.127276f, -59.899396f, 161.283932f};

// Magnetic scaling factor matrix
float Mag_ScFactor[3][3] = {
    {1.102381f, 0.021149f, 0.032939f},
    {0.021149f, 1.007658f, -0.108953f},
    {0.032939f, -0.108953f, 1.119184f}
};
```

# BNO055's Automatic Background Calibration
The sensor fusion algorithm inside the Bno055 performs automatic background calibration of all sensors and cannot be disabled.
To get the automatic calibrated data change the Operation Mode to NDOF Mode:
```
handle_bno055.OP_Mode = NDOF;					// Sensor Fusion mode
```
If you wanna use the Madgwick algorithm with automatic calibrated data change the input of Update function:
```
ReadData(&handle_bno055, &BNO055, SENSOR_ACCEL| SENSOR_MAG);
ecompass(&BNO055, BNO055.Accel.X, BNO055.Accel.Y, BNO055.Accel.Z,
		BNO055.Magneto.X, BNO055.Magneto.Y, BNO055.Magneto.Z);

Madgwick_Update(&BNO055, BNO055.Accel.X, BNO055.Accel.Y, BNO055.Accel.Z,
					BNO055.Gyro.X, BNO055.Gyro.Y, BNO055.Gyro.Z,
					BNO055.Magneto.X, BNO055.Magneto.Y, BNO055.Magneto.Z);
```

# BNO55 Calibration
To calibrate Bno055, modify Sensor_Init function as follow:
```
//uint8_t OffsetDatas[22];
void Sensor_Init(void){

	// Reset Bno055
	ResetBno055();

	handle_bno055.ACC_Range = ACC_RANGE_4G;
	handle_bno055.GYR_Range = GYR_RANGE_1000_DPS;
	handle_bno055.Frame = NED_FRAME;
	handle_bno055.Clock_Source = CLOCK_EXTERNAL;		//CLOCK_EXTERNAL or CLOCK_INTERNAL
	handle_bno055.PWR_Mode = BNO055_NORMAL_MODE;		//BNO055_X_MODE   X:NORMAL, LOWPOWER, SUSPEND
	handle_bno055.OP_Mode = AMG;						//if the application requires only raw meas, this mode can be chosen.
	//handle_bno055.OP_Mode = NDOF;						// Sensor Fusion mode
	//handle_bno055.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_RPS | UNIT_ACC_MS2);
	 handle_bno055.Unit_Sel = ( UNIT_GYRO_RPS | UNIT_ACC_MS2);
	BNO055_Init(&handle_bno055);

	//------------------BNO055 Calibration------------------

    /*This function enables the calibration offset data gathered after the BNO055 sensor is calibrated to be saved 
 	*to its registers. This allows the sensor to retain its calibration settings, eliminating the need for recalibration
 	*each time it is powered on.*/
	//setSensorOffsets(OffsetDatas);

	/*-=-=-=-=-=-=Calibration Part-=-=-=-=-=-=*/
	if(Calibrate_BNO055())
	{
		getSensorOffsets(OffsetDatas);
	}
	else
	{
		SERIAL_Printf("Sensor calibration failed.\nFailed to retrieve offset data\n");
	}

	Check_Status(&Status);
	SERIAL_Printf("Selftest Result: %d\t",Status.STresult);
	SERIAL_Printf("System Status: %d\t",Status.SYSStatus);
	SERIAL_Printf("System Error: %d\n",Status.SYSError);
}
```
The Calibrate_BNO055() function returns True if the CALIB_STAT values for the system, gyroscope, accelerometer, and magnetometer are fully calibrated (calibration accuracy level 3). Details the [calibration instructions](https://www.youtube.com/watch?v=Bw0WuAyGsnY&t=92s) provided by Bosch.
* Acceleration Calibration
	- Place the device in 6 different stable positions for a period of few seconds to allow the accelerometer to calibrate.
	- Make sure that there is slow movement between 2 stable positions.
	- The 6 stable positions could be in any direction, but make sure that the device is lying at least once perpendicular to the x, y, and z axis.
* Gyroscope Calibration
	- Place the device in a single stable position for a period.
* Magnetometer Calibration
	- Make some random movements (for example: writing an 'infinity symbol' on air).

The BNO055 sensor must be calibrated each time it is turned on, or the offset data obtained after calibration must be stored in the offset registers. After calibrating the sensor, store the offset data in the `OffsetDatas` buffer, then remove the `setSensorOffsets(OffsetDatas);` function from comment line and add calibration part in the comment line.
# Embedded Sensor Fusion Algorithm Of the Bno055
To obtain output of the embedded sensor fusion algorithm modify source code as follow:
* Change Op_Mode to NDOF and config the unit:
```
handle_bno055.OP_Mode = NDOF;					// Sensor Fusion mode
handle_bno055.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_RPS | UNIT_ACC_MS2);
```
* Modify the Timer Callback function:
	- if Euler angles output:
```
ReadData(&BNO055, SENSOR_EULER);    
len = snprintf(msg, sizeof(msg), "%f,%f,%f\n",
				BNO055.Euler.Z,   
				BNO055.Euler.Y,  
				BNO055.Euler.X); 
SERIAL_Printf(msg);
```
 - if Quaternion output:
```
ReadData(&BNO055, SENSOR_QUATERNION);    
len = snprintf(msg, sizeof(msg), "%f,%f,%f,%f\n",
				BNO055.Quaternion.W,   
				BNO055.Quaternion.X,  
				BNO055.Quaternion.Y,
				BNO055.Quaternion.Z); 
SERIAL_Printf(msg);
```
# Data Visualization
* Data Transmission via UART:
The SERIAL_Printf() function transmits output data through the UART protocol to the ESP32. In this example, the ESP32 acts as a UART receiver, and data from the BNO055 sensor is displayed through serial communication.
* Data Reception on ESP32:
On the ESP32 side, the device is set up to receive data from the STM32 and display it over the serial monitor. The data received via UART can be visualized through the serial output:
* Real-Time Visualization:
 - Line Graph Visualization: To plot the received sensor data as a line graph, use the `Serial_plot.py` script. This script dynamically plots data in real time as it is received over the serial port. Make sure to adjust the Num_Vals variable in the script to match the number of values you are expecting to plot (the default is set to 4). Valid format `%f,%f,%f\n"

 Example command to run the script:
 ```
 python Serial_plot.py --port COM19	# Replace with your COM port
 ```
 - 3D Body Visualization: The `3dBody.py` script can be used to create a 3D visualization of the orientation of the device based on the euler angles data received from the BNO055 sensor. Change the. Valid format `%f,%f,%f\n`. You can modify the COM port and baud rate (COM19, 115200) to match your setup.







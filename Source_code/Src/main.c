/*
 * main.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void Error_handler(void);
void GPIO_Init(void);
void UART2_Init(void);
void UART6_Init(void);
void I2C2_Init(void);
void TIMER6_Init(void);
void Sensor_Init(void);
void Magwick_Init(void);
void SERIAL_Printf(	char msg[100] );
void SERIAL6_Printf( char msg[100] );
void SystemClock_Config_HSE(uint8_t clock_freq);

/* Private variables ---------------------------------------------------------*/
RCC_OscInitTypeDef Osc_Init;
RCC_ClkInitTypeDef Clock_Init;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htimer6;

BNO055_Handle_t handle_bno055;
BNO055_Sensors_t BNO055;

//uint8_t OffsetDatas[22];

char msg[200];
int len = 0;

int start_flag;
int init_flag;

//Euler
float yaw_mad, pitch_mad, roll_mad;

int main(void)
{
	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_120_MHZ);

	GPIO_Init();

	UART2_Init();

	UART6_Init();

	I2C2_Init();

	Sensor_Init();

	TIMER6_Init();

	Magwick_Init();

    while(1)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && start_flag == 0) //wait til button(PA0) is pressed
        {
            // Start timer
            HAL_TIM_Base_Start_IT(&htimer6);
            start_flag = 1;
        }

        if (start_flag == 1)
        {
            break;
        }
    }

	while(1){}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{

  uint8_t flash_latency=0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE ;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq) {
  case SYS_CLOCK_FREQ_50_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 50;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    //Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 1;
    break;

  case SYS_CLOCK_FREQ_84_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 84;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    //Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 2;
    break;

  case SYS_CLOCK_FREQ_120_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 120;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    //Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
    flash_latency = 3;
    break;

  case SYS_CLOCK_FREQ_168_MHZ:
    Osc_Init.PLL.PLLM = 8;
    Osc_Init.PLL.PLLN = 336;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 7;
    //Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
    flash_latency = 5;
    break;

  default:
    return ;
	}

	if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
	  Error_handler();
	}

	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
	{
		Error_handler();
	}

	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef ledgpio, btngpio;

	ledgpio.Pin = GPIO_PIN_12;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD,&ledgpio);

	btngpio.Pin = GPIO_PIN_0;
	btngpio.Mode = GPIO_MODE_INPUT;
	btngpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&btngpio);
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void UART6_Init(void)
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart6) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
**/
void I2C2_Init(void)
{
	  hi2c2.Instance = I2C2;
	  hi2c2.Init.ClockSpeed = 100000;
	  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	  hi2c2.Init.OwnAddress1 = 0;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
		SERIAL_Printf("**************************** \r\n");
		SERIAL_Printf("Error with I2C2 initialization \r\n");
		SERIAL_Printf("**************************** \r\n");
	    Error_handler();
	  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 5999;
	htimer6.Init.Period = 125-1;			//interrupt every 0.0125s (80hz)
	if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
	{
		SERIAL_Printf("Error with Timer6 initialization \r\n");
		Error_handler();
	}
}

void Sensor_Init(void){

	// Reset Bno055
	ResetBno055();

	handle_bno055.ACC_Range = ACC_RANGE_4G;
	handle_bno055.GYR_Range = GYR_RANGE_1000_DPS;
	handle_bno055.Frame = NED_FRAME;
	handle_bno055.Clock_Source = CLOCK_EXTERNAL;		//CLOCK_EXTERNAL or CLOCK_INTERNAL
	handle_bno055.PWR_Mode = BNO055_NORMAL_MODE;			//BNO055_X_MODE   X:NORMAL, LOWPOWER, SUSPEND
	handle_bno055.OP_Mode = AMG;						//if the application requires only raw meas, this mode can be chosen.
	//handle_bno055.OP_Mode = NDOF;					// Sensor Fusion mode
	//handle_bno055.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_RPS | UNIT_ACC_MS2);
	 handle_bno055.Unit_Sel = ( UNIT_GYRO_RPS | UNIT_ACC_MS2);
	BNO055_Init(&handle_bno055);
}

void Magwick_Init(void){

	Madgwick_Handle_t hMAD;

	uint32_t APB1_clk = HAL_RCC_GetPCLK1Freq();
	uint8_t APB1_timer_multiplier;

	if (Clock_Init.APB1CLKDivider == RCC_HCLK_DIV1) {
		APB1_timer_multiplier = 1;
	}else {
		APB1_timer_multiplier = 2;
	}

	hMAD.beta = 0.68f;
	hMAD.deltat = (float)(htimer6.Init.Period + 1) / ((APB1_clk * APB1_timer_multiplier) / (htimer6.Init.Prescaler + 1));
	hMAD.frame = handle_bno055.Frame;

	Magwick_Setup(&hMAD);
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htimer6){

		   if (init_flag == 0)
		   {
			   ReadData(&handle_bno055, &BNO055, SENSOR_ACCEL| SENSOR_MAG);
			   ecompass(&BNO055, BNO055.Accel.X, BNO055.Accel.Y, BNO055.Accel.Z,
                       BNO055.Magneto_CAL.X, BNO055.Magneto_CAL.Y, BNO055.Magneto_CAL.Z);
			   init_flag = 1;

		   }else{
		       ReadData(&handle_bno055, &BNO055, SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG);

		       Madgwick_Update(&BNO055, BNO055.Accel.X, BNO055.Accel.Y, BNO055.Accel.Z,
		                         BNO055.Gyro.X, BNO055.Gyro.Y, BNO055.Gyro.Z,
		                         BNO055.Magneto_CAL.X, BNO055.Magneto_CAL.Y, BNO055.Magneto_CAL.Z);
		   }

		   // Madgwick's algorithm
		   quat2EUangle(BNO055.Quaternion_MAD.W, BNO055.Quaternion_MAD.X, BNO055.Quaternion_MAD.Y, BNO055.Quaternion_MAD.Z,
				   	    &yaw_mad, &pitch_mad, &roll_mad);

		   len = snprintf(msg, sizeof(msg), "%f,%f,%f\n",
				   	   	   yaw_mad,   
						   roll_mad,  
						   pitch_mad); 
		   SERIAL_Printf(msg);

		}
}


/**
  * @brief printf by uart protocol Function
  * @param None
  * @retval None
**/
void SERIAL_Printf(	char msg[200] )
{
	if ( HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY) != HAL_OK )
	{
		SERIAL_Printf("**************************** \r\n");
		SERIAL_Printf("Error with UART2 Transmission \r\n");
		SERIAL_Printf("**************************** \r\n");
		Error_handler();
	}
	memset(msg, 0, 200);
}
void SERIAL6_Printf(	char msg[200] )
{
	if ( HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY) != HAL_OK )
	{
		SERIAL_Printf("**************************** \r\n");
		SERIAL_Printf("Error with UART6 Transmission \r\n");
		SERIAL_Printf("**************************** \r\n");
		Error_handler();
	}
	memset(msg, 0, 200);
}


/**
  * @brief Error handle function
  * @param None
  * @retval None
**/
void Error_handler(void)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	while(1);
}

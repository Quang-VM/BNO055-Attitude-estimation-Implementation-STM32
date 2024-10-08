/*
 * msp.c
 *
 *  Created on: Aug 20, 2024
 *      Author: HQLap
 */

#include "stm32f4xx_hal.h"


/**
  * @brief  Initialize the MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
  //Here will do low level processor specific inits.
  //1. Set up the priority grouping of the arm cortex mx processor
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  //2. Enable the required system exceptions of the arm cortex mx processor
  SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions 1 1 1

  //3. configure the priority for the system exceptions
  HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
  HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
  HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);
}


/**
  * @brief  UART MSP Init.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_uart;

    if (huart->Instance == USART2)
    {
        // 1. Enable the clock for the USART2 peripheral as well as for GPIOA peripheral
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // 2. Do the pin muxing configurations for USART2
        gpio_uart.Pin = GPIO_PIN_2;          // UART2 TX pin
        gpio_uart.Mode = GPIO_MODE_AF_PP;
        gpio_uart.Pull = GPIO_PULLUP;
        gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_uart.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio_uart);

        gpio_uart.Pin = GPIO_PIN_3;          // UART2 RX pin
        gpio_uart.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio_uart);

        // 3. Enable the IRQ and set up the priority for USART2
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);
    }
    else if (huart->Instance == USART6)
    {
        // 1. Enable the clock for the USART1 peripheral as well as for GPIOA peripheral
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        // 2. Do the pin muxing configurations for USART1
        gpio_uart.Pin = GPIO_PIN_6;          // UART6 TX pin
        gpio_uart.Mode = GPIO_MODE_AF_PP;
        gpio_uart.Pull = GPIO_PULLUP;
        gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_uart.Alternate = GPIO_AF8_USART6 ;
        HAL_GPIO_Init(GPIOC, &gpio_uart);

        gpio_uart.Pin = GPIO_PIN_7;         // UART6 RX pin
        gpio_uart.Alternate = GPIO_AF8_USART6 ;
        HAL_GPIO_Init(GPIOC, &gpio_uart);

        // 3. Enable the IRQ and set up the priority for USART1
        HAL_NVIC_EnableIRQ(USART6_IRQn);
        HAL_NVIC_SetPriority(USART6_IRQn, 15, 0);
    }
}



/**
  * @brief  i2c MSP Init.
  * @param  hi2c  Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C module.
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio_i2c;
  //here we are going to do the low level inits. of the i2c2 peripheral

  //1. enable the clock for the i2c2 peripheral as well as for GPIOB peripheral
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //2 . Do the pin muxing configurations

  /**I2C2 GPIO Configuration
  PB10     ------> I2C2_SCL
  PB11     ------> I2C2_SDA
  */

  gpio_i2c.Pin = GPIO_PIN_10;
  gpio_i2c.Mode = GPIO_MODE_AF_OD;
  gpio_i2c.Pull = GPIO_NOPULL;
  gpio_i2c.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_i2c.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio_i2c);

  gpio_i2c.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &gpio_i2c);

  //3 . Enable the IRQ and set up the priority (NVIC settings )

}


/**
  * @brief  Initializes the TIM Base MSP.
  * @param  htim TIM Base handle
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer)
{

	//1. enable the clock for the TIM6 peripheral
	__HAL_RCC_TIM6_CLK_ENABLE();

	//2. Enable the IRQ of TIM6
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	//3. setup the priority for TIM6_DAC_IRQn
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn,15,0);


}

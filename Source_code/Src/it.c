/*
 * it.c
 */

#include <main.h>

extern void initial_estimate(void);

extern TIM_HandleTypeDef htimer6;
extern uint8_t init_complete_flag;

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt.

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}
*/

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
	if (init_complete_flag)
	{
		    initial_estimate();  //ecompass

        HAL_TIM_Base_Start_IT(&htimer6);  //Start timer
        init_complete_flag = 0;
	}
	 //Clear pending bit in exti pending register, this bit was set whenever an external event happens
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}


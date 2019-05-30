/**
 **************************************************************************
 * File Name    : at32f4xx_it.c
 * Description  : at32f4xx interrupt service routines.
 * Date         : 2018-02-12
 * Version      : V1.0.4
 **************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "at32f4xx_it.h"

extern uint32_t buzzerFreq;
uint32_t buzzer_counter;
__IO uint16_t ADC2ConvertedValue;

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 at32f4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_at32f403_xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles ADC1 and ADC2 global interrupts requests.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  /* Get injected channel13 converted value */
  ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
	ADC_ClearINTPendingBit(ADC1, ADC_INT_EC);
}

void TMR2_GLOBAL_IRQHandler(void){
	static uint8_t state = 0;
	if (TMR_GetINTStatus(TMR2, TMR_INT_Overflow) != RESET)
	{
		TMR_ClearITPendingBit(TMR2, TMR_INT_Overflow );
		if(buzzerFreq == 0) {
			GPIO_ResetBits(GPIOA, GPIO_Pins_4);
			state = 0;
			return;
		}
		buzzer_counter++;
		if((buzzer_counter % buzzerFreq) == 0) {
			buzzer_counter = 0;
			if(state) GPIO_SetBits(GPIOA, GPIO_Pins_4);
			else GPIO_ResetBits(GPIOA, GPIO_Pins_4);
			state = 1 - state;
		}
	}

}

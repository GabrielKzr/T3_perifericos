#include <stm32f4xx_conf.h>
#include <hal.h>
#include <usart.h>

static volatile int sensor1_poll = 0;
static volatile int sensor2_poll = 0;

/* sensor data is polled every 10 seconds (for example) */
void TIM2_IRQHandler()
{
	static uint16_t secs = 0;
	
	/* Checks whether the TIM2 interrupt has occurred or not */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
		if (++secs > 3) {
			secs = 0;
			sensor1_poll = 1;
			sensor2_poll = 1;
		}
		/* Clears the TIM2 interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void tim2_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	/* Enable clock for TIM2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* TIM2 initialization overflow every 1000ms
	 * TIM2 by default has clock of 84MHz
	 * Here, we must set value of prescaler and period,
	 * so update event is 1Hz or 1000ms
	 * Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * (TIM_Period + 1))
	 * Update Event (Hz) = 84MHz / ((8399 + 1) * (9999 + 1)) = 1 Hz
	 */
	TIM_TimeBaseInitStruct.TIM_Prescaler = 8399;
	TIM_TimeBaseInitStruct.TIM_Period = 9999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	/* TIM2 initialize */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	/* Enable TIM2 interrupt */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* Start TIM2 */
	TIM_Cmd(TIM2, ENABLE);
	
	/* Nested vectored interrupt settings */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOC Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* configure board LED as output */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

int sensor1_poll_data()
{
	if (sensor1_poll) {
		sensor1_poll = 0;
		
		return 1;
	}
	
	return 0;
}

int sensor2_poll_data()
{
	if (sensor2_poll) {
		sensor2_poll = 0;
		
		return 1;
	}
	
	return 0;
}

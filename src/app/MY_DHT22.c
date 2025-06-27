#include <stm32f4xx_conf.h>
#include <hal.h>
#include "MY_DHT22.h"

static void tim10_start()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	/* preset default values of the timer struct */
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	
	/* TIM10 by default has clock of 84MHz
	 * Here, we must set value of prescaler, period and mode */
	TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock / 1000000;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	/* TIM10 initialize */
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStruct);

	/* Start TIM10 */
	TIM_Cmd(TIM10, ENABLE);
}

static void tim10_stop()
{
	/* Stop TIM10 */
	TIM_Cmd(TIM10, DISABLE);
}

static void delay_us(uint16_t delay)
{
	volatile uint16_t start;
	
	tim10_start();

	start = TIM10->CNT;
	while ((TIM10->CNT - start) <= delay);

	tim10_stop();
}

static int dht_poll(struct dht_s *sensor)
{
	return GPIO_ReadInputDataBit(sensor->bus.port, sensor->bus.pin);
}

static int dht_wait(struct dht_s *sensor, uint8_t logic, uint16_t timeout_us, uint16_t mult_us)
{
	int time = 0;
	
	/* wait for logic level to change on pin */
	while (dht_poll(sensor) == logic) {
		delay_us(1);
		if (++time > timeout_us * mult_us)
			return -1;
	}
	
	return 0;
}

static void dht_start(struct dht_s *sensor)
{
	GPIO_ResetBits(sensor->bus.port, sensor->bus.pin);
	delay_us(DHT_STARTLOW);
	GPIO_SetBits(sensor->bus.port, sensor->bus.pin);
	delay_us(DHT_STARTHIGH);
}

static int dht_read_byte(struct dht_s *sensor)
{
	int i;
	uint8_t byte = 0;
	
	for (i = 0; i < 8; i++) {
		/* wait for the start of bit transmission (twice the time for tolerance) */
		if (dht_wait(sensor, 0, (DHT_STARTXMIT << 1), 1))
			return ERR_TIMEOUT3;

		delay_us(DHT_DATAZERO);
		
		/* sample data */
		if (dht_poll(sensor))
			byte |= (1 << (7 - i));
			
		/* wait for the end of bit transmission (twice the time for tolerance) */
		if (dht_wait(sensor, 1, (DHT_DATAONE << 1), 1))
			return ERR_TIMEOUT4;
	}
	
	return byte;
}

static int dht_read_data(struct dht_s *sensor)
{
	int i, val;
	
	dht_start(sensor);
	
	/* wait for the sensor to wake up after a start */
	if (dht_wait(sensor, 1, DHT_TIMEOUTMS, 1000))
		return ERR_TIMEOUT1;
	
	/* if the line is low, sensor is up */
	if (!dht_poll(sensor)) {
		delay_us(DHT_INITLOW);

		/* wait for sync (high) */
		if (dht_wait(sensor, 0, DHT_INITLOW, 1))
			return ERR_TIMEOUT2;
		
		delay_us(DHT_INITHIGH);

		/* wait for sync (low) */
		if (dht_wait(sensor, 1, DHT_INITHIGH, 1))
			return ERR_TIMEOUT2;

		/* in sync, fetch data */
		for (i = 0; i < 5; i++) {
			val = dht_read_byte(sensor);
			
			if (val < 0)
				return val;
				
			sensor->data[i] = val;
		}
		
		if (((sensor->data[0] + sensor->data[1] + 
		sensor->data[2] + sensor->data[3]) & 0xff) == 
		sensor->data[4])
			return ERR_NOERROR;
		else
			return ERR_CHECKSUM;
	}
	
	return ERR_TIMEOUT1;
}

int dht_read(struct dht_s *sensor)
{
	int val;
	
	val = dht_read_data(sensor);
		
	if (val != ERR_NOERROR) {
		
		return val;
	}

	switch (sensor->type) {
	case DHT11:
		val = (int)sensor->data[2];
		val = sensor->data[3] & 0x80 ? -val - 1 : val;
		val = val * 10;
		val += (int)(sensor->data[3] & 0x0f);
		sensor->temperature = val;
		sensor->humidity = ((int)sensor->data[0] * 10) + (int)sensor->data[1];
		break;
	case DHT12:
		val = (int)sensor->data[2];
		val += (int)(sensor->data[3] & 0x0f);
		sensor->temperature = sensor->data[2] & 0x80 ? -val * 10 : val * 10;
		sensor->humidity = ((int)sensor->data[0] * 10) + (int)sensor->data[1];
		break;
	case DHT22:
		val = (((int)(sensor->data[2] & 0x7f)) << 8) | sensor->data[3];
		sensor->temperature = sensor->data[2] & 0x80 ? -val : val;		
		val = (((int)sensor->data[0]) << 8) | sensor->data[1];
		sensor->humidity = val;
		break;
	default:
		return ERR_ERROR;
	}
	
	return ERR_NOERROR;
}

void dht_setup(struct dht_s *sensor, uint8_t type, uint32_t clock_en, GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable clock for TIM10 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	
	sensor->type = type;
	sensor->bus.clock_en = clock_en;
	sensor->bus.port = port;
	sensor->bus.pin = pin;
	
	/* GPIO Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(sensor->bus.clock_en, ENABLE);
	
	/* configure pin as an open drain mode output, with internal pullup */
	GPIO_InitStructure.GPIO_Pin   = sensor->bus.pin;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(sensor->bus.port, &GPIO_InitStructure);

	/* let the pin float by setting '1' to the output
	 * (PMOS disabled in open drain GPIO mode).
	 * this way, external peripherals can pull the line low */
	GPIO_SetBits(sensor->bus.port, sensor->bus.pin);
}
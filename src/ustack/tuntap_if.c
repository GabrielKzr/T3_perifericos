/* file:          tuntap_if.c
 * description:   tun/tap interface access / low level
 *                ethernet driver abstraction / tunnel via serial interface (MCU side)
 * date:          07/2021
 * author:        Sergio Johann Filho <sergio.johann@acad.pucrs.br>
 */

#include "ustack.h"
#include <hal.h>
#include <usart.h>


int32_t if_setup()
{
	uart_init(USART_PORT, USART_BAUD, 0);
	
	return 0;
}

void en_ll_output(uint8_t *frame, uint16_t size)
{
	uint16_t i;

	uart_tx(USART_PORT, 0x7e);
	for (i = 0; i < size && i < FRAME_SIZE; i++) {
		if (frame[i] == 0x7e || frame[i] == 0x7d) {
			uart_tx(USART_PORT, 0x7d);
			uart_tx(USART_PORT, frame[i] ^ 0x20);
		} else {
			uart_tx(USART_PORT, frame[i]);
		}
	}
	uart_tx(USART_PORT, 0x7e);
}


int32_t en_ll_input(uint8_t *frame)
{
	uint16_t i = 0;
	uint8_t data;

	if (uart_rxsize(USART_PORT) == 0)
		return 0;

	data = uart_rx(USART_PORT);
	if (data != 0x7e) {
		while (uart_rxsize(USART_PORT) > 0 && data != 0x7e)
			data = uart_rx(USART_PORT);

		if (uart_rxsize(USART_PORT) == 0)
			return 0;
	}

	for (i = 0; i < FRAME_SIZE; i++) {
		if (uart_rxsize(USART_PORT) == 0) {
			delay_ms(10);
			if (uart_rxsize(USART_PORT) == 0)
				return 0;
		}
		
		data = uart_rx(USART_PORT);

		if (data == 0x7e)
			break;

		if (data == 0x7d) {
			if (uart_rxsize(USART_PORT) == 0) {
				delay_ms(10);
				if (uart_rxsize(USART_PORT) == 0)
					return 0;
			}
			
			data = uart_rx(USART_PORT);
			data ^= 0x20;
		}

		frame[i] = data;
	}

	return i;
}

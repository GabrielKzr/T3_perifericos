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

uint16_t netif_send(uint8_t *packet, uint16_t len)
{
	uint16_t i;

	uart_tx(USART_PORT, 0x00);
	uart_tx(USART_PORT, 0x00);
	uart_tx(USART_PORT, 0x08);
	uart_tx(USART_PORT, 0x00);

	for (i = 0; i < len && i < FRAME_SIZE; i++)
		uart_tx(USART_PORT, packet[i]);
}

uint16_t netif_recv(uint8_t *packet)
{
	uint16_t i = 0;
	uint8_t data;

	if (uart_rxsize(USART_PORT) == 0)
		return 0;
		
	for (i = 0; i < 4; i++) {
		if (uart_rxsize(USART_PORT) == 0) {
			delay_ms(1);
			if (uart_rxsize(USART_PORT) == 0)
				return 0;
		}
		uart_rx(USART_PORT);
	}

	for (i = 0; i < FRAME_SIZE; i++) {
		if (uart_rxsize(USART_PORT) == 0) {
			delay_ms(1);
			if (uart_rxsize(USART_PORT) == 0)
				return i;
		}
		
		packet[i] = uart_rx(USART_PORT);
	}

	return i;
}

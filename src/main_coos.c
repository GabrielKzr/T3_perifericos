/*
 * demo application for ustack-stm32
 * 
 * connect to the application with:
 * nc -u 172.31.69.20 30168
 * 
 * then type a message, it should reply with 'Hello world!'.
 */

#include <stm32f4xx_conf.h>
#include <hal.h>
#include <usart.h>
#include <coos.h>
#include <ustack.h>

uint8_t eth_frame[FRAME_SIZE];
uint8_t mymac[6] = {0x0e, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t myip[4];
uint8_t mynm[4];
uint8_t mygw[4];


int32_t app_udp_handler(uint8_t *packet)
{
	uint8_t dst_addr[4];
	uint16_t src_port, dst_port;
	struct ip_udp_s *udp = (struct ip_udp_s *)packet;
	uint8_t msg[] = "Ola Mundo!\n";

	src_port = ntohs(udp->udp.src_port);
	dst_port = ntohs(udp->udp.dst_port);

	if (dst_port == UDP_DEFAULT_PORT) {
		memcpy(dst_addr, udp->ip.src_addr, 4);
		memcpy(packet + sizeof(struct ip_udp_s), msg, sizeof(msg));
		udp_out(dst_addr, dst_port, src_port, packet, sizeof(struct udp_s) + sizeof(msg));
	}
	
	return 0;
}

void *network_task(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	uint16_t len;
	
	len = netif_recv(packet);

	if (len > 0) {
		/* turn board LED on */
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		
		ip_in(myip, packet, len);
	
		/* turn board LED off */
		GPIO_SetBits(GPIOC, GPIO_Pin_13);	
	}
	
	return 0;
}

void *hello_task(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	uint8_t dst_addr[4] = {172, 31, 69, 55};
	uint8_t dst_mac[6] = {0x00, 0x00, 0x00, 0x33, 0x33, 0x33};
	uint16_t src_port, dst_port;
	char *msg = "Send this to the network...";
	char data[512];
	static int count = 0;
	static int pkt = 1;

	// send a packet every 50k calls...
	if (count++ % 50000 == 0) {
		src_port = 12345;
		dst_port = 5555;
		// generate data
		sprintf(data, "%s %d\n", msg, pkt);
		// fill datagram payload
		memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
		// update arp entry with fake MAC address
		arp_update(dst_addr, dst_mac);
		// send bogus data: UDP -> IP -> Ethernet
		udp_out(dst_addr, dst_port, src_port, packet, sizeof(struct udp_s) + strlen(data));
		GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
		pkt++;
	}
	
	return 0;
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

int main(void)
{
	struct task_s tasks[MAX_TASKS] = { 0 };
	struct task_s *ptasks = tasks;

	led_init();

	/* setup ustack */
	if_setup();
	config(mymac + 2, USTACK_IP_ADDR);
	config(myip, USTACK_IP_ADDR);
	config(mynm, USTACK_NETMASK);
	config(mygw, USTACK_GW_ADDR);
	udp_set_callback(app_udp_handler);
	
	/* setup CoOS and tasks */
	task_pinit(ptasks);
	task_add(ptasks, network_task, 50);
	task_add(ptasks, hello_task, 150);
	
	while (1) {
		task_schedule(ptasks);
	}
	
	return 0;
}

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
#include <hw_res.h>
#include <MY_DHT22.h>

#define SENSOR_LDR		"oi/teste/lux"
#define SENSOR_TEMP		"oi/teste/temp"
#define SENSOR_HUMIDITY	"oi/teste/humidity"
#define ACTUATOR1		"oi/teste/actuator1"

struct dht_s dht_sensor = {
	.type = DHT22,
	.data = {0},
	.temperature = 0,
	.humidity = 0
};

const float V_RAIL = 3300.0;		// 3300mV rail voltage
const float ADC_MAX = 4095.0;		// max ADC value
const int ADC_SAMPLES = 1024;		// ADC read samples
const int REF_RESISTANCE = 10000;	// 4k7 ohms

const int LUX_MIN = 10;
const int LUX_MAX = 100;
const int DUTY_MAX = 1000;

uint8_t eth_frame[FRAME_SIZE];
uint8_t mymac[6] = {0x0e, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t myip[4];
uint8_t mynm[4];
uint8_t mygw[4];

/* ADC library */
void analog_config();
void adc_config(void);
void adc_channel(uint8_t channel);
uint16_t adc_read();
void setup_topic(uint8_t *packet, char *topic);
void set_relay1_topic(char * dataval);
int control_relay1(int val);

/* PWM library */
void pwm_config();

int32_t app_udp_handler(uint8_t *packet)
{
	uint8_t dst_addr[4];
	uint16_t src_port, dst_port;
	struct ip_udp_s *udp = (struct ip_udp_s *)packet;
	char *datain, *dataval;
	char data[256];

	src_port = ntohs(udp->udp.dst_port);
	dst_port = ntohs(udp->udp.src_port);

	if (ntohs(udp->udp.dst_port) == UDP_DEFAULT_PORT) {
		memcpy(dst_addr, udp->ip.src_addr, 4);
		
		datain = (char *)packet + sizeof(struct ip_udp_s);
		datain[ntohs(udp->udp.len) - sizeof(struct udp_s)] = '\0';
		
		if (strstr(datain, ACTUATOR1)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			set_relay1_topic(dataval);
			
			/* print received data and its origin */
			sprintf(data, "[%s]", dataval,
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);

			/* toggle LED */
			GPIO_ToggleBits(GPIOC, GPIO_Pin_13);

			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
	}
	
	return 0;
}

void *network_task(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	uint16_t len;
	
	len = netif_recv(packet);

	if (len > 0) {
		// turn board LED on 
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		
		ip_in(myip, packet, len);
	
		// turn board LED off 
		GPIO_SetBits(GPIOC, GPIO_Pin_13);	
	}
	
	return 0;
}

void net_setup(uint8_t *packet)
{
	uint16_t i;
	
	for (i = 0; i < 100; i++) {
		netif_recv(packet);
		delay_ms(100);
	}

	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
}

/*
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
*/

void set_relay1_topic(char * dataval)
{
	if(strcmp(dataval, "-1") == 0) control_relay1(-1);
	else { 
		int control_relay =  control_relay1(0);
		if(control_relay == -1) { 
			if(strcmp(dataval, "turn on relay1 ") == 0) {
				GPIO_SetBits(GPIOB, GPIO_Pin_6);
			} 	
			else if (strcmp(dataval, "turn off relay1") == 0) {
				GPIO_ResetBits(GPIOB, GPIO_Pin_6);
			} 
		} 
	}
}

int control_relay1(int val)
{ 
	//val = 0 to read the value of control relay1
	//val = -1 to toggle the value of control relay1 
	static int relay1_control = 1;
	if(val == -1){
		relay1_control = relay1_control * -1; 
	}
	return relay1_control;
}

/* these functions are called when timer 2 generates an interrupt */
void read_temperature(uint8_t *packet, char *topic)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];

	dht_read(&dht_sensor);
	
	//sprintf(data, "PUBLISH %s Temperature: %d.%d | %d | %dC", topic, dht_sensor.temperature/10, dht_sensor.temperature % 10, control_relay1(0), dht_sensor.temperature + (random() % 7 - 3));
	sprintf(data, "PUBLISH %s Temperature: %d.%d C", topic, dht_sensor.temperature/10, dht_sensor.temperature % 10);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

void read_humidity(uint8_t *packet, char *topic)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	
	dht_read(&dht_sensor);
	
	sprintf(data, "PUBLISH %s Humidade: %d.%d kg/m3", topic, dht_sensor.humidity/10, dht_sensor.humidity % 10);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

void set_relay1_automatic ()
{
 	int control_relay =  control_relay1(0);
	dht_read(&dht_sensor);
		if(control_relay == 1) { 
			//int temp = (int)dht_sensor.temperature/10 + (dht_sensor.temperature%10)/10.0;	
			int  temp = dht_sensor.temperature + (random() % 7 - 3);
			if(temp < 194)  {
				GPIO_SetBits(GPIOB, GPIO_Pin_6);
  			}
   			else if (temp >= 194) {
				GPIO_ResetBits(GPIOB, GPIO_Pin_6);
   			}
	} 
}

float luminosity()
{
	float voltage, lux = 0.0, rldr;
	
	for (int i = 0; i < ADC_SAMPLES; i++) {
		voltage = adc_read() * (V_RAIL / ADC_MAX);
		rldr = (REF_RESISTANCE * (V_RAIL - voltage)) / voltage;
		lux += 500 / (rldr / 650);
	}
	
	return (lux / ADC_SAMPLES);
}

void set_pwm_lux(float lux)
{
    int duty_cycle;
    
    if (lux < LUX_MIN) lux = LUX_MIN;
    if (lux > LUX_MAX) lux = LUX_MAX;
    
    duty_cycle = (1.0 - (lux - LUX_MIN) / (LUX_MAX - LUX_MIN)) * DUTY_MAX;
    
    TIM4->CCR4 = duty_cycle;
}

void sensor_ldr_data(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];

	memset(data, 0, sizeof(data));
	memset(buf, 0, sizeof(buf));
	
    if (val < LUX_MIN) val = LUX_MIN;
    if (val > LUX_MAX) val = LUX_MAX;

	ftoa(val, buf, 1);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

void *ldr(void *arg)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
    
	static float f = 0.0f;	
    // static int count = 0;
    
	adc_channel(ADC_Channel_8);
	f = luminosity();	
    
    if (sensor1_poll_data()) {
		sensor_ldr_data(packet, SENSOR_LDR, f);
		set_pwm_lux(f);
		GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
    }
    
	return 0;
}

void *temp(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	
	if (sensor2_poll_data()) {
		delay_ms(200);
		read_temperature(packet, SENSOR_TEMP);
		read_humidity(packet, SENSOR_HUMIDITY);
		set_relay1_automatic();
	}
	
	return 0;
}

void setup_topic(uint8_t *packet, char *topic)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	
	strcpy(data, "TOPIC ");
	strcat(data, topic);
	
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

int main(void)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	struct task_s tasks[MAX_TASKS] = { 0 };
	struct task_s *ptasks = tasks;

	led_init();
    analog_config();
	adc_config();
	pwm_config();
	tim2_init();

	/* setup ustack */
	if_setup();
	config(mymac + 2, USTACK_IP_ADDR);
	config(myip, USTACK_IP_ADDR);
	config(mynm, USTACK_NETMASK);
	config(mygw, USTACK_GW_ADDR);

	net_setup(packet);
	udp_set_callback(app_udp_handler);

	setup_topic(packet, SENSOR_LDR);
	setup_topic(packet, SENSOR_TEMP);
	setup_topic(packet, SENSOR_HUMIDITY);
	setup_topic(packet, ACTUATOR1);

	/* setup CoOS and tasks */
	task_pinit(ptasks);
	task_add(ptasks, network_task, 50);
	task_add(ptasks, temp, 50);
    task_add(ptasks, ldr, 50);

	dht_setup(&dht_sensor, DHT22, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOC Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* configure board LED as output */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	
	srand(123);

	while (1) {
		task_schedule(ptasks);
	}
	
	return 0;
}

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
// #define ACTUATOR1		"oi/teste/actuator1"
#define DIMER 			"oi/teste/dimer"
#define RELAY1			"oi/teste/relay1"
#define RELAY2			"oi/teste/relay2"

struct dht_s dht_sensor = {
	.type = DHT22,
	.data = {0},
	.temperature = 0,
	.humidity = 0
};

struct relay_t {
	uint16_t pin;
	int TEMP_MIN;
	int TEMP_MAX;
};

struct relay_t relay1 = {
	.pin = GPIO_Pin_6,
	.TEMP_MIN = -1,
	.TEMP_MAX = -1
};

struct relay_t relay2 = {
	.pin = GPIO_Pin_5,
	.TEMP_MIN = -1,
	.TEMP_MAX = -1
};

const float V_RAIL = 3300.0;		// 3300mV rail voltage
const float ADC_MAX = 4095.0;		// max ADC value
const int ADC_SAMPLES = 1024;		// ADC read samples
const int REF_RESISTANCE = 10000;	// 4k7 ohms

int LUX_MIN = -1;
int LUX_MAX = -1;
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
void dimer_topic(char * dataval);
void set_pwm_percent(int percent);
void relay_topic(const char *dataval, struct relay_t *relay);
void relay_on(uint16_t pin);
void relay_off(uint16_t pin);

/* PWM library */
void pwm_config();

int32_t app_udp_handler(uint8_t *packet)
{
	struct ip_udp_s *udp = (struct ip_udp_s *)packet;
	char *datain, *dataval;

	// GPIO_ToggleBits(GPIOC, GPIO_Pin_13);

	if (ntohs(udp->udp.dst_port) == UDP_DEFAULT_PORT) {
		
		datain = (char *)packet + sizeof(struct ip_udp_s);
		datain[ntohs(udp->udp.len) - sizeof(struct udp_s)] = '\0';
		
		if(strstr(datain, DIMER)) {

			// GPIO_ToggleBits(GPIOC, GPIO_Pin_13);

			dataval = strstr(datain, " ") + 1;

			dimer_topic(dataval);			
		} else if(strstr(datain, RELAY1)) {

			// GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
			GPIO_ToggleBits(GPIOC, GPIO_Pin_13);

			dataval = strstr(datain, " ") + 1;

			relay_topic(dataval, &relay1);
		} else if(strstr(datain, RELAY2)) {


			GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
			// GPIO_ToggleBits(GPIOC, GPIO_Pin_13);

			dataval = strstr(datain, " ") + 1;

			relay_topic(dataval, &relay2);
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
		//GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		
		ip_in(myip, packet, len);
	
		// turn board LED off 
		//GPIO_SetBits(GPIOC, GPIO_Pin_13);	
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
	char *msg = "Send this to the network...";ACTUATOR1
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

void dimer_topic(char *dataval)
{
    char type[4];  // cabe “min”/“max”/“dim” + '\0'
    const char *space;
    const char *numstr;
    char *endptr;
    int num;
    size_t len;

    // 1) localiza o primeiro espaço
    space = strchr(dataval, ' ');
    if (!space) return;  // sem separador, sai

    // 2) calcula e valida comprimento do tipo
    len = space - dataval;
    if (len == 0 || len > 3) return;  // vazio ou muito grande

    // 3) copia e encerra com '\0'
    memcpy(type, dataval, len);
    type[len] = '\0';

    // 4) valida somente os tipos permitidos
    if (strcmp(type, "min") != 0 &&
        strcmp(type, "max") != 0 &&
        strcmp(type, "dim") != 0) {
        return;
    }

    // 5) aponta para o início do número e pula espaços
    numstr = space + 1;
    while (*numstr == ' ') ++numstr;
    if (*numstr == '\0') return;  // sem número

    // 6) converte com strtol e valida
    num = strtol(numstr, &endptr, 10);
    if (endptr == numstr) return;        // nada convertido
    while (*endptr == ' ') ++endptr;     // pula espaços finais
    if (*endptr != '\0') return;         // caracteres extras => formatação errada

    // 7) aqui você tem ‘type’ válido e ‘num’ convertido
    if (strcmp(type, "min") == 0) {
        LUX_MIN = (int)num;
    } else if (strcmp(type, "max") == 0) {
        LUX_MAX = (int)num;
    } else {  // dim
		set_pwm_percent((int)num);
    }
}

void relay_topic(const char *dataval, struct relay_t *relay)
{
    char cmd[4];        // “on”, “off”, “min” ou “max” + '\0'
    const char *space;
    const char *argstr;
    char *endptr;
    long val;
    size_t len;

    // 1) localiza o primeiro espaço (se houver)
    space = strchr(dataval, ' ');
    if (space) {
        len = space - dataval;
        if (len == 0 || len > 3) return;           // comando vazio ou muito longo
        memcpy(cmd, dataval, len);
        cmd[len] = '\0';
        // prepara a string do argumento
        argstr = space + 1;
        while (*argstr && isspace((unsigned char)*argstr)) ++argstr;
    }
    else {
        // não há espaço: todo dataval é o comando
        len = strlen(dataval);
        if (len == 0 || len > 3) return;
        strncpy(cmd, dataval, 3);
        cmd[3] = '\0';
        argstr = NULL;
    }

    // 3) trata cada comando
    if (strcmp(cmd, "on") == 0) {
		if(relay->TEMP_MIN == -1 || relay->TEMP_MAX == -1) {
        	relay_on(relay->pin);
		}
    }
    else if (strcmp(cmd, "off") == 0) {
		if(relay->TEMP_MIN == -1 || relay->TEMP_MAX == -1) {
        	relay_off(relay->pin);
		}
    }
    else if ((strcmp(cmd, "min") == 0 || strcmp(cmd, "max") == 0) && argstr) {
        // converte o valor numérico após min/max
        val = strtol(argstr, &endptr, 10);
        if (endptr == argstr) return;              // nada convertido
        while (*endptr && isspace((unsigned char)*endptr)) ++endptr;
        if (*endptr != '\0') return;              // lixo extra -> aborta

        if (strcmp(cmd, "min") == 0) {
            relay->TEMP_MIN = (int)val;
        } else {
            relay->TEMP_MAX = (int)val;
        }

        // opcional: após ajustar o range, verificar estado atual do relé
        // ex: check_and_update_relay(read_current_temperature());
    }
    else {
        // comando inválido: ignora
        return;
    }
}

void relay_on(uint16_t pin)
{
	GPIO_ResetBits(GPIOB, pin);
}

void relay_off(uint16_t pin)
{
	GPIO_SetBits(GPIOB, pin);
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

void set_relay_automatic (struct relay_t *relay)
{
	dht_read(&dht_sensor);
	int  temp = dht_sensor.temperature;    if (relay->TEMP_MIN == -1 || relay->TEMP_MAX == -1) {
        return;
    }

	if(temp >= relay->TEMP_MIN*10 && temp <= relay->TEMP_MAX*10) {
		relay_on(relay->pin);
	} else {
		relay_off(relay->pin);
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

static void _set_pwm_duty(int duty_cycle)
{
    if (duty_cycle < 0) duty_cycle = 0;
    if (duty_cycle > DUTY_MAX) duty_cycle = DUTY_MAX;

    TIM4->CCR4 = duty_cycle;
}

// Interface 1: com base na luminosidade
void set_pwm_lux(float lux)
{
    if (lux < LUX_MIN) lux = LUX_MIN;
    if (lux > LUX_MAX) lux = LUX_MAX;

    float ratio = 1.0 - (lux - LUX_MIN) / (LUX_MAX - LUX_MIN);
    int duty = (int)(ratio * DUTY_MAX);

    _set_pwm_duty(duty);
}

// Interface 2: com base em percentual (0 a 100)
void set_pwm_percent(int percent)
{
	if (LUX_MAX != -1 && LUX_MIN != -1) {
		// Se os limites estão definidos, não faz nada
		return;
	}
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    int duty = (int)((percent / 100.0) * DUTY_MAX);

    _set_pwm_duty(duty);
}

void sensor_ldr_data(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 254};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];

	memset(data, 0, sizeof(data));
	memset(buf, 0, sizeof(buf));
	
    // if (val < LUX_MIN) val = LUX_MIN;
    // if (val > LUX_MAX) val = LUX_MAX;

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
		if(LUX_MIN != -1 && LUX_MAX != -1) {
			set_pwm_lux(f);
		}
		// GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
    }
    
	return 0;
}

void *temp(void *)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	
	if (sensor2_poll_data()) {
		//delay_ms(200);
		read_temperature(packet, SENSOR_TEMP);
		read_humidity(packet, SENSOR_HUMIDITY);

		if(relay1.TEMP_MIN != -1 && relay1.TEMP_MAX != -1) 
		{
			set_relay_automatic(&relay1);
		}

		if(relay2.TEMP_MIN != -1 && relay2.TEMP_MAX != -1)
		{
			set_relay_automatic(&relay2);
		}
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
	// setup_topic(packet, ACTUATOR1);
	setup_topic(packet, DIMER);
	setup_topic(packet, RELAY1);
	setup_topic(packet, RELAY2);


	/* setup CoOS and tasks */
	task_pinit(ptasks);
	task_add(ptasks, network_task, 50);
	task_add(ptasks, temp, 50);
    task_add(ptasks, ldr, 50);

	dht_setup(&dht_sensor, DHT22, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7);

	
	/* GPIOC Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* configure board LED as output */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_6);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	
	srand(123);

	while (1) {
		task_schedule(ptasks);
	}
	
	return 0;
}

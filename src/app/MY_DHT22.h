/* DHT11 / DHT22 */
enum dht_error {
	ERR_NOERROR	= 0,
	ERR_ERROR	= -1,
	ERR_TIMEOUT1	= -2,
	ERR_TIMEOUT2	= -3,
	ERR_TIMEOUT3	= -4,
	ERR_TIMEOUT4	= -5,
	ERR_CHECKSUM	= -6
};

enum dht_timing {
	DHT_TIMEOUTMS	= 100,
	DHT_STARTLOW	= 20000,
	DHT_STARTHIGH	= 30,
	DHT_INITLOW	= 80,
	DHT_INITHIGH	= 80,
	DHT_STARTXMIT	= 50,
	DHT_DATAZERO	= 40,
	DHT_DATAONE	= 60
};

enum dht_type {
	DHT11, DHT12, DHT22
};

struct dht_bus_s {
	uint32_t clock_en;
	GPIO_TypeDef *port;
	uint16_t pin;
};

struct dht_s {
	struct dht_bus_s bus;
	uint8_t type;
	uint8_t data[5];
	uint16_t temperature;
	uint16_t humidity;
};

int dht_read(struct dht_s *sensor);
void dht_setup(struct dht_s *sensor, uint8_t type, uint32_t clock_en, GPIO_TypeDef *port, uint16_t pin);
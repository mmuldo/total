#define I2C_CHANNEL i2c0
#define SDA_GPIO_PIN 16
#define SCL_GPIO_PIN 17
#define I2C_BAUDRATE_HZ 400000

#define MS5803_14BA_ADDRESS             0x76
#define MS5803_14BA_RESET_CMD           0x1E
#define MS5803_14BA_PROM_READ_BASE_CMD  0xA0
#define MS5803_14BA_D1_4096             0x40
#define MS5803_14BA_D2_4096             0x58
#define MS5803_14BA_ADC_READ            0x00
 
void ms5803_14ba_reset();
uint16_t ms5803_14ba_get_coefficient(uint8_t coefficient);
uint32_t ms5803_14ba_get_d1(void);
uint32_t ms5803_14ba_get_d2(void);
unsigned char crc4(unsigned int n_prom[]);

void init_i2c();
void get_temperature_and_pressure(double* temperature_pointer, double* pressure_pointer);

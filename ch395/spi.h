#include "stdint.h"
#define PIN_CS 10
#define PIN_DO 13
#define PIN_DI 11
#define PIN_CK 12

void spi1_init(void);
uint8_t spi1_read_write_byte(uint8_t write);

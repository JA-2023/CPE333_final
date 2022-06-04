#include <stdint.h>

#ifndef SRC_SPI_H_
#define SRC_SPI_H_
#define DAC_RESOLUTION 		4096
#define VOLT_REF_mV			3300
#define SPI_MOSI_AD         0x110E0000
#define SPI_CS_AD           0x11100000
#define SWITCHES_AD         0x11000000


void DAC_init(void);
void DAC_write(uint16_t digital_output); //need to write 12 bits so use
void DAC_volt_conv(uint16_t volt); //will convert voltage to 12 bit value to control DAC
#endif /* SRC_SPI_H_ */
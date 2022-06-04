#include "spi_header.h"
#include "saw_wave.h"
#include "sinewave.h"
#include "trianglewave.h"

int main(void)
{

    volatile int* SPI_MOSI_AD;
    volatile int* SPI_CS_AD;
    volatile int* SWITCHES_AD;
    uint16_t wave_point;
    while(1)
    {
        //put loop in here to go through arrays
        if(*SWITCHES_AD == 0)
        {
            //output sinewave
            for(int i = 0; i <540; i++)
            {
                wave_point = sine_array[i];
                DAC_write(wave_point);
            }
        }
        else if(*SWITCHES_AD == 1)
        {
            //output saw wave
            for(int i = 0; i <540; i++)
            {
                wave_point = saw_array[i];
                DAC_write(wave_point);
            }
        }
        else if(*SWITCHES_AD == 2)
        {
            //output triangle wave
            for(int i = 0; i <540; i++)
            {
                wave_point = triangle_array[i];
                DAC_write(wave_point);
            }
        }
    }
}

DAC_write(uint16_t point)
{
    //wait until SPI ready to send
    while(!TXE_READY);
    //send out the first byte 
    *SPI_MOSI_AD = wave_point >> 8;

    //wait until SPI ready to send again
    while(!TXE_READY);
    //send out the lower byte
    *SPI_MOSI_AD = wave_point & 0x00FF;
}

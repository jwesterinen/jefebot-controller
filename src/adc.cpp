/*
 * adc.c
 *
 *  ADC on Raspberry Pi SPI device 0
 */

#include <stdint.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "adc.h"

ADC::ADC(unsigned _period) : GenericSensor(_period), spiDevId(SPI_DEV_0)
{
	for (int i = 0; i < 8; ++i)
		digitalCodes[i] = 0;

    // init the SPI device and cache the FD
	if ((fd = InitSPI(spiDevId)) == -1)
    {
    	throw DP::FrameworkException("ADC", ERR_INITIALIZATION);
    }
}

void ADC::Routine()
{
    unsigned dcode;    // a single adc reading
    uint8_t inbuf[4];  // data to receive
    uint8_t outbuf[4]; // data to send

    struct spi_ioc_transfer tr =
    {
        tr.tx_buf = (unsigned long)outbuf,
        tr.rx_buf = (unsigned long)inbuf,
        tr.len = 4,
        tr.delay_usecs = 0,
    };

    for (int i = 0; i < 8; i++)
    {
        // start bit, single ended, channel 0
        outbuf[0] = 0xc0 | (i << 3);
        outbuf[1] = outbuf[2] = outbuf[3] = 0;
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) == -1)
        {
        	throw DP::FrameworkException("ADC", ERR_READ);
        }
        dcode  = ((uint16_t)(inbuf[3]) << 3) & 0x0200;
        dcode += ((uint16_t)(inbuf[3]) << 1) & 0x0100;
        dcode += ((uint16_t)(inbuf[2]) << 7) & 0x0080;
        dcode += ((uint16_t)(inbuf[2]) << 5) & 0x0040;
        dcode += ((uint16_t)(inbuf[2]) << 3) & 0x0020;
        dcode += ((uint16_t)(inbuf[2]) << 1) & 0x0010;
        dcode += ((uint16_t)(inbuf[2]) >> 1) & 0x0008;
        dcode += ((uint16_t)(inbuf[2]) >> 3) & 0x0004;
        dcode += ((uint16_t)(inbuf[2]) >> 5) & 0x0002;
        dcode += ((uint16_t)(inbuf[2]) >> 7) & 0x0001;
        digitalCodes[i] = dcode;
    }
}

// InitSPI():  Open/init SPI port0,ce0.  Return fd or -1
int ADC::InitSPI(const char *dev)
{
    // mode is combination of : SPI_LOOP; SPI_CPHA; SPI_CPOL; SPI_LSB_FIRST;
    //                          SPI_CS_HIGH; SPI_3WIRE; SPI_NO_CS; SPI_READY;
    static uint8_t mode = 0;
    static uint8_t bits = 8;
    static uint32_t speed = 1000000;
    int fd;

    fd = open(dev, O_RDWR);
    if (fd < 0)
        return(-1);

    /* spi mode */
    if ((ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) ||
        (ioctl(fd, SPI_IOC_RD_MODE, &mode) == -1))
    {
        return(-1);
    }

    /* bits per word */
    if ((ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) ||
        (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1))
    {
        return(-1);
    }

    /* max speed hz */
    if ((ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) ||
        (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1))
    {
        return(-1);
    }

    return(fd);
}


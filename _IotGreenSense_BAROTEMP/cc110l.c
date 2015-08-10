#include <stdbool.h>
#include <stdint.h>

#include "math.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "cc110l.h"
#include "spi_master_mnhs.h"

uint32_t cc110l_spi_base_addr;

uint8_t dummy[16];

#define CC110_SS 10 
#define CC110_GDO0 11 

void CC110L_setupspibaseaddr(uint32_t addr)
{
 cc110l_spi_base_addr = addr;
}

void SPIStrobe(uint8_t byte)
{
 uint8_t dat[2];
dat[0] = byte;	
 spi_master_tx_rx_ss((uint32_t *)cc110l_spi_base_addr, CC110_SS, 1, (const uint8_t *)dat, dummy);
}

void SPIWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
  nrf_gpio_pin_clear(CC110_SS);
	spi_master_tx_rx_noss((uint32_t *)cc110l_spi_base_addr, count, (const uint8_t *)buffer, dummy);
  nrf_gpio_pin_set(CC110_SS);
	nrf_delay_us(40);
}
char SPIReadReg(uint8_t addr)
{
	char data[2];
	data[0] = (addr | CC110L_READ_SINGLE);
	data[1] = 0;
	spi_master_tx_rx_ss((uint32_t *)cc110l_spi_base_addr, CC110_SS, 2, (const uint8_t *)data, dummy);
  return dummy[1];
}
void SPIReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
  nrf_gpio_pin_clear(CC110_SS);
	spi_master_tx_rx_noss((uint32_t *)cc110l_spi_base_addr, 1, (const uint8_t *)(addr | CC110L_READ_BURST), dummy);
	spi_master_tx_rx_noss((uint32_t *)cc110l_spi_base_addr, count, buffer, dummy);
  nrf_gpio_pin_set(CC110_SS);
}



void CC110L_SPIWriteReg(uint8_t addr, uint8_t data)
{
 char dat[2];
 dat[0] = addr;
 dat[1] = data;
 spi_master_tx_rx_ss((uint32_t *)cc110l_spi_base_addr, CC110_SS, 2, (const uint8_t *)dat, dummy);
}

//GFSK_38Kbps_18KHz_70KHzRxBW_-0.5dBm
void setup_cc110l(void)
{
nrf_gpio_cfg_output(CC110_SS);
nrf_gpio_pin_set(CC110_SS);
nrf_gpio_cfg_input(CC110_GDO0,NRF_GPIO_PIN_NOPULL);
nrf_delay_us(100);
SPIStrobe(CC110L_SRES);
nrf_delay_us(100);

writeRFSettings();
CC110L_SPIWriteReg(CC110L_IOCFG0,0x06); // GDO0 notifies uC while the packet sending is in progress	
SPIStrobe(CC110L_SIDLE);
SPIStrobe(CC110L_SFTX);
SPIStrobe(CC110L_SPWD);
}


void writeRFSettings(void)
{
//Modulated = true
//Address config = No address check
//Data format = Normal mode
//Channel spacing = 199.951172
//Base frequency = 867.999939
//Device address = 0
//Manchester enable = false
//Modulation format = GFSK
//Packet length = 255
//Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word
//RX filter BW = 325.000000
//Sync word qualifier mode = 30/32 sync word bits detected
//Carrier frequency = 867.999939
//Preamble count = 4
//Data rate = 99.9756
//TX power = 0
//CRC enable = true
//CRC autoflush = false
//Deviation = 47.607422

CC110L_SPIWriteReg(0x40 | 0x0002,0x06);
CC110L_SPIWriteReg(0x40 | 0x0008,0x05);
CC110L_SPIWriteReg(0x40 | 0x000b,0x08);
CC110L_SPIWriteReg(0x40 | 0x000d,0x21);
CC110L_SPIWriteReg(0x40 | 0x000e,0x62);
CC110L_SPIWriteReg(0x40 | 0x000f,0x76);
CC110L_SPIWriteReg(0x40 | 0x0010,0x5B);
CC110L_SPIWriteReg(0x40 | 0x0011,0xF8);
CC110L_SPIWriteReg(0x40 | 0x0012,0x13);
CC110L_SPIWriteReg(0x40 | 0x0018,0x18);
CC110L_SPIWriteReg(0x40 | 0x0019,0x1D);
CC110L_SPIWriteReg(0x40 | 0x001a,0x1C);
CC110L_SPIWriteReg(0x40 | 0x001b,0xC7);
CC110L_SPIWriteReg(0x40 | 0x001c,0x00);
CC110L_SPIWriteReg(0x40 | 0x001d,0xB2);
CC110L_SPIWriteReg(0x40 | 0x0020,0xFB);
CC110L_SPIWriteReg(0x40 | 0x0021,0xB6);
CC110L_SPIWriteReg(0x40 | 0x0023,0xEA);
CC110L_SPIWriteReg(0x40 | 0x0024,0x2A);
CC110L_SPIWriteReg(0x40 | 0x0025,0x00);
CC110L_SPIWriteReg(0x40 | 0x0026,0x1F);
CC110L_SPIWriteReg(0x40 | 0x002e,0x09);
}
// PATABLE (-0.5 dBm output power)
char paTable[] = {0xc0,0x00};   // -0.5 dBm
char paTableLen = 2;

void sendbyte(uint8_t a)
{
  uint8_t Buffer[3];
  Buffer[0] = 2;                           // Packet length
  Buffer[1] = 0x01;                        // Packet address
  Buffer[2] = a;    	// Load four switch inputs
  CC110L_RFSendPacket(Buffer, 3);                 // Send value over RF
}

void CC110L_RFSendPacket(uint8_t *txBuffer, uint8_t size)
{
	txBuffer[1] = size-2;
	SPIStrobe(CC110L_SIDLE);
	nrf_delay_us(300);
  SPIWriteBurstReg(CC110L_TXFIFO, txBuffer, size); // Write TX data
  SPIStrobe(CC110L_STX);           // Change state to TX, initiating data transfer
  while(!nrf_gpio_pin_read(CC110_GDO0)){}
  while(nrf_gpio_pin_read(CC110_GDO0)){}
	SPIStrobe(CC110L_SIDLE);
  SPIStrobe(CC110L_SFTX);
	SPIStrobe(CC110L_SPWD);
}

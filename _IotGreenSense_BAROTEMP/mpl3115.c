#include <stdbool.h>
#include <stdint.h>

#include "math.h"

#include "nrf_delay.h"

#include "mpl3115.h"
#include "twi_master.h"

// MPL3115A2 register definitions
#define _OUT_P_MSB            0x01 // Pressure/Altitude data (MSB)
#define _OUT_P_CSB            0x02 // Pressure/Altitude data (middle)
#define _OUT_P_LSB            0x03 // Pressure/Altitude data (LSB)
#define _OUT_T_MSB            0x04 // Temperature data (MSB)
#define _OUT_T_LSB            0x05 // Temperature data (LSB)
#define _PT_DATA_CFG          0x13 // Data event flag configuration
#define _BAR_IN_MSB           0x14 // Barometric input for Altitude calculation
#define _BAR_IN_LSB           0x15 // Barometric input for Altitude calculation
#define _CTRL_REG1            0x26 // Control register 1
#define _CTRL_REG2            0x27 // Control register 1
#define _OFFH                 0x2D // Altitude Data User Offset Register
#define _MPL3115A2_W_ADDRESS  0xC0
#define _MPL3115A2_R_ADDRESS  0xC1
#define _MPL3115A2_ADDRESS    0x60

uint8_t dataa[3];

void MPL3115A2_Write(unsigned short address, unsigned short data_) {
 dataa[0] = address;
 dataa[1] = data_;
 twi_master_transfer(_MPL3115A2_W_ADDRESS, dataa, 2, TWI_ISSUE_STOP);
}

uint8_t MPL3115A2_Read(uint8_t address)
{
  uint8_t config[6];
	uint8_t datatosend = address;

  if (twi_master_transfer(_MPL3115A2_W_ADDRESS, &datatosend, 1, TWI_DONT_ISSUE_STOP))
  {
    if (twi_master_transfer(_MPL3115A2_W_ADDRESS | TWI_READ_BIT, config, 2, TWI_ISSUE_STOP)) // Read: current configuration
    {
    //  TWI_powerdown();
			// Read succeeded, configuration stored to variable "config"
			return config[0]; 
    }
    else
    {
			//TWI_powerdown();
      // Read failed
      return false;
    }
  } 
			//TWI_powerdown();
	return false;
 
}

// Initialize Barometer
void MPL3115A2_Init(void) {
  MPL3115A2_Write(_OFFH, 0);
  MPL3115A2_Write(_CTRL_REG1, 27);     // Barometer selected, one shot mode, 128x oversampling (512 ms)
  MPL3115A2_Write(_CTRL_REG1, 25);     // Clear oversampling bit
  MPL3115A2_Write(_CTRL_REG2, 4);     // Clear oversampling bit
}

// Read Baromterer register
uint32_t MPL3115A2_Read_Baro(void) {
  return MPL3115A2_Read(_OUT_P_LSB) + (MPL3115A2_Read(_OUT_P_CSB)<<8) + (MPL3115A2_Read(_OUT_P_MSB)<<16);
}

// Read Temperature register
uint16_t MPL3115A2_Read_Temp(void) {
  return MPL3115A2_Read(_OUT_T_LSB) + (MPL3115A2_Read(_OUT_T_MSB)<<8);
}

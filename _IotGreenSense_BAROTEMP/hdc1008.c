#include <stdbool.h>
#include <stdint.h>

#include "math.h"

#include "nrf_delay.h"

#include "hdc1008.h"
#include "twi_master.h"


// MPL3115A2 register definitions
#define _TEMPERATURE_REG      0x00 // Pressure/Altitude data (middle)
#define _HUMIDITY_REG         0x01 // Pressure/Altitude data (LSB)
#define _CONFIG_REG           0x02 // Pressure/Altitude data (MSB)
#define _HDC1008_ADDRESS      0x80


void HDC1008_Write(uint8_t address, uint8_t * data) {
 twi_master_transfer(_HDC1008_ADDRESS, data, 2, TWI_ISSUE_STOP);
}

uint8_t HDC1008_Read(uint8_t address, uint8_t * value)
{
  uint8_t data[2];

    if (twi_master_transfer(_HDC1008_ADDRESS | TWI_READ_BIT, data, 2, TWI_ISSUE_STOP)) // Read: current configuration
    {
    //  TWI_powerdown();
			// Read succeeded, configuration stored to variable "config"
			value[0] = data[0];
			value[1] = data[1];
    }
    else
    {
			//TWI_powerdown();
      // Read failed
      return false;
    }
			//TWI_powerdown();
	return false;
 
}

// Initialize Barometer
void HDC1008_Init(void) {
 uint8_t config[] = {0x06,0x00}; //normal operation, heater disabled, temperature OR humidity acuiqred, 11bit temperature, 8bit humidity
 HDC1008_Write(_CONFIG_REG,config);
}

// Read Data Registers
void HDC1008_TriggerHumidity(void) {
	uint8_t datatosend[] = {_HUMIDITY_REG};
  twi_master_transfer(_HDC1008_ADDRESS, datatosend, 1, TWI_ISSUE_STOP);
}
// Read Data Registers
uint8_t HDC1008_ReadHumidity(void) {
  uint8_t value[2];
  uint32_t val_completed;
	HDC1008_Read(_HUMIDITY_REG,value);
  val_completed = value[0]<<8 | value[1];
	return (val_completed*100)/65536;     //formula as stated in datasheet of HDC1008, page 14
}

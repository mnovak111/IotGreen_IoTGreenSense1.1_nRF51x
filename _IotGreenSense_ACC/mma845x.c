#include <stdbool.h>
#include <stdint.h>

#include "math.h"

#include "nrf_delay.h"

#include "mma845x.h"
//#include "twi_master_config.h"
#include "twi_master.h"


void MMA845x_init(void)
{
	uint8_t config0[] = {0x0b,0x00}; //SYSMOD register - STANDBY mode
	uint8_t config1[] = {0x0e,0x00}; //XYZ_DATA_CFG    - +-2G full scale range, no high pass filtering
	uint8_t config2[] = {0x2a,0x3D}; //CTRL_REG1       - ACTIVE, low noise enable, data rate 1.56 Hz, 
	uint8_t config3[] = {0x2b,0x03}; //Low power mode - 6 microamps

  twi_master_transfer(56, config0, 2, TWI_ISSUE_STOP);
	nrf_delay_us(50);
  twi_master_transfer(56, config1, 2, TWI_ISSUE_STOP);
	nrf_delay_us(50);
  twi_master_transfer(56, config2, 2, TWI_ISSUE_STOP);
	nrf_delay_us(50);
  twi_master_transfer(56, config3, 2, TWI_ISSUE_STOP);
	nrf_delay_us(50);
}

bool MMA845x_read_all(uint8_t * data)
{
	uint8_t datatosend = 1;
  // Write: command protocol
  if (twi_master_transfer(56, &datatosend, 1, TWI_DONT_ISSUE_STOP))
  {
    if (twi_master_transfer(56 | TWI_READ_BIT, data, 6, TWI_ISSUE_STOP)) // Read: current configuration
    {
			// Read succeeded, configuration stored to variable "config"
			return true;
    }
    else
    {
      // Read failed
      return false;
    }
  } 
	return false;
 
}

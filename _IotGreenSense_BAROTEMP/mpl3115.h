#include <stdbool.h>
#include <stdint.h>

void MPL3115A2_Write(unsigned short address, unsigned short data_);
uint8_t MPL3115A2_Read(uint8_t address);
void MPL3115A2_Init(void);
uint32_t MPL3115A2_Read_Baro(void);
uint16_t MPL3115A2_Read_Temp(void);

#include <stdbool.h>
#include <stdint.h>

void HDC1008_Write(uint8_t address, uint8_t * data);
uint8_t HDC1008_Read(uint8_t address, uint8_t * value);
void HDC1008_Init(void);
uint8_t HDC1008_ReadHumidity(void);
void HDC1008_TriggerHumidity(void);

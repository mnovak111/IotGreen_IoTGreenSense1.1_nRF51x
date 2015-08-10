#include <stdbool.h>
#include <stdint.h>

void MMA845x_init(void);
bool MMA845x_read_all(uint8_t * data);

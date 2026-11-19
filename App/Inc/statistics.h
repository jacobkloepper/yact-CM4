// statistics.h
// Interface for interacting with the link statistics object.

#include "App/Inc/telemetry.h"

void Statistics_Init(void);

void Statistics_Add_TX(uint32_t bytes);
void Statistics_Add_Rx(uint32_t bytes);

void Statistics_Add_CRC_Error(void);

telemetry_t Statistics_Get(void);

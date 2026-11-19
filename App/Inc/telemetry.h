// Define the telemetry data packet.

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* Telemetry datatype */
typedef struct
{
	uint32_t tx_packets;
	uint32_t rx_packets;
	uint32_t crc_errors;
	uint32_t throughput_bps;
} telemetry_t;

/* Lifecycle */
void Telemetry_Init(void);

/* Telemetry Access */
telemetry_t* Telemetry_Get(void);

void Telemetry_Update(void);
void Telemetry_Reset(void);

#endif

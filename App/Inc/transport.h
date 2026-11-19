// transport.h
// Link-agnostic send/receive interface

int Transport_Send(uint8_t* data, uint16_t len);
int Transport_Receive(uint8_t* data, uint16_t maxlen);

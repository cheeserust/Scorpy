#ifndef MCP2515_H
#define MCP2515_H

#include "config.h"

#define MCP2515_OSC_8MHZ   8
#define MCP2515_OSC_16MHZ  16

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;

extern volatile uint8_t g_mcp2515_irq_pending;

void spi2_init(void);
uint8_t mcp2515_init_500k(uint8_t osc_mhz);
uint8_t mcp2515_read_frame(CanFrame *frame);
uint8_t mcp2515_send_frame(const CanFrame *frame);
uint8_t mcp2515_int_asserted(void);
uint8_t mcp2515_service(void);
void mcp2515_abort_all_tx(void);

#endif

#include <control.h>
#include <registers.h>
#include <stdbool.h>

#define CMD_SRC 0xE0

static volatile int i;

void enc28j60_reset(struct enc28j60_state_s *state)
{
    state->spi_cs(0);
    state->spi_rw(CMD_SRC);
    state->spi_cs(1);
    
    for (i = 0; i < 1000UL; i++)
        ;
    state->current_bank = 0;
}

void enc28j60_enable_rx(struct enc28j60_state_s *state, int enable)
{
    if (enable)
        enc28j60_set_bits8(state, ECON1, 1 << 2);
    else
        enc28j60_clear_bits8(state, ECON1, 1 << 2);
}

bool enc28j60_tx_ready(struct enc28j60_state_s *state)
{
    return !(enc28j60_read_register8(state, ECON1) & (1 << 3));
}

bool enc28j60_tx_err(struct enc28j60_state_s *state, uint8_t *status)
{
    if(enc28j60_read_register8(state, EIR) & (1<<1))
    {
        enc28j60_read_buffer(state, state->send_status_addr, status, 7);
        enc28j60_set_bits8(state, ECON1, 1 << 3);
        enc28j60_clear_bits8(state, ECON1, 1 << 3);
        return true;
    }
    return false;
}

#include <registers.h>
#include <tx.h>
#include <control.h>

uint16_t __enc28j60_send_status_addr;

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *data, size_t len)
{
    if (!enc28j60_tx_ready(state))
        return false;

    enc28j60_write_buffer(state, state->send_buffer_start, "\x00", 1);
    enc28j60_write_buffer(state, state->send_buffer_start+1, data, len);

    enc28j60_write_register16(state, ETXSTL, state->send_buffer_start);
    enc28j60_write_register16(state, ETXNDL, state->send_buffer_start + 1 + len - 1);
    state->send_status_addr = state->send_buffer_start + 1 + len;

    enc28j60_set_bits8(state, ECON1, 1 << 3);
    return true;
}

bool enc28j60_send_start(struct enc28j60_state_s *state)
{
    if (!enc28j60_tx_ready(state))
        return false;
    state->write_ptr = state->send_buffer_start;
    enc28j60_write_register16(state, ETXSTL, state->write_ptr);
    enc28j60_write_buffer(state, state->write_ptr, "\x00", 1);
    state->write_ptr++;
    enc28j60_write_register16(state, ETXNDL, state->write_ptr-1);
    state->send_status_addr = state->write_ptr;
    return true;
}

void enc28j60_send_append(struct enc28j60_state_s *state, const uint8_t *data, size_t len)
{
    enc28j60_write_buffer(state, state->write_ptr, data, len);
    state->write_ptr += len;
    enc28j60_write_register16(state, ETXNDL, state->write_ptr-1);
    state->send_status_addr = state->write_ptr;
}

void enc28j60_send_commit(struct enc28j60_state_s *state)
{
    enc28j60_set_bits8(state, ECON1, 1 << 3);
}

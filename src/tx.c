#include <registers.h>
#include <tx.h>
#include <control.h>

uint16_t __enc28j60_send_status_addr;

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *src, const uint8_t *dst, const uint8_t *data, size_t len)
{
    if (!enc28j60_tx_ready(state))
        return false;

    enc28j60_write_buffer(state, state->send_buffer_start, "\x00", 1);
    enc28j60_write_buffer(state, state->send_buffer_start+1, dst, 6);
    enc28j60_write_buffer(state, state->send_buffer_start+1+6, src, 6);
    enc28j60_write_buffer(state, state->send_buffer_start+1+6+6, data, len);

    enc28j60_write_register16(state, ETXSTL, state->send_buffer_start);
    enc28j60_write_register16(state, ETXNDL, state->send_buffer_start + 1 + 6 + 6 + len - 1);
    state->send_status_addr = state->send_buffer_start + 1 + 6 + 6 + len;

    enc28j60_set_bits8(state, ECON1, 1 << 3);
    return true;
}

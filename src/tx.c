#include <registers.h>
#include <tx.h>
#include <control.h>

uint16_t __enc28j60_send_status_addr;

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

struct tx_status_vector_s enc28j60_read_status_vector(struct enc28j60_state_s *state)
{
    struct tx_status_vector_s vector;
    uint8_t buffer[7];
    uint16_t vector_addr = state->write_ptr;
    enc28j60_read_buffer(state, vector_addr, buffer, 7);
    vector.count = ((int)buffer[1]) << 8 | buffer[0];
    vector.collisions = buffer[2] & 0x0F;
    vector.transmit_crc_error = (buffer[2] & 0x10) != 0;
    vector.length_check_error = (buffer[2] & 0x20) != 0;
    vector.length_out_of_range = (buffer[2] & 0x40) != 0;
    vector.transmit_done = (buffer[2] & 0x80) != 0;
    vector.transmit_multicast = (buffer[3] & 0x01) != 0;
    vector.transmit_broadcast = (buffer[3] & 0x02) != 0;
    return vector;
}

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *data, size_t len)
{
    if (!enc28j60_send_start(state))
        return false;
    enc28j60_send_append(state, data, len);
    enc28j60_send_commit(state);
    return true;
}

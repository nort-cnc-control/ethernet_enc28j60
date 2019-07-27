#include <unistd.h>
#include <rx.h>
#include <registers.h>
#include <string.h>

static uint8_t read_byte_from_rxbuf(struct enc28j60_state_s *state)
{
    uint8_t b;
    enc28j60_read_buffer(state, state->read_ptr, &b, 1);
    state->read_ptr = enc28j60_read_register16(state, ERDPTL);
    return b;
}

static uint16_t read_buf_from_rxbuf_len(struct enc28j60_state_s *state, uint8_t *buf, size_t len)
{
    enc28j60_read_buffer(state, state->read_ptr, buf, len);
    state->read_ptr = enc28j60_read_register16(state, ERDPTL);
    return state->read_ptr;
}

static ssize_t read_buf_from_rxbuf_end(struct enc28j60_state_s *state, uint8_t *buf, uint16_t end, size_t maxlen)
{
    size_t len;
    if (end > state->read_ptr)
    {
        len = end - state->read_ptr;
    }
    else
    {
        len = end - state->recv_buffer_start + state->recv_buffer_last + 1 - state->read_ptr;
    }
    if (len > maxlen)
        return -1;
    read_buf_from_rxbuf_len(state, buf, len);
    return len;
}

ssize_t enc28j60_read_packet(struct enc28j60_state_s *state, uint8_t *buf, size_t maxlen, uint32_t *status, uint32_t *crc)
{
    uint16_t npl = read_byte_from_rxbuf(state);
    uint16_t nph = read_byte_from_rxbuf(state);
    uint16_t next = (nph << 8) | npl;
    int data_end = next - 4;
    if (data_end < (int)state->recv_buffer_start)
        data_end += state->recv_buffer_size;
    read_buf_from_rxbuf_len(state, (uint8_t *)status, 4);
    ssize_t len = read_buf_from_rxbuf_end(state, buf, data_end, maxlen);
    if (len >= 0)
    {
        read_buf_from_rxbuf_len(state, (uint8_t *)crc, 4);
    }
    else
    {
        memset(&crc, 0xFF, 4);
    }
    // mark as readed
    enc28j60_write_register16(state, ERXRDPTL, next);
    // decrease PKTCNT
    enc28j60_set_bits8(state, ECON2, 1<<6);
    return len;
}

bool enc28j60_has_package(struct enc28j60_state_s *state)
{
    return enc28j60_read_register8(state, EPKTCNT) > 0;
}

#include <unistd.h>
#include <rx.h>
#include <registers.h>

size_t enc28j60_received_data_length(struct enc28j60_state_s *state)
{
    int epkt_cnt = enc28j60_read_register8(state, EPKTCNT);
    int erxwrpt = enc28j60_read_register16(state, ERXWRPTL);
    int rdptr = state->read_ptr;
    size_t len = 0;
    if (rdptr == erxwrpt)
        return 0;
    do
    {
        len += 1;
        rdptr += 1;
        if (rdptr == state->recv_buffer_last + 1)
            rdptr = state->recv_buffer_start;
    } while (rdptr != erxwrpt);
    return len;
}

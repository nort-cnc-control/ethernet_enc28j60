#pragma once

#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <state.h>

struct tx_status_vector_s
{
    unsigned int count;
    unsigned int collisions;
    bool transmit_crc_error;
    bool length_check_error;
    bool length_out_of_range;
    bool transmit_done;
    bool transmit_multicast;
    bool transmit_broadcast;
};

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *data, size_t len);

bool enc28j60_send_start(struct enc28j60_state_s *state);
void enc28j60_send_append(struct enc28j60_state_s *state, const uint8_t *data, size_t len);
void enc28j60_send_commit(struct enc28j60_state_s *state);
struct tx_status_vector_s enc28j60_read_status_vector(struct enc28j60_state_s *state);

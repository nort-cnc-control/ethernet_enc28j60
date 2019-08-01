#pragma once

#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <state.h>

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *data, size_t len);

bool enc28j60_send_start(struct enc28j60_state_s *state);
void enc28j60_send_append(struct enc28j60_state_s *state, const uint8_t *data, size_t len);
void enc28j60_send_commit(struct enc28j60_state_s *state);

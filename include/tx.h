#pragma once

#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <state.h>

bool enc28j60_send_data(struct enc28j60_state_s *state, const uint8_t *data, size_t len);

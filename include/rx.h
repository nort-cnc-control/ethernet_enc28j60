#pragma once

#include <unistd.h>
#include <state.h>
#include <stdbool.h>

ssize_t enc28j60_read_packet(struct enc28j60_state_s *state, uint8_t *buf, size_t maxlen, uint32_t *status, uint32_t *crc);
bool enc28j60_has_package(struct enc28j60_state_s *state);

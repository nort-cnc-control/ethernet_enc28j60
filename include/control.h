#pragma once

#include <registers.h>
#include <stdbool.h>
#include <state.h>

void enc28j60_reset(struct enc28j60_state_s *state);
void enc28j60_hard_reset(struct enc28j60_state_s *state);

void enc28j60_enable_rx(struct enc28j60_state_s *state, int enable);

bool enc28j60_tx_ready(struct enc28j60_state_s *state);
bool enc28j60_tx_err(struct enc28j60_state_s *state, uint8_t *status);


#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include "control.h"
#include <tx.h>
#include <rx.h>
#include <state.h>
#include <eth_codec.h>

extern const uint8_t enc28j60_eth_bcast[6];

void enc28j60_init(struct enc28j60_state_s *state,
                   void (*hard_reset)(bool rst),
                   uint8_t (*spi_read_write)(uint8_t data),
                   void (*spi_set_cs)(bool val),
                   void (*spi_write_buf)(const uint8_t *, size_t),
                   void (*spi_read_buf)(uint8_t *, size_t));

bool enc28j60_detect(struct enc28j60_state_s *state);

bool enc28j60_configure(struct enc28j60_state_s *state,
                        const uint8_t *mac,
                        uint16_t rx_buffer_size,
                        bool full_duplex);

void enc28j60_interrupt_enable(struct enc28j60_state_s *state, bool enable);

void enc28j60_fill_payload(uint8_t *buf, const uint8_t *payload, size_t len);

bool enc28j60_link_detect(struct enc28j60_state_s *state);

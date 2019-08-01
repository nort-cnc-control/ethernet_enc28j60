#pragma once

#include <stdint.h>
#include <unistd.h>

struct enc28j60_state_s {
    uint16_t send_buffer_start;
    uint16_t send_buffer_last;

    uint16_t recv_buffer_start;
    uint16_t recv_buffer_last;

    size_t recv_buffer_size;

    uint8_t (*spi_rw)(uint8_t);
    void (*spi_cs)(uint8_t);

    uint16_t read_ptr;
    uint16_t write_ptr;
    uint8_t current_bank;
    uint16_t send_status_addr;
};

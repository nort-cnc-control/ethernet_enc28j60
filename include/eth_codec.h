#pragma once

#include <unistd.h>
#include <stdint.h>

#define ETHERTYPE_IP  0x0800
#define ETHERTYPE_ARP 0x0806

#define ETHERNET_ADDR_LEN 6
#define ETHERNET_HEADER_LEN 14

uint16_t enc28j60_get_ethertype(const uint8_t *data, size_t len);
const uint8_t *enc28j60_get_target(const uint8_t *data, size_t len);
const uint8_t *enc28j60_get_source(const uint8_t *data, size_t len);
const uint8_t *enc28j60_get_payload(const uint8_t *data, size_t len, size_t *payload_len);

size_t enc28j60_fill_header(uint8_t *buf, const uint8_t *src, const uint8_t *dst, uint16_t ethertype, size_t len);

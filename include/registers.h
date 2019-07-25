#pragma once

#include <stdint.h>
#include <unistd.h>

#include <state.h>

uint8_t enc28j60_read_register8(struct enc28j60_state_s *state, uint16_t addr);
uint16_t enc28j60_read_register16(struct enc28j60_state_s *state, uint16_t addr);

void enc28j60_write_register8(struct enc28j60_state_s *state, uint16_t addr, uint8_t val);
void enc28j60_write_register16(struct enc28j60_state_s *state, uint16_t addr, uint16_t val);

void enc28j60_set_bits8(struct enc28j60_state_s *state, uint16_t addr, uint8_t mask);
void enc28j60_clear_bits8(struct enc28j60_state_s *state, uint16_t addr, uint8_t mask);

uint16_t enc28j60_read_phy_register16(struct enc28j60_state_s *state, uint8_t addr);
void enc28j60_write_phy_register16(struct enc28j60_state_s *state, uint8_t addr, uint16_t val);
void enc28j60_set_phy_bits16(struct enc28j60_state_s *state, uint8_t addr, uint16_t mask);
void enc28j60_clear_phy_bits16(struct enc28j60_state_s *state, uint8_t addr, uint16_t mask);

void enc28j60_write_buffer(struct enc28j60_state_s *state, uint16_t addr, const uint8_t *data, size_t len);
void enc28j60_read_buffer(struct enc28j60_state_s *state, uint16_t addr, uint8_t *data, size_t len);

// Eth registers
#define EBSTCON 0x0307
#define EBSTCSH 0x0309
#define EBSTCSL 0x0308
#define EBSTSD 0x0306
#define ECOCON 0x0315
#define EDMACSH 0x0017
#define EDMACSL 0x0016
#define EDMADSTH 0x0015
#define EDMADSTL 0x0014
#define EDMANDH 0x0013
#define EDMANDL 0x0012
#define EDMASTH 0x0011
#define EDMASTL 0x0010
#define EFLOCON 0x0317
#define EHT0 0x0100
#define EHT1 0x0101
#define EHT2 0x0102
#define EHT3 0x0103
#define EHT4 0x0104
#define EHT5 0x0105
#define EHT6 0x0106
#define EHT7 0x0107
#define EPAUSH 0x0319
#define EPAUSL 0x0318
#define EPKTCNT 0x0119
#define EPMCSH 0x0111
#define EPMCSL 0x0110
#define EPMM0 0x0108
#define EPMM1 0x0109
#define EPMM2 0x010A
#define EPMM3 0x010B
#define EPMM4 0x010C
#define EPMM5 0x010D
#define EPMM6 0x010E
#define EPMM7 0x010F
#define EPMOH 0x0115
#define EPMOL 0x0114
#define ERDPTH 0x0001
#define ERDPTL 0x0000
#define EREVID 0x0312
#define ERXFCON 0x0118
#define ERXNDH 0x000B
#define ERXNDL 0x000A
#define ERXRDPTH 0x000D
#define ERXRDPTL 0x000C
#define ERXSTH 0x0009
#define ERXSTL 0x0008
#define ERXWRPTH 0x000F
#define ERXWRPTL 0x000E
#define ETXNDH 0x0007
#define ETXNDL 0x0006
#define ETXSTH 0x0005
#define ETXSTL 0x0004
#define EWRPTH 0x0003
#define EWRPTL 0x0002

// MAC registers
#define MAADR1 0x0304
#define MAADR2 0x0305
#define MAADR3 0x0302
#define MAADR4 0x0303
#define MAADR5 0x0300
#define MAADR6 0x0301
#define MABBIPG 0x0204
#define MACLCON1 0x0208
#define MACLCON2 0x0209
#define MACON1 0x0200
#define MACON2 0x0201
#define MACON3 0x0202
#define MACON4 0x0203
#define MAIPGH 0x0207
#define MAIPGL 0x0206
#define MAMXFLH 0x020B
#define MAMXFLL 0x020A

// MII registers
#define MICMD 0x0212
#define MIRDH 0x0219
#define MIRDL 0x0218
#define MIREGADR 0x0214
#define MISTAT 0x030A
#define MIWRH 0x0217
#define MIWRL 0x0216

// Common
#define EIE 0x001B
#define EIR 0x001C
#define ESTAT 0x001D
#define ECON2 0x001E
#define ECON1 0x001F

// PHY Registers
#define PHCON1  0x00
#define PHSTAT1 0x01
#define PHID1   0x02
#define PHID2   0x03
#define PHCON2  0x10
#define PHSTAT2 0x11
#define PHIE    0x12
#define PHIR    0x13
#define PHLCON  0x14

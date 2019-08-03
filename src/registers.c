#include <registers.h>
#include <stdbool.h>
#include <unistd.h>

#define CMD_RCR 0x00
#define CMD_RBM (0x20 | 0x1A)
#define CMD_WCR 0x40
#define CMD_WBM (0x60 | 0x1A)
#define CMD_BFS 0x80
#define CMD_BFC 0xA0

static uint8_t read_register_cb(struct enc28j60_state_s *state, uint8_t addr, bool skip)
{
    uint8_t cmd = CMD_RCR | (addr & 0x1F);
    state->spi_cs(0);
    state->spi_rw(cmd);
    if (skip)
        state->spi_rw(0xFF);
    uint8_t val = state->spi_rw(0xFF);
    state->spi_cs(1);
    return val;
}

static void modify_register_cb(struct enc28j60_state_s *state, uint8_t type, uint8_t addr, uint8_t value)
{
    uint8_t cmd = type | (addr & 0x1F);
    state->spi_cs(0);
    state->spi_rw(cmd);
    state->spi_rw(value);
    state->spi_cs(1);
}

static void select_bank(struct enc28j60_state_s *state, uint8_t bank)
{
    if (bank == state->current_bank)
        return;
    state->current_bank = bank;
    modify_register_cb(state, CMD_BFC, 0x1F, 3);
    modify_register_cb(state, CMD_BFS, 0x1F, bank);
}

uint8_t enc28j60_read_register8(struct enc28j60_state_s *state, uint16_t addr)
{
    uint8_t bank = (addr >> 8) & 0x03;
    addr &= 0xFF;
    select_bank(state, bank);
    if (addr >= 0x1A)
        return read_register_cb(state, addr, 0);
    switch (bank)
    {
        case 0:
        case 1:
            return read_register_cb(state, addr, 0);
        case 2:
            return read_register_cb(state, addr, 1);
        case 3:
            if (addr <= 0x05 || addr == 0x0A)
                return read_register_cb(state, addr, 1);
            return read_register_cb(state, addr, 0);
    }
    return 0xFF;
}

uint16_t enc28j60_read_register16(struct enc28j60_state_s *state, uint16_t addr)
{
    uint16_t low = enc28j60_read_register8(state, addr);
    uint16_t high = enc28j60_read_register8(state, addr+1);
    return (high << 8) | low;
}

void enc28j60_write_register8(struct enc28j60_state_s *state, uint16_t addr, uint8_t val)
{
    uint8_t bank = (addr >> 8) & 0x03;
    addr &= 0xFF;
    select_bank(state, bank);
    modify_register_cb(state, CMD_WCR, addr, val);
}

void enc28j60_write_register16(struct enc28j60_state_s *state, uint16_t addr, uint16_t val)
{
    enc28j60_write_register8(state, addr, val & 0xFF);
    enc28j60_write_register8(state, addr+1, (val >> 8) & 0xFF);
}

void enc28j60_set_bits8(struct enc28j60_state_s *state, uint16_t addr, uint8_t mask)
{
    uint8_t val = enc28j60_read_register8(state, addr);
    val |= mask;
    enc28j60_write_register8(state, addr, val);
}

void enc28j60_clear_bits8(struct enc28j60_state_s *state, uint16_t addr, uint8_t mask)
{
    uint8_t val = enc28j60_read_register8(state, addr);
    val &= ~mask;
    enc28j60_write_register8(state, addr, val);
}

/* PHY REGISTERS FUNCTIONS */

uint16_t enc28j60_read_phy_register16(struct enc28j60_state_s *state, uint8_t addr)
{
    enc28j60_write_register8(state, MIREGADR, addr & 0x1F);
    enc28j60_set_bits8(state, MICMD, 1 << 0); // MIIRD
    int busy = 0;
    do
    {
        uint8_t stat = enc28j60_read_register8(state, MISTAT);
        busy = stat & 0x01;
    } while (busy);
    enc28j60_clear_bits8(state, MICMD, 1 << 0); // MIIRD
    return enc28j60_read_register16(state, MIRDL);
}

void enc28j60_write_phy_register16(struct enc28j60_state_s *state, uint8_t addr, uint16_t val)
{
    enc28j60_write_register8(state, MIREGADR, addr & 0x1F);
    enc28j60_write_register8(state, MIWRL, val & 0xFF);
    enc28j60_write_register8(state, MIWRH, (val >> 8) & 0xFF);
    int busy = 0;
    do
    {
        uint8_t stat = enc28j60_read_register8(state, MISTAT);
        busy = stat & 0x01;
    } while (busy);
}

void enc28j60_set_phy_bits16(struct enc28j60_state_s *state, uint8_t addr, uint16_t mask)
{
    uint16_t val = enc28j60_read_phy_register16(state, addr);
    val |= mask;
    enc28j60_write_phy_register16(state, addr, val);
}

void enc28j60_clear_phy_bits16(struct enc28j60_state_s *state, uint8_t addr, uint16_t mask)
{
    uint16_t val = enc28j60_read_phy_register16(state, addr);
    val &= ~mask;
    enc28j60_write_phy_register16(state, addr, val);
}

void enc28j60_write_buffer(struct enc28j60_state_s *state, uint16_t addr, const uint8_t *data, size_t len)
{
    if (len == 0)
        return;
    enc28j60_write_register16(state, EWRPTL, addr);
    state->spi_cs(0);
    state->spi_rw(CMD_WBM);
    state->spi_write(data, len);
    state->spi_cs(1);
}

void enc28j60_read_buffer(struct enc28j60_state_s *state, uint16_t addr, uint8_t *data, size_t len)
{
    enc28j60_write_register16(state, ERDPTL, addr);
    state->spi_cs(0);
    state->spi_rw(CMD_RBM);
    state->spi_read(data, len);
    state->spi_cs(1);
}

#include <enc28j60.h>
#include <registers.h>
#include <control.h>

const uint8_t enc28j60_eth_bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void enc28j60_init(struct enc28j60_state_s *state, uint8_t (*spi_read_write)(uint8_t data), void (*spi_set_cs)(uint8_t val))
{
    state->spi_rw = spi_read_write;
    state->spi_cs = spi_set_cs;
}

void enc28j60_configure(struct enc28j60_state_s *state,
                        const uint8_t *mac,
                        uint16_t rx_buffer_size,
                        bool full_duplex)
{
    state->recv_buffer_size = rx_buffer_size;

    state->recv_buffer_start = 0;
    state->recv_buffer_last = (rx_buffer_size-1) & 0x1FFF;

    state->send_buffer_start = rx_buffer_size;
    state->send_buffer_last = 0x1FFF;

    enc28j60_reset(state);
    enc28j60_enable_rx(state, 0);
    enc28j60_write_register16(state, ERXSTL, 0);
    enc28j60_write_register16(state, ERXNDL, state->recv_buffer_last);

    enc28j60_set_bits8(state, ECON2, 1 << 7); // auto increment pointer wher r / w

    while (enc28j60_read_register8(state, EPKTCNT) > 0)
        enc28j60_set_bits8(state, ECON2, 1<<6);

    enc28j60_write_register16(state, ERXWRPTL, 0);
    enc28j60_write_register16(state, ERXRDPTL, 0);

    // See ENC28J60 datasheet
    // 6.0 Initialization

    // clear MACON2
    enc28j60_write_register8(state, MACON2, 0x00);

    // set MARXEN
    enc28j60_set_bits8(state, MACON1, 1 << 0);

    // set RXPAUS and TXPAUS
    enc28j60_set_bits8(state, MACON1, (1 << 2) | (1 << 3));

    // set PADCFG to 60 bytes padding and TXCRCEN=1
    enc28j60_clear_bits8(state, MACON3, 0xF0);
    enc28j60_set_bits8(state, MACON3, (0 << 7) | (0 << 6) | (1 << 5) | (1 << 4));

    // DEFER=1 - in half duplex wait for medium released
    enc28j60_set_bits8(state, MACON4, 1 << 6);

    // set MAMXFL = 1518
    enc28j60_write_register16(state, MAMXFLL, 1518);

    // Set gaps
    if (full_duplex)
    {
        enc28j60_write_register8(state, MABBIPG, 0x15);
        enc28j60_write_register16(state, MAIPGL, 0x0012);
    }
    else
    {
        enc28j60_write_register8(state, MABBIPG, 0x12);
        enc28j60_write_register16(state, MAIPGL, 0x0C12);
    }

    // drop own packets in half-duplex
    if (!full_duplex)
    {
        enc28j60_set_phy_bits16(state, PHCON2, (1 << 8));
    }

    // Set half / full duplex
    if (full_duplex)
    {
        enc28j60_set_phy_bits16(state, PHCON1, (1 << 8));
        enc28j60_set_bits8(state, MACON3, 1 << 0);
    }
    else
    {
        enc28j60_clear_phy_bits16(state, PHCON1, (1 << 8));
        enc28j60_clear_bits8(state, MACON3, 1 << 0);
    }

    // Set LED config
    enc28j60_clear_phy_bits16(state, PHLCON, (0xF << 8) | (0xF << 4));
    enc28j60_set_phy_bits16(state, PHLCON, (0x4 << 8) | (0x7 << 4));

    // set MAC address
    enc28j60_write_register8(state, MAADR1, mac[0]);
    enc28j60_write_register8(state, MAADR2, mac[1]);
    enc28j60_write_register8(state, MAADR3, mac[2]);
    enc28j60_write_register8(state, MAADR4, mac[3]);
    enc28j60_write_register8(state, MAADR5, mac[4]);
    enc28j60_write_register8(state, MAADR6, mac[5]);

    state->read_ptr = state->recv_buffer_start;

    // enable RX
    enc28j60_enable_rx(state, 1);
}

void enc28j60_interrupt_enable(struct enc28j60_state_s *state, bool enable)
{
    if (enable)
    {
        // enable INT and PKT
        enc28j60_set_bits8(state, EIE, (1 << 7) | (1 << 6));
    }
    else
    {
        // disable INT and PKT
        enc28j60_clear_bits8(state, EIE, (1 << 7) | (1 << 6));
    }
}

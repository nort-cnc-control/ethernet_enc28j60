#define STM32F1

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>

#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <enc28j60.h>
#include <uip.h>
#include <uip_arp.h>

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
    rcc_periph_clock_enable(RCC_USART1);

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_SPI2);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}


static void usart_setup(int baudrate)
{
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    /* Setup GPIO pin GPIO_USART1_TX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, baudrate);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    USART_CR1(USART1) |= USART_CR1_RXNEIE;
    USART_CR1(USART1) |= USART_CR1_TCIE;

    /* Finally enable the USART. */
    usart_enable(USART1);
}

void spi2_isr(void)
{

}

static uint8_t spi_rw(uint8_t d)
{
    spi_send(SPI2, d);
    return spi_read(SPI2);
}

static void spi_cs(uint8_t v)
{
    if (v)
        gpio_set(GPIOB, GPIO_SPI2_NSS);
    else
        gpio_clear(GPIOB, GPIO_SPI2_NSS);
}

static void spi_setup(void)
{
    nvic_enable_irq(NVIC_SPI2_IRQ);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_MOSI | GPIO_SPI2_SCK);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI2_NSS);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_SPI2_MISO);

    spi_reset(SPI2);

    spi_init_master(SPI2,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_32,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);

    spi_enable(SPI2);
}

static unsigned char txbuf[256];
static volatile int txpos;
static volatile int txlast;

static void transmit_char(unsigned char data)
{
    USART_DR(USART1) = data;
}

void usart1_isr(void)
{
    if (USART_SR(USART1) & USART_SR_TC)
    {
        USART_SR(USART1) &= ~USART_SR_TC;
        /* byte has been transmitted */
        if (txpos != txlast)
        {
            int cur = txpos;
            txpos = (txpos + 1) % sizeof(txbuf);
            transmit_char(txbuf[cur]);
        }
    }
}

void print(const void *data, ssize_t len)
{
    int i;
    if (len < 0)
    {
        len = strlen(data);
    }
    bool empty = (txlast == txpos);
    for (i = 0; i < len; i++)
    {
        txbuf[txlast] = ((const char*)data)[i];
        txlast = (txlast + 1) % sizeof(txbuf);
    }
    txbuf[txlast] = '\n';
    txlast = (txlast + 1) % sizeof(txbuf);
    txbuf[txlast] = '\r';
    txlast = (txlast + 1) % sizeof(txbuf);
    if (empty)
    {
        int cur = txpos;
        txpos = (txpos + 1) % sizeof(txbuf);
        transmit_char(txbuf[cur]);
    }
}

static bool print_ready(void)
{
    return (txpos == txlast);
}

uint8_t rcvbuf[1518];
uint8_t mac[6] = {0x0C, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t ip[4] = {10, 55, 1, 200};

struct enc28j60_state_s state;
bool configured = false;

static void enc28j60setup(struct enc28j60_state_s *state)
{
    // interrupt pin
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1);

    // enable interrupt
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    exti_select_source(EXTI1, GPIOB);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI1);

    enc28j60_init(state, spi_rw, spi_cs);
    enc28j60_configure(state, mac, 4096, false);
    enc28j60_interrupt_enable(state, true);
}

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

void period_timer_reset(void)
{

}

void arp_timer_reset(void)
{
    
}

bool period_timer_expired(void)
{
    return false;    
}

bool arp_timer_expired(void)
{
    return false;    
}

static void read_pkt(void)
{
    uint32_t status, crc;
    int i;
    ssize_t len = enc28j60_read_packet(&state, uip_buf, sizeof(uip_buf), &status, &crc);
    if (len < 0)
        len = 0;

    uip_len = len;
    
    if (uip_len > 0)
    {
        print("Packet rcv", -1);
        if (BUF->type == htons(UIP_ETHTYPE_IP))
        {
            print("IP", -1);
            uip_arp_ipin();
            uip_input();
            /*  If the above function invocation resulted in data that
	            should be sent out on the network, the global variable
	            uip_len is set to a value > 0. */
            if (uip_len > 0)
            {
                uip_arp_out();
                enc28j60_send_data(&state, uip_buf, uip_len);
            }
        }
        else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
            print("ARP", -1);
            uip_arp_arpin();
            /*  If the above function invocation resulted in data that
	            should be sent out on the network, the global variable
	            uip_len is set to a value > 0. */
            if (uip_len > 0)
            {
                print("ARP response", -1);
                enc28j60_send_data(&state, uip_buf, uip_len);
            }
        }
    }
    else if (period_timer_expired())
    {
        period_timer_reset();
        for (i = 0; i < UIP_CONNS; i++)
        {
            uip_periodic(i);
            /* If the above function invocation resulted in data that
	         should be sent out on the network, the global variable
	         uip_len is set to a value > 0. */
            if (uip_len > 0)
            {
                uip_arp_out();
                enc28j60_send_data(&state, uip_buf, uip_len);
            }
        }

        /* Call the ARP timer function every 10 seconds. */
        if (arp_timer_expired())
        {
            arp_timer_reset();
            uip_arp_timer();
        }
    }
}

void exti1_isr(void)
{
    exti_reset_request(EXTI1);
    if (!configured)
    {   
        return;
    }
    while (enc28j60_has_package(&state))
    {
        read_pkt();
    }
}

void hard_fault_handler(void)
{
    int i;
    while (1)
    {
        for (i = 0; i < 0x400000; i++)
            __asm__("nop");
        gpio_set(GPIOC, GPIO13);
        for (i = 0; i < 0x400000; i++)
            __asm__("nop");
        gpio_clear(GPIOC, GPIO13);
    }
}

void shell_appcall(void)
{
    print("App call", -1);
}

int main(void)
{
    uip_ipaddr_t ipaddr;

    SCB_VTOR = (uint32_t) 0x08000000;

    clock_setup();
    usart_setup(9600);
    spi_setup();
    gpio_set(GPIOC, GPIO13);

    print("HELLO", -1);

    enc28j60setup(&state);


    uip_init();

    uip_ipaddr(ipaddr, 10,55,1,200);
    uip_sethostaddr(ipaddr);

    uip_ipaddr(ipaddr, 10,55,1,1);
    uip_setdraddr(ipaddr);

    uip_ipaddr(ipaddr, 255,255,255,0);
    uip_setnetmask(ipaddr);
    
    memcpy(uip_ethaddr.addr, mac, 6);

    uip_listen(HTONS(10000));

    configured = true;
    while(1)
    {
    }
    return 0;
}

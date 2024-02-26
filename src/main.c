#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <string.h>
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "lcd.h"
#include "data_packet.h"
#include "delay.h"
#include <assert.h>

#define U_PORT USART1

struct RadioDataPacket rdp = {0};
struct ControlDataPacket cdp = {0};
bool rdp_valid = false;

#define RECV_BUF_SIZE	100		/* Arbitrary buffer size */
char recv_buf[RECV_BUF_SIZE];
volatile uint8_t recv_ndx_nxt;		/* Next place to store */
volatile uint8_t recv_ndx_cur;		/* Next place to read */

void usart1_isr(void) {
    do {
        if (usart_get_flag(U_PORT, USART_FLAG_RXNE)) {
            recv_buf[recv_ndx_nxt] = usart_recv(U_PORT);

            /* Check for "overrun" */
            uint16_t i = (recv_ndx_nxt + 1) % RECV_BUF_SIZE;
            if (i != recv_ndx_cur) {
                recv_ndx_nxt = i;
            }
        }
    } while (usart_get_flag(U_PORT, USART_FLAG_RXNE)); /* can read back-to-back interrupts */
}

static char usart_getc(bool wait) {
    char c = 0;

    while (wait && recv_ndx_cur == recv_ndx_nxt);
    if (recv_ndx_cur != recv_ndx_nxt) {
        c = recv_buf[recv_ndx_cur];
        recv_ndx_cur = (recv_ndx_cur + 1) % RECV_BUF_SIZE;
    }
    return c;
}

static void usart_setup(void) {
    static_assert(U_PORT == USART1, "U_PORT isn't USART1!");
    /* Setup GPIO pins for USART transmit. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

    /* Setup USART parameters. */
    usart_set_baudrate(U_PORT, 115200);
    usart_set_databits(U_PORT, 8);
    usart_set_parity(U_PORT, USART_PARITY_NONE);
    usart_set_stopbits(U_PORT, USART_CR2_STOPBITS_1);
    usart_set_mode(U_PORT, USART_MODE_TX_RX);
    usart_set_flow_control(U_PORT, USART_FLOWCONTROL_NONE);

    /* Enable interrupts from the USART */
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Specifically enable recieve interrupts */
    usart_enable_rx_interrupt(U_PORT);

    /* Finally enable the USART. */
    usart_enable(U_PORT);
}

static void usart_tick(void) {
    static uint32_t last_millis = 0;
    if (millis() - last_millis >= 20) {
        last_millis = millis();
        char ch = usart_getc(false);
        if (ch == '#') {
            char buf[50];
            char *ptr = buf;
            uint8_t i;
            if_begin:

            for (i = 0; i < 5; i++) {
                *ptr = usart_getc(1);
                if (*ptr == '#') goto if_begin;
                ptr++;
            }
            *(ptr++) = ' ';
            for (i = 0; i < 5; i++) {
                *ptr = usart_getc(1);
                if (*ptr == '#') goto if_begin;
                ptr++;
            }
            *(ptr++) = ' ';
            for (i = 0; i < 5; i++) {
                *ptr = usart_getc(1);
                if (*ptr == '#') goto if_begin;
                ptr++;
            }
            *(ptr++) = ' ';
            for (i = 0; i < 5; i++) {
                *ptr = usart_getc(1);
                if (*ptr == '#') goto if_begin;
                ptr++;
            }
            *(ptr++) = ' ';
            *ptr = usart_getc(1);
            if (*ptr == '#') goto if_begin;
            ptr++;
            *(ptr++) = ' ';
            *ptr = usart_getc(1);
            if (*ptr == '#') goto if_begin;
            ptr++;
            *(ptr++) = ' ';
            *ptr = usart_getc(1);
            if (*ptr == '#') goto if_begin;
            ptr++;
            *ptr = usart_getc(1);
            if (*ptr == '#') goto if_begin;
            ptr++;
            *(ptr++) = 0;

            char *end;
            rdp.ch1 = strtol(buf, &end, 10);
            rdp.ch2 = strtol(end, &end, 10);
            rdp.ch3 = strtol(end, &end, 10);
            rdp.ch4 = strtol(end, &end, 10);
            rdp.gen_on = strtol(end, &end, 10);
            rdp.force_gen = strtol(end, &end, 10);
            rdp.hash = strtol(end, &end, 10);

            rdp_valid = rdp_check_hash(&rdp);
        }
    }
}

static void send_cdp(void) {
    char buf[5];
    sprintf(buf, "#%01d%02d", cdp.force_gen, cdp.hash);

    uint8_t len = strlen(buf);
    char *ptr = buf;
    while (len--) {
        usart_send_blocking(U_PORT, *(ptr++));
    }
}

static void i2c_setup(void) {
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_set_i2c_clock_hsi(I2C1);

    rcc_periph_reset_pulse(RST_I2C1);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
    i2c_peripheral_disable(I2C1);
    //configure ANFOFF DNF[3:0] in CR1
    i2c_enable_analog_filter(I2C1);
    i2c_set_digital_filter(I2C1, 0);
    /* HSI is at 8Mhz */
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
    //configure No-Stretch CR1 (only relevant in slave mode)
    i2c_enable_stretching(I2C1);
    //addressing mode
    i2c_set_7bit_addr_mode(I2C1);

    i2c_peripheral_enable(I2C1);
}

static void lcd_tick(void) {
    static uint32_t last_millis = 0;
    if (millis() - last_millis >= 200) {
        last_millis = millis();
        lcd_clear();

        char buf[17];
        if (rdp_valid) {
            snprintf(buf, 17, "c1:%hu c2:%hu", rdp.channels[0] * 100 / 65535, rdp.channels[1] * 100 / 65535);
            lcd_put_cur(0, 0);
            lcd_send_string(buf);

            snprintf(buf, 17, "c3:%hu c4:%hu", rdp.channels[2] * 100 / 65535, rdp.channels[3] * 100 / 65535);
            lcd_put_cur(1, 0);
            lcd_send_string(buf);

            if (rdp.force_gen) {
                lcd_put_cur(1, 14);
                lcd_send_string("f");
            }
            if (rdp.gen_on) {
                lcd_put_cur(1, 15);
                lcd_send_string("g");
            }
        } else {
            snprintf(buf, 17, "%s", "Invalid packet");
            lcd_put_cur(0, 0);
            lcd_send_string(buf);
        }
    }
}

static void do_click_action(void) {
    cdp.force_gen = !cdp.force_gen;
    cdp_gen_hash(&cdp);
    send_cdp();
}

bool been_pressed = false;
uint32_t press_time = 0;
bool click_started = false;

static void btn_tick(void) {
    if (!been_pressed && !gpio_get(GPIOA, GPIO0)) {
        press_time = millis();
        been_pressed = true;
        return;
    } else if (been_pressed) {
        if ((millis() - press_time) >= 70) {
            if (!gpio_get(GPIOA, GPIO0)) {
                if (!click_started) {
                    click_started = true;
                    do_click_action();
                } else {
                    //click already started. do nothing
                }
            } else {
                been_pressed = false;
                click_started = false;
            }
        } else {
            //do nothing. wait for debounce time
        }
    }
}

static void setup_btn(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0);
}

static void systick_setup(int xms) {
    /* div8 per ST, stays compatible with M3/M4 parts, well done ST */
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    /* clear counter so it starts right away */
    STK_CVR = 0;

    systick_set_reload(rcc_ahb_frequency / 8 / 1000 * xms);
    systick_counter_enable();
    systick_interrupt_enable();
}

int main(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    systick_setup(1);
    setup_btn();
    usart_setup();
    i2c_setup();
    lcd_init();

    while (1) {
        btn_tick();
        usart_tick();
        lcd_tick();
    }

    return 0;
}
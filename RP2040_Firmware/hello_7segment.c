/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/multicore.h"

#define FIRST_GPIO  0
#define SECOND_GPIO 20
#define GPIO_KEY    24
#define UART_ID uart0
#define TX_PIN 28
#define RX_PIN 29
#define BUFFER_SIZE 64

static uint8_t uart_buffer[BUFFER_SIZE];
static uint8_t uart_pos;
static critical_section_t crit_sec;

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
	uart_buffer[uart_pos] = uart_getc(UART_ID);
	uart_pos++;
    }
}

/* core1 toggles GPIO0 ~ GPIO15, GPIO20 ~ GPIO27 */
void core1_entry() {
    while (1) {
	// To check the Key input
	if (!gpio_get(GPIO_KEY))
	{
	    // Do nothing here...
	}
	else
	{
            // Set all our GPIOs in one go!
            // If something else is using GPIO, we might want to use gpio_put_masked()
            gpio_set_mask(0xef0ffff);
            sleep_ms(500);
            gpio_clr_mask(0xef0ffff);
            sleep_ms(500);
	}
    }
}

/// \tag::hello_gpio[]
int main() {
    int stdin_char;
    char tx_data = 'A';
    uint8_t buffer_l[BUFFER_SIZE] = {0xFF};
    uint8_t count_l = 0;
    uint8_t i = 0;

    uart_pos = 0;

    /*
    ** Initialise a critical_section structure allowing
    ** the system to assign a spin lock.
    */
    critical_section_init (&crit_sec);

    /* STD initializations */
    stdio_init_all();
    stdio_usb_init();

    /* Initialise GPIO0 to GPIO15 here */
    for (int gpio = FIRST_GPIO; gpio < FIRST_GPIO + 16; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }

    /* Initialise GPIO20 to GPIO27 here */
    for (int gpio = SECOND_GPIO; gpio < SECOND_GPIO + 8; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }

    /* Initialise GPIO24 as KEY input */
    gpio_init(GPIO_KEY);
    gpio_set_dir(GPIO_KEY, GPIO_IN);

    /* UART0-8-N-1-9600 */
    /* Setup baudrate */
    uart_init(UART_ID, 9600);
    /* Set GPIO pin 28 to UART function */
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    /* Set GPIO pin 29 to UART function */
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    /* Disable flow control CTS/RTS */
    uart_set_hw_flow(UART_ID, false, false);
    /* Setup data format */
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    printf("UART connection is enabled!\r\n");

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    /* Launch core1 */
    multicore_launch_core1(core1_entry);

    while (true) {
 	/*
	** Process the data received from UART RX
	*/
	critical_section_enter_blocking (&crit_sec);
	memcpy(buffer_l, uart_buffer, uart_pos);
	count_l = uart_pos;
	uart_pos = 0;
	critical_section_exit (&crit_sec);
	for(i = 0; i < count_l; i++)
	{
	    printf("%c", buffer_l[i]);
	}
	/*
	** Process the data received from USB
	*/
	stdin_char = getchar_timeout_us(0);
        if(stdin_char != PICO_ERROR_TIMEOUT)
        {
            tx_data = stdin_char;
            uart_tx_wait_blocking(UART_ID);     //Wait for clear TX FIFO
            uart_putc_raw(UART_ID, tx_data);
        }
    }

    return 0;
}
/// \end::hello_gpio[]

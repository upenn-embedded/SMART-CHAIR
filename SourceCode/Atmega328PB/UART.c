#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#define RX_BUFFER_SIZE 50

volatile static char uart_rx_buffer[RX_BUFFER_SIZE];
volatile static uint8_t uart_rx_index = 0;

volatile static uint8_t uart_message_ready = false; // Set to 1 when \r\n received

// ---- UART1 (USART1) for ESP32 ----
// 9600 baud @ 16 MHz -> UBRR = 103
#define UART1_UBRR_9600  8

void UART1_Init(void)
{
    // Set baud rate
    UBRR1H = (uint8_t)(UART1_UBRR_9600 >> 8);
    UBRR1L = (uint8_t)(UART1_UBRR_9600 & 0xFF);

    // Enable transmitter (and receiver if you want RX from ESP32)
    UCSR1B = (1 << TXEN1);              // add (1 << RXEN1) if needed

    // 8N1: 8 data bits, no parity, 1 stop bit
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void UART1_SendChar(char c)
{
    while (!(UCSR1A & (1 << UDRE1))) {
        // wait for transmit buffer empty
    }
    UDR1 = c;
}

void UART1_SendString(const char *s)
{
    while (*s) {
        UART1_SendChar(*s++);
    }
}


void UART0_Init(void)
{
    uint16_t ubrr = 8;  // 115200 baud @ 16 MHz

    UBRR0H = (ubrr >> 8);
    UBRR0L = (ubrr & 0xFF);

    UCSR0B = (1 << RXEN0)    // Enable RX
           | (1 << TXEN0)    // Enable TX
           | (1 << RXCIE0);  // Enable RX interrupt

    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

bool UART_isRxMsgReady(void)
{
    return uart_message_ready;
}

void UART_ResetRxMsgReadyFlag(void)
{
    uart_message_ready = false;
}

void UART_ResetRxMsgIndex(void)
{
    uart_rx_index = 0;
}

static void UART0_SendChar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void UART0_SendString(const char *s)
{
    while (*s)
    {
        UART0_SendChar(*s++);
    }
}

ISR(USART0_RX_vect)
{
    char c = UDR0;  // Read received character

    // If message is already ready, ignore incoming data until processed
    if (uart_message_ready)
        return;

    // If buffer full, reset (avoid overflow)
    if (uart_rx_index >= (RX_BUFFER_SIZE - 1))
    {
        uart_rx_index = 0;
    }

    uart_rx_buffer[uart_rx_index++] = c;

    // Check if last two characters are "\r\n"
    if (uart_rx_index >= 2 &&
        uart_rx_buffer[uart_rx_index - 2] == '\r' &&
        uart_rx_buffer[uart_rx_index - 1] == '\n')
    {
        uart_rx_buffer[uart_rx_index] = '\0';  // Null-terminate string
        uart_message_ready = true;                // Tell main loop message is complete
    }
}


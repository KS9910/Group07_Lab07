#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// Define UART2, GPIO pins, and buttons
#define SW1_PIN   (1 << 4)  // Assume SW1 is connected to PF4
#define SW2_PIN   (1 << 0)  // Assume SW2 is connected to PF0
#define LED_GREEN (1 << 3)  // Assume Green LED is on PF3
#define LED_BLUE  (1 << 2)  // Assume Blue LED is on PF2
#define LED_RED   (1 << 1)  // Assume Red LED is on PF1

// Initialize UART2 for 9600 baud with odd parity
void UART2_Init(void) {
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;     // Enable UART2 clock
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;     // Enable GPIO Port D clock
        while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {} // Wait for Port D to be ready

        // Configure PD6 as TX and PD7 as RX for UART2
        GPIO_PORTD_AFSEL_R |= (1 << 6) | (1 << 7);   // Enable alternate functions for PD6 (TX) and PD7 (RX)
        GPIO_PORTD_PCTL_R &= ~((0xF << 24) | (0xF << 28)); // Clear the PCTL values for PD6 and PD7
        GPIO_PORTD_PCTL_R |= (0x1 << 24) | (0x1 << 28);  // Set PCTL for PD6 as U2TX and PD7 as U2RX
        GPIO_PORTD_DEN_R |= (1 << 6) | (1 << 7);     // Enable digital I/O for PD6 and PD7

        UART2_CTL_R &= ~UART_CTL_UARTEN;              // Disable UART2 to configure it
        UART2_IBRD_R = 130;                          // Set integer part of baud rate (9600 baud)
        UART2_FBRD_R = 13;                           // Set fractional part of baud rate
        UART2_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS); // 8 data bits, odd parity, 1 stop bit

        UART2_CTL_R |= UART_CTL_LBE;                 // Enable loopback mode (set 7th bit of UARTCTL)
        UART2_CTL_R |= UART_CTL_UARTEN;               // Enable UART2
}

// Write a byte of data to UART2
void UART2_Write(uint8_t data) {
    while((UART2_FR_R & UART_FR_TXFF) != 0) {}   // Wait until TX FIFO is not full
    UART2_DR_R = data;                           // Send data
}

// Read a byte of data from UART2
uint8_t UART2_Read(void) {
    if (UART2_FR_R & UART_FR_RXFE) {
        return 0;  // No data available
    }
    return (uint8_t)(UART2_DR_R & 0xFF); // Read received data
}

// Initialize Port F for LEDs and switches
void PortF_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000020; /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B; /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F; /* make PORTF0 configurable */
    GPIO_PORTF_DEN_R = 0x1F; //1E /* set PORTF pins 4 pin */
    GPIO_PORTF_DIR_R = 0x0E; /* set PORTF4 pin as input user switch pin */
    GPIO_PORTF_PUR_R = 0x11; //10 /* PORTF4 is pulled up */
}

// Main function
int main(void) {
    UART2_Init();    // Initialize UART2
    PortF_Init();    // Initialize Port F for switches and LEDs

    while(1) {
        int present=GPIO_PORTF_DATA_R & 0x10;
        int present_2=GPIO_PORTF_DATA_R & SW2_PIN;

        // Check if SW1 is pressed and send 0xF0
        if (present==0x0) {
            UART2_Write(0xF0);
            //GPIO_PORTF_DATA_R = LED_RED;// Send 0xF0 if SW1 is pressed
        }
        // Check if SW2 is pressed and send 0xAA
        else if (present_2==0x0) {
            UART2_Write(0xAA);  // Send 0xAA if SW2 is pressed
            //GPIO_PORTF_DATA_R = LED_GREEN;
        }

        // Continuously check for incoming data
        uint8_t receivedData = UART2_Read();

        // If valid data is received, handle it
        if (receivedData != 0) {
            if (receivedData == 0xAA) {
                GPIO_PORTF_DATA_R = LED_GREEN; // Turn green LED if 0xAA received
            }
            else if (receivedData == 0xF0) {
                GPIO_PORTF_DATA_R = LED_BLUE;  // Turn blue LED if 0xF0 received
            }
            else {
                GPIO_PORTF_DATA_R = LED_RED;   // Turn red LED for any other data or error
            }
        }
    }
}

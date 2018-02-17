/*
 * Hear Rate Monitor Microprocessor Code
 *
 * Written by:
 * John Careaga
 * Timothy Chong
 *
 */

#include "msp430g2553.h"

// Baud Rate Parameters:
#define SMCLKRATE 8000000								   // Sub-main clock speed.
#define BAUDRATE 115200									   // Baud rate for UART serial communication.
#define BRDIV16 ((16*SMCLKRATE)/BAUDRATE)	 // Baud rate division factor multiplied by 16.
#define BRDIV (BRDIV16/16)								 // Baud rate division factor.
#define BRMOD ((BRDIV16-(16*BRDIV)+1)/16)	 // Baud rate modulation value.
#define BRDIVHI (BRDIV/256)								 // Upper byte of Baud rate prescaler.
#define BRDIVLO (BRDIV-BRDIVHI*256)			   // Lower byte of Baud rate prescaler.

// UART Parameters:
#define TXBIT 0x04  // MSP UART transmit pin bit.
#define RXBIT 0x02	// MSP UART receive pin bit.

// Receive Bytes:
#define START 106		// Receive op-code to start heart rate monitoring.
#define STOP 182		// Receive op-code to stop heart rate monitoring.

// Sending Bytes:
#define READY 198							// Send op-code to indicate heart rate data transmission is ready.
#define START_SUCCESSFUL 107	// Send op-code to indicate monitoring has begun.
#define STOP_SUCCESSFUL 183		// Send op-code to indicate monitoring has terminated.

// UART data transmission state:
#define SEND_BYTE_PULSE_READY 0					// send_byte() will send the READY op-code.
#define SEND_BYTE_BYTE_1 1							// send_byte() will send the first byte of heart rate data.
#define SEND_BYTE_BYTE_2 2							// send_byte() will send the second byte of heart rate data.
#define SEND_BYTE_START_SUCCESSFUL 3		// send_byte() will send the START_SUCCESSFUL op-code.
#define SEND_BYTE_STOP_SUCCESSFUL 4			// send_byte() will send the STOP_SUCCESSFUL op-code.

// GPIO parameters:
#define INBIT 8	  // Input pin for pulse signal (P1.3).

// MSP Data Recording State Machine Parameters:
#define INACTIVE 0		// MSP is not acquiring heart rate data.
#define ACTIVE 1			// MSP is actively acquiring heart rate data.

// Global variables:
unsigned char send_state;				         // MSP UART data send state variable.
unsigned char read = 0;					         // Variable to read the MSP receive buffer.
unsigned char state = INACTIVE;	         // MSP heart rate data recording system state.
unsigned char interrupt_count_low = 0; 	 // Least significant byte of heart rate data
unsigned char interrupt_count_high = 0;	 // Most significant byte of heart rate data
unsigned char pin_state = 0;	 	         // State variable used to detect signal edges.
unsigned int interrupt_count = 0;        // Number of WDT interrupts counter.
unsigned int down_count = 0;	 	         // Number of interrupts that occur while input signal is low.
unsigned int half_count = 0;		         // Half the number of interrupts that occur while input signal is high.

// @group UART SECTION

void init_uart() {
  UCA0CTL1 = UCSWRST;		// USCI module in reset condition.

  // Parity bit disabled.
  // One stop bit.
  // Receive and transmit shift registers in LSB first mode.
  // Character length: 8 bits.
  // UART mode.
  // Asynchronous mode.

  UCA0CTL1 |= UCSSEL_2;	// USCI clock source set to SMCLK.
  UCA0BR1 = BRDIVHI;		// Clock prescaler setting of the Baud rate generator (high byte).
  UCA0BR0 = BRDIVLO;		// Clock prescaler setting of the Baud rate generator (low byte).
  UCA0MCTL = 2*BRMOD;		// Set modulation setting.
  P1SEL |= TXBIT;				// Set P1.2 to UART transmit data bus function.
  P1SEL2 |= TXBIT;			// Set P1.2 to UART transmit data bus function.
  P1SEL |= RXBIT;				// Set P1.1 to UART receive data bus function.
  P1SEL2 |= RXBIT;			// Set P1.1 to UART receive data bus function.
  UCA0CTL1 &= ~UCSWRST;	// USCI reset released for operation.
  IE2 |= UCA0RXIE;			// Enable UART receive interrupts.
  IE2 &= ~UCA0TXIE;			// Disable UART transmit interrupts. (Included for clarity).
}

void send_byte() {
  while (UCA0STAT & UCBUSY) {
  };

  switch(send_state) {
    case SEND_BYTE_START_SUCCESSFUL:
      UCA0TXBUF = START_SUCCESSFUL;
      send_state = SEND_BYTE_PULSE_READY;
      state = ACTIVE;
      IE1 |= WDTIE; // Enable WDT interrupts
      break;

    case SEND_BYTE_STOP_SUCCESSFUL:
      UCA0TXBUF = STOP_SUCCESSFUL;
      break;

    case SEND_BYTE_PULSE_READY:
      UCA0TXBUF = READY;
      send_state = SEND_BYTE_BYTE_1;
      break;

    case SEND_BYTE_BYTE_1:
      UCA0TXBUF = interrupt_count_low;
      send_state = SEND_BYTE_BYTE_2;
      break;

    case SEND_BYTE_BYTE_2:
      UCA0TXBUF = interrupt_count_high;
      send_state = SEND_BYTE_PULSE_READY;
      break;

    default:
      break;
  }
}

// MSP UART receive interrupt handler:
void interrupt rx_handler() {
 	read = UCA0RXBUF;  // Read received by the MSP from the computer.

  if (read == START) {
    send_state = SEND_BYTE_START_SUCCESSFUL;
    send_byte();
  }
  else if (read == STOP) {
    send_state = SEND_BYTE_STOP_SUCCESSFUL;
    state = INACTIVE;
    IE1 &= ~WDTIE;  // Disable Watch Dog Timer interrupts.
    interrupt_count = 0;
    half_count = 0;
    down_count = 0;
    send_byte();
  }
}
// Declare rx_handler as the USCI receive interrupt handler.
ISR_VECTOR(rx_handler, ".int07")

// @endgroup
// @group GPIO SECTION

void init_gpio() {
  P1DIR &= ~INBIT;	// Set P1.3 as an input
  P1IE &= ~INBIT;		// Disable P1.3 interrupt triggering.

  // P1.3 selected as GPIO.
  // No pullup resistors needed.
  // @note this function is redundant as these commands do not change anything; included for clarity.
}

// @endgroup
// @group WDT SECTION

void init_wdt() {
  WDTCTL = (WDTPW + WDTTMSEL + WDTCNTCL + 2);
  // Include the password.
  // Set WDT to interval mode.
  // Clear the WDT counter to 0.
  // Use SMCLK as WDT clock source but divide this source by 512 to trigger interrupts every 512/8M seconds .
}

// WDT handler:
interrupt void wdt_handler() {
  interrupt_count ++;

  if(state == ACTIVE) {
    if((P1IN & INBIT) && !pin_state) { // rising edge
      pin_state = 1;  // Toggle to detect the next falling edge.
      down_count = interrupt_count;
      interrupt_count = 0;
    }
    else if(!(P1IN & INBIT) && pin_state) { // falling edge
      pin_state = 0;	// Toggle to detect the next rising edge.
      volatile unsigned i = half_count;			// half the number of interrupts that occurred while the input signal was high during the previous pulse.
      half_count = interrupt_count / 2;			// Update half_count for the most recent pulse.

      if ((i != 0) && (down_count > 1)) {		// If the old value of half_count is not zero and down_count is not the minimum
        i += half_count + down_count;			// i = old half_count + updated half_count + current down_count
        interrupt_count_low = 0xFF & i;	  // least significant byte of 'i'.
        interrupt_count_high = i >> 8;		// most significant byte of 'i'.

        // If the USCI is not busy transmitting data and the UART data send state is set to pulse ready:
        if(!(UCA0STAT & UCBUSY) && (send_state == SEND_BYTE_PULSE_READY)) {
          send_byte();  // Send the pulse ready op-code
          send_byte();	// Send the first byte of heart rate data.
          send_byte();	// Send the second byte of heart rate data.
        }
      }

      interrupt_count = 0;
      send_state = SEND_BYTE_PULSE_READY;
    }
  }
}
ISR_VECTOR(wdt_handler, ".int10")

// @endgroup
// @group MAIN SECTION

void main() {
  WDTCTL = WDTPW + WDTHOLD;	// Stop the WDT.
  BCSCTL1 = CALBC1_8MHZ;	  // Calibrate SMCLK to 8MHz.
  DCOCTL = CALDCO_8MHZ;			// Calibrate SMCLK to 8MHz.

  init_gpio();
  init_wdt();
  init_uart();

  // Enter low power mode, enable general interrupts.
  _bis_SR_register(LPM0_bits + GIE);
}

// @endgroup

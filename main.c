//***************************************************************************************
//  Customer MSP430 Flasher Firmware
//
//  Description; Flashes the target MSP430 using it's built-in UART BSL.
//  Receives information from backchannel UART, tells target BSL to:
//      1) Wipe target device using the incorrect password method.
//      2) Write any bytes the connected PC tells this device to write to the target.
//      3) Inform the controlling PC of any UART or other errors.
//
//  Intended to work on the MSP-EXP430FR6989 only (for now).
//
//                MSP-EXP430FR6989
//             -------------------------
//         /|\|              eUSCA1 UART| <-> Connection to PC
//          | |              eUSCA0 UART| <-> Connection to Target Device
//          --|RST                  P8.4|-->Target RST
//            |                     P8.5|-->Target TST
//            |                     P1.0|-->Tx Status (LED 1 on Launchpad)
//            |                     P9.7|-->Rx Status (LED 2 on Launchpad)
//
//  Texas Instruments, Inc
//  July 2013
//***************************************************************************************

#include <msp430.h>

#include <stdint.h>

void init_cs() {
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.

    // Configure FRAM to add wait states.
    // This function sets the MCLK to 16 MHz, but the FRAM can only be
    // read at 8 MHz. To handle this, we set a single wait state to limit
    // reads to 8 MHz.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // first, unlock the registers by setting the password.
    CSCTL0_H = CSKEY >> 8; // unlock the clock system registers.

    // next set the DCO speed to 16 MHz
    // might be worth using MODCLK instead for this in the future.
    CSCTL1 = DCOFSEL_4 | DCORSEL; // per table 3-5 in the users guide, this sets clock to 16 MHz

    // set the MCLK and SMCLK to use the DCO as their oscillator
    // set ACLK to use the VLOCLK (maybe use a timing crystal instead in the future)
    CSCTL2 = SELA__VLOCLK | SELM__DCOCLK | SELS__DCOCLK;

    // set dividers. Run everything at 16 MHz
    CSCTL3 = DIVA__1 | DIVM__1 | DIVS__1;

    // re-lock the CS registers
    CSCTL0_H = 0;
}

void init_backchannel_uart()
{
    // init pins
    P3OUT &= ~(BIT4 | BIT5); // clear these pins if they're already set.
    P3OUT |= BIT4; // P3.4 is the Tx pin, set it to an output.
                   // Datasheet says it doesn't matter, but I do this anyway.

    // per table 6-25 in the datasheet, set pins to function as Tx/Rx pins.
    P3SEL0 |= (BIT4 | BIT5); // set bits 4 and 5 to one
    P3SEL1 |= ~(BIT4 | BIT5); // clear bits 4 and 5 so that this is zero


    // init eUSCI_A1
    UCA1CTLW0 = UCSWRST; // RESET eUSCIA1 so that we can set registers
    UCA1CTLW0 |= UCSSEL__SMCLK; // Set this to use the SMCLK as it's source
    // Per table 30-5 in the user's guide
    UCA1BRW = 104; // table 30-5, USCA1BRW
    // UCA1MCTLW most significant 8 bits is UCBRSx, next four bits are USBRF, then 3 spare, then UCOS16 flag
    UCA1MCTLW = (UCOS16 | UCBRF_2 | 0xD600); // 16x oversampling enabled, UCBRF set to 2, D6 for USCBRS
    UCA1CTLW0 &= ~UCSWRST; // restart eUSCI_A1

    // enable the eUSCI Rx interrupt
    UCA1IE = UCRXIE | UCTXIE;
}

void init_flashing_uart()
{
    // init pins
    P3OUT &= ~(BIT4 | BIT5); // clear these pins if they're already set.
    P3OUT |= BIT4; // P3.4 is the Tx pin, set it to an output.
                   // Datasheet says it doesn't matter, but I do this anyway.

    // per table 6-25 in the datasheet, set pins to function as Tx/Rx pins.
    P3SEL0 |= (BIT4 | BIT5); // set bits 4 and 5 to one
    P3SEL1 |= ~(BIT4 | BIT5); // clear bits 4 and 5 so that this is zero


    // init eUSCI_A1
    UCA1CTLW0 = UCSWRST; // RESET eUSCIA1 so that we can set registers
    UCA1CTLW0 |= UCSSEL__SMCLK; // Set this to use the SMCLK as it's source
    // Per table 30-5 in the user's guide
    UCA1BRW = 104; // table 30-5, USCA1BRW
    // UCA1MCTLW most significant 8 bits is UCBRSx, next four bits are USBRF, then 3 spare, then UCOS16 flag
    UCA1MCTLW = (UCOS16 | UCBRF_2 | 0xD600); // 16x oversampling enabled, UCBRF set to 2, D6 for USCBRS
    UCA1CTLW0 &= ~UCSWRST; // restart eUSCI_A1

    // enable the eUSCI Rx interrupt
    UCA1IE = UCRXIE | UCTXIE;
}

void init_led_pins()
{
    P1OUT &= ~BIT0;                         // clear P1.0 OUT if it's set.
    P1DIR |= BIT0;                          // Set P1.0 to output direction

    P9OUT &= ~BIT7;                         // clear P9.7 OUT if it's set.
    P9DIR |= BIT7;                          // Set P9.7 to output direction.
}

void init_rst_and_tst_pins()
{
    P8OUT &= ~(BIT4 | BIT5);
    P8DIR |= (BIT4 | BIT5);
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    init_cs(); // set clock system to 16 MHz

    init_backchannel_uart(); // set up back channel UART

    init_led_pins(); // setup Tx/Rx pins

    init_rst_and_tst_pins(); // setup pins connected to target RST/TST

    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

    __bis_SR_register(LPM3_bits | GIE);       // Enter LPM3, interrupts enabled
    __no_operation();
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  // handle events from the backchannel UART connection to PC
  switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      while(!(UCA1IFG&UCTXIFG)); // wait until TX Buffer is ready to accept a new character
      UCA1TXBUF = UCA1RXBUF + 1;
      P1OUT ^= BIT0;
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}


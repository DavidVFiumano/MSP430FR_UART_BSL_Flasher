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

typedef uint8_t pc_command_t;

enum COMMAND {
    NO_OP = 0x0,
    CLEAR_TARGET = 0x1,
    WRITE_DATA = 0x2
};

struct PC_Command {
    pc_command_t command_type;
    int16_t length; // command length
    uint32_t crc32; // CRC32 of command, including this header and underlying data
};

typedef uint8_t bsl_command_t;

enum TARGET_BSL_COMMAND {
    RX_DATA_BLOCK = 0x10,
    RX_PASSWORD = 0x11,
    MASS_ERASE = 0x15,
    CRC_CHECK = 0x16,
    LOAD_PC = 0x17,
    TX_DATA_BLOCK = 0x18,
    TX_BSL_VERSION = 0x19,
    RX_DATA_BLOCK_FAST = 0x1B,
    CHANGE_BAUD_RATE = 0x52
};

struct target_bsl_command_message {
    bsl_command_t cmd;
    uint16_t data_length;
    uint8_t* data;
};

//#pragma RETAIN(pc_command_buffer)
struct PC_Command pc_command_buffer;

volatile uint8_t target_tx_buffer[8192] = {0};
volatile uint8_t target_rx_buffer[256] = {0};

volatile uint8_t pc_tx_buffer[256] = {0};
volatile uint8_t pc_rx_buffer[8192] = {0};

volatile uint16_t target_tx_buffer_pos = 0;
volatile uint16_t target_rx_buffer_pos = 0;

volatile uint16_t target_tx_buffer_len = 0;
volatile uint16_t target_rx_buffer_len = 0;

volatile uint16_t pc_tx_buffer_len = 0;
volatile uint16_t pc_rx_buffer_len = 0;

volatile uint16_t pc_tx_buffer_pos = 0;
volatile uint16_t pc_rx_buffer_pos = 0;

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
    UCA1IFG = 0;
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

void activate_bsl()
{
    // to activate the BSL, the test pin needs two high clock edges
    // they must last
    P8OUT |= BIT5;
    __delay_cycles(4000); // wait ~250 uS before setting the pin low, then high again.

    // drop TST pin low for 250 uS.
    P8OUT &= ~BIT5;
    __delay_cycles(4000);

    // raise TST pin high again
    P8OUT |= ~BIT5;
    __delay_cycles(2000); // only need 125 uS

    // raise RST pin
    P8OUT |= BIT4;
    __delay_cycles(4000); // wait 250 uS

    // lower TST pin and MCU should BSL should start
    P8OUT &= ~BIT5;

}

void send_target_cmd(struct target_bsl_command_message* msg)
{
    // after this function exits, the user should be able to read the rx buffer to see the response
    // goal is to build packet to send to target
    // target has a header, length, command & data, and a checksum
    while (target_tx_buffer_pos != 0); // wait for whatever's in there to be out before filling it
    target_tx_buffer[0] = 0x80; // start with the UART header, which is always 0x80
    target_tx_buffer[1] = (msg->data_length + 1) & 0xFF;
    target_tx_buffer[2] = (msg->data_length + 1) >> 8;
    target_tx_buffer[3] = msg->cmd & 0xFF;

    CRCINIRES = 0xFFFF; // calculate the CRC 16 while we do this
    // the CKL and CKH are calculated only from the command and associated data, not the header.
    // this is not well documented, I discovered this by reading/running the python code posted
    // at this thread here: https://e2e.ti.com/support/microcontrollers/msp-low-power-microcontrollers-group/msp430/f/msp-low-power-microcontroller-forum/437559/calculating-msp430-bsl-checksum
    // Credit to Jason Gramse for his algorithm.
    CRCDIRB = msg->cmd;
    __no_operation();
    uint16_t i = 0;
    for (i = 0; i < msg->data_length; i++)
    {
        target_tx_buffer[i + 4] = msg->data[i];
        CRCDIRB = msg->data[i];
        __no_operation();
    }

    uint16_t crc16 = CRCINIRES;
    target_tx_buffer[4+msg->data_length] = crc16 & 0xFF; // CKL
    target_tx_buffer[5+msg->data_length] = crc16 >> 8; // CKH

    target_tx_buffer_pos = 1;
    target_tx_buffer_len = 4 + msg->data_length;
    UCA1TXBUF = target_tx_buffer[0]; // TODO update to go to target and not here.

    while (target_tx_buffer_pos != 0);
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    init_cs(); // set clock system to 16 MHz

    init_backchannel_uart(); // set up back channel UART

    init_led_pins(); // setup Tx/Rx pins

    //init_rst_and_tst_pins(); // setup pins connected to target RST/TST

    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

    uint8_t one_byte_array = 0;
    struct target_bsl_command_message cmd = {0x15, 0, &one_byte_array};

    target_tx_buffer_len = 0;
    target_tx_buffer_pos = 0; // TODO init others.

    __bis_SR_register(GIE);

    send_target_cmd(&cmd);

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
      if (target_rx_buffer_pos == target_rx_buffer_len)
      {
          P1OUT &= ~BIT0;
      } else {
          P1OUT |= BIT0;
          target_rx_buffer[target_rx_buffer_pos] = UCA1RXBUF;
          target_rx_buffer_pos += 1;
      }
      break;
    case USCI_UART_UCTXIFG:
      if (target_tx_buffer_pos == target_tx_buffer_len)
      {
          P9OUT &= ~BIT7; // transmitting complete, turn the LED off
          target_tx_buffer_pos = 0;
          target_tx_buffer_len = 0;
      } else {
          P9OUT |= BIT7; // transmitting ongoing, turn LED on
          UCA1TXBUF = target_tx_buffer[target_tx_buffer_pos];
          target_tx_buffer_pos += 1;
      }
      break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}


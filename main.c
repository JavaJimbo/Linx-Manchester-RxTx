/*********************************************************************************************
 * MAIN file for LINX MANCHESTER RXTX
 * For PIC32MX795F512X boards
 * 
 * 11-18-2016: Got transmitting and receiving working beautifully 
 * with Manchester coding on Parallax Linx 413 Mhz boards.
 * 11-19-2016: Uses all START pulses
 * 11-20-2016: Added LINXUART to receive RS232 data from magic wands at 9600 baud
 * 11-26-2016: Fixed problem in RECEIVER interrupt for much smoother operation with fewer errors.
 * 11-27-16: Simplified Manchester transmitting / receiving and greatly improved reliability.
 *  Transmit errors rarely occur. Distance improved significantly. 
 *  Switching PR2 register between 100 uS and 10 uS interrupts is working well.
 *  Added multibyte packets and CRC check. Works great!
 * 11-28-16: Works with Linx 8 pushbutton transmitter.
 * 12-1-16: Linx UART test
 * 12-6-16: Got MAGICWAND working again.
 *      Manchester coding, works well, limit to 8 byte packets, 
 *      short pulses = 100 uS, long pulses = 300 uS
 *      Use 200 uS as MAXBITLENGTH 20
 *      Use interrupt on change instead of Timer 2 interrupts,
 *      use Timer 4 interrupts to enable interrupt on change every 100 uS
 * 
 *      CLEANED UP CODE, eliminated transmitter and LINX keypad options,
 *      So this version only implements a MANCHESTER RECEIVER for 10 kHz data.
 *********************************************************************************************/

#include <plib.h>
#include <string.h>
#include <ctype.h>

#define true TRUE
#define false FALSE

#define MAXBITLENGTH 20
#define START_ONE 70 // was 29
#define START_TWO 70 
#define START_THREE 30
#define START_FOUR 30
#define TIMEOUT 800

#define TEST_OUT PORTBbits.RB1 
#define TRIG_OUT LATBbits.LATB15
#define TRIGGER_IN PORTReadBits(IOPORT_B, BIT_2)
#define RX_IN PORTReadBits(IOPORT_D, BIT_14)

#define UART_TIMEOUT 400

#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select


/** V A R I A B L E S ********************************************************/
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR 

unsigned char dataReady = FALSE;

#define MAXBUFFER 128
unsigned char HOSTTxBuffer[MAXBUFFER];
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned short HOSTTxLength;
unsigned short HOSTRxLength = 0;

#define MAXPOTS 4
unsigned char arrPots[MAXPOTS];
unsigned char error = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void ConfigAd(void);

unsigned char RXstate = 0;

#define MAXDATABYTES 16
unsigned char arrData[MAXDATABYTES];
unsigned char numDataBytes = 0;
unsigned char timeoutFlag = FALSE;

int main(void) {
    unsigned short trialCounter = 0;
    unsigned short i;
    unsigned short CRCcheck;

    union {
        unsigned char CRCbyte[2];
        unsigned short CRCinteger;
    } convert;

    InitializeSystem();

    DelayMs(200);

    printf("\rTesting MANCHESTER RECEIVER AT 10 kHZ...");

    
    while (1) {
        if (numDataBytes) {
            if ((numDataBytes > MAXDATABYTES) || (numDataBytes < 4))
                printf("\r\rERROR: Bytes received: %d", numDataBytes);
            else {
                printf("\r\r#%d Bytes received: %d DATA: ", ++trialCounter, numDataBytes);
                for (i = 0; i < numDataBytes; i++) printf("%X, ", arrData[i]);
                CRCcheck = CRCcalculate(&arrData[1], numDataBytes - 3);
                convert.CRCbyte[0] = arrData[numDataBytes - 2];
                convert.CRCbyte[1] = arrData[numDataBytes - 1];
                if (convert.CRCinteger != CRCcheck) printf("\rCRC ERROR: %X != %X", CRCcheck, convert.CRCinteger);
                else printf("\rSUCCESS!: %X = %X", CRCcheck, convert.CRCinteger);
                if (timeoutFlag) printf("\rTIMEOUT");
                timeoutFlag = FALSE;
            }
            numDataBytes = 0;
        }
        if (error) {
            printf("\rError: %d", error);
            error = 0;
        }
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000


    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);

    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_2);

    PORTSetPinsDigitalIn(IOPORT_D, BIT_14);
    mCNOpen(CN_ON, CN20_ENABLE, CN20_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    PORTSetPinsDigitalOut(IOPORT_B, BIT_15 | BIT_1);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3 | BIT_13 | BIT_14 | BIT_4);

    // Set up Port G outputs:
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);

    // Set up Timer 2, no interrupts 
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T2CONbits.TCKPS1 = 1;
    T2CONbits.TCKPS0 = 1;
    T2CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR2 = 0xFFFF; 
    T2CONbits.TON = 1; // Let her rip   

    // Set up Timer 4 for 10 Khz (100 uS) interrupts
    T4CON = 0x00;
    T4CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 1;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 500;
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_5);
    T4CONbits.TON = 1; // Let her rip   

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define HOSTuart UART2
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    // INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);        

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit


// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned short HOSTRxIndex = 0;
    static unsigned char TxIndex = 0;
    unsigned char ch;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            if (ch != 0 && ch != '\n') {
                if (HOSTRxIndex < MAXBUFFER - 2)
                    HOSTRxBuffer[HOSTRxIndex++] = ch;
                if (ch == '\r') {
                    HOSTRxBuffer[HOSTRxIndex] = '\0';
                    HOSTRxLength = HOSTRxIndex;
                    HOSTRxIndex = 0;
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        if (HOSTTxLength) {
            if (TxIndex < MAXBUFFER) {
                ch = HOSTTxBuffer[TxIndex++];
                if (TxIndex <= HOSTTxLength) {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ch);
                } else {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    HOSTTxLength = false;
                    TxIndex = 0;
                }
            } else {
                TxIndex = 0;
                HOSTTxLength = false;
                INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
            }
        } else INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    }
}

unsigned long getLongInteger(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {

    union {
        unsigned char byte[4];
        unsigned long lngInteger;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    convert.byte[2] = b2;
    convert.byte[3] = b3;

    return (convert.lngInteger);
}

unsigned short getShort(unsigned char b0, unsigned char b1) {

    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    return (convert.integer);
}

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_0);
    mPORTBSetPinsAnalogIn(BIT_1);
    mPORTBSetPinsAnalogIn(BIT_2);
    mPORTBSetPinsAnalogIn(BIT_3);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set inputs to analog
#define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA

    // Only scan AN0, AN1, AN2, AN3 for now
#define PARAM5   SKIP_SCAN_AN4 |SKIP_SCAN_AN5 |SKIP_SCAN_AN6 |SKIP_SCAN_AN7 |\
                    SKIP_SCAN_AN8 |SKIP_SCAN_AN9 |SKIP_SCAN_AN10 |\
                      SKIP_SCAN_AN11 | SKIP_SCAN_AN12 |SKIP_SCAN_AN13 |SKIP_SCAN_AN14 |SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();

}

void __ISR(_ADC_VECTOR, ipl6) AdcHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned char) (ReadADC10(offSet + i) / 4); // read the result of channel 0 conversion from the idle buffer

}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    static unsigned short Timer2Counter = 0;
    static unsigned char byteMask = 0x0001;
    static unsigned char dataInt = 0x00;
    static unsigned char oddFlag = FALSE;
    static unsigned short dataIndex = 0;
    static unsigned char numExpectedBytes = 0;
    unsigned short PORTDin, RX_PIN;

    // Step #1 - always clear the mismatch condition first
    PORTDin = PORTDbits.RD14; //  PORTReadBits(IOPORT_D, BIT_14);

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    Timer2Counter = TMR2 / 100;
    TMR2 = 0x0000;

    if (!PORTDin) RX_PIN = 0;
    else RX_PIN = 1;

    if (RXstate < 5) ConfigIntCN(CHANGE_INT_OFF | CHANGE_INT_PRI_2);

    // RX_IN goes HIGH: first START
    if (RXstate == 0 && RX_PIN) RXstate++;
        // RX_IN goes LOW: second START
    else if (RXstate == 1) {
        if (Timer2Counter > START_ONE) RXstate++;
        else RXstate = 0;
        // RX_IN goes HIGH: third START      
    } else if (RXstate == 2) {
        if (Timer2Counter > START_TWO) RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: fourth START                 
    } else if (RXstate == 3) {
        if (Timer2Counter > START_THREE) RXstate++;
        else RXstate = 0;
        // RX_IN goes HIGH: dummy - even pulse               
    } else if (RXstate == 4) {
        if (Timer2Counter > START_FOUR) RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: dummy - odd pulse                  
    } else if (RXstate == 5) {
        TEST_OUT = 1;
        byteMask = 0x01;
        oddFlag = FALSE;
        dataInt = 0x00;
        error = 0;
        dataIndex = 0;
        numExpectedBytes = 0;
        RXstate++;
    } else {
        if (Timer2Counter > MAXBITLENGTH) {
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) numExpectedBytes = dataInt + 3;
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
            oddFlag = FALSE;
        } else if (oddFlag) {
            oddFlag = FALSE;
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) numExpectedBytes = 8; // dataInt + 3;
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
        } else oddFlag = TRUE;

        if (numExpectedBytes && (dataIndex >= numExpectedBytes)) {
            TEST_OUT = 0;
            numDataBytes = dataIndex;
            RXstate = 0;
        }
    }
    // end if (RX_IN != PreviousPINstate)
    if (RXstate && (Timer2Counter > TIMEOUT)) {
        timeoutFlag = TRUE;
        if (RXstate == 6) numDataBytes = dataIndex;
        RXstate = 0;
    }
} // end Timer2Handler(void)

void __ISR(_TIMER_4_VECTOR, ipl5) Timer4Handler(void) {
    mT4ClearIntFlag(); // clear the interrupt flag 
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
}


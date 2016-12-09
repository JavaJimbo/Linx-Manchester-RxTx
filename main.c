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
 * 
 * 12-7-16: Additional optimization. Uses interrupt on change.
 *      Can handle up to 64 bytes: length + data + CRC
 * 12-8-16; Implemented Linx communication for five button fob.
 * *********************************************************************************************/

#include <plib.h>
#include <string.h>
#include <ctype.h>

#define MANCHESTER
// #define LINXFOB

#ifdef LINXFOB
#define RIGHT_PUSHBUTTON 0x332B
#define DOWN_PUSHBUTTON 0xB333
#define LEFT_PUSHBUTTON 0x2B33
#define UP_PUSHBUTTON 0x32B3
#define CENTER_PUSHBUTTON 0x3333
#endif

#define true TRUE
#define false FALSE

#define MAXBITLENGTH 20

#ifdef LINXFOB
#define START_ONE 3000
#endif

#ifdef MANCHESTER
#define START_ONE 80
#endif

#define STOP 3000

#define START_TWO 80
#define START_THREE 40
#define START_FOUR 40
#define TIMEOUT 200

#define TEST_OUT LATBbits.LATB1
#define TRIG_OUT LATBbits.LATB15
#define TRIGGER_IN PORTReadBits(IOPORT_B, BIT_2)

#define UART_TIMEOUT 400

// For UBW32 Board with 8 Mhz crystal, this should yield 80 Mhz system & peripheral clock
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
unsigned short previousExpected = 0, numExpectedBytes = 0;

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

#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];


unsigned char timeoutFlag = FALSE;

#ifdef LINXFOB
#define NUM_FOB_INTEGERS 5
unsigned short arrFobIntegers[NUM_FOB_INTEGERS];
const char LinxHeader[6] = {0xCC, 0xD5, 0x55, 0x55, 0x55, 0x55};
unsigned short dataInt = 0;
unsigned short numBitsReceived = 0;
#define MAXBITS 128
unsigned short arrBits[MAXBITS];
unsigned short arrBitTimer[MAXBITS];
#endif

#ifdef MANCHESTER
unsigned short numBytesReceived = 0;

int main(void) {
    unsigned short trialCounter = 0;
    unsigned short i, j, k;
    unsigned short mask;
    unsigned short dataInt, pushKey;

    unsigned short CRCcheck;

    union {
        unsigned char CRCbyte[2];
        unsigned short CRCinteger;
    } convert;

    InitializeSystem();
    DelayMs(200);
    TEST_OUT = 0;

    printf("\rTESTING MANCHESTER CODE...");

    while (1) {
        if (numBytesReceived) {
            CRCcheck = CRCcalculate(&arrData[1], numBytesReceived - 3);
            convert.CRCbyte[0] = arrData[numBytesReceived - 2];
            convert.CRCbyte[1] = arrData[numBytesReceived - 1];

            printf("\r\r#%d: %d bytes received:", ++trialCounter, numBytesReceived);
            for (i = 0; i < numBytesReceived; i++) {
                if ((i % 8) == 0) printf("\r%X, ", arrData[i]);
                else printf("%X, ", arrData[i]);
            }
            if (convert.CRCinteger != CRCcheck) printf("\r\rCRC ERROR: %d != %d", convert.CRCinteger, CRCcheck);
            if (timeoutFlag) {
                printf("\rTIMEOUT");
                timeoutFlag = FALSE;
            }
            numBytesReceived = 0;
        }
        if (error) {
            printf("\rError: %X", error);
            error = 0;
        }
    }//end while
}//end main
#endif    


#ifdef LINXFOB    

int main(void) {
    unsigned short trialCounter = 0;
    unsigned short i, j, k;
    unsigned short mask;
    unsigned short dataInt, pushKey;

    InitializeSystem();
    DelayMs(200);
    TEST_OUT = 0;
    
    printf("\r\rTESTING LINX FOB COMMUNICATION");
    
    trialCounter = 0;
    while (1) {
        if (RXstate == 4) {
            if (numBitsReceived >= MAXBITS) printf("\rOVERRUN ERROR");
            else {
                i = 0;
                j = 0;
                k = 0;
                dataInt = 0x0000;
                mask = 0x0001;
                //printf("\rINTEGERS: ");
                while (i < numBitsReceived) {
                    if (arrBitTimer[i] > 0xC0) dataInt = dataInt | mask;
                    mask = mask << 1;
                    j++;
                    if (j >= 16) {
                        j = 0;
                        mask = 0x0001;
                        //printf("%X, ", dataInt);                        
                        if (k < NUM_FOB_INTEGERS) arrFobIntegers[k++] = dataInt;
                        dataInt = 0x0000;
                    }
                    i++;
                }
                pushKey = arrFobIntegers[3];
                printf("\r#%d %d bits received. KEY: ", trialCounter++, numBitsReceived);
                if (pushKey == RIGHT_PUSHBUTTON) printf("RIGHT");
                else if (pushKey == LEFT_PUSHBUTTON) printf("LEFT");
                else if (pushKey == UP_PUSHBUTTON) printf("UP");
                else if (pushKey == DOWN_PUSHBUTTON) printf("DOWN");
                else if (pushKey == CENTER_PUSHBUTTON) printf("CENTER");
                else printf("INVALID KEY: %X", pushKey);
            }
            DelayMs(50);
            numBitsReceived = 0;
            RXstate = 0;
        } // end if (RXstate == 4)
    } // end while(1)
} // end main())
#endif

static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_0);
    mCNOpen(CN_ON, CN2_ENABLE, CN2_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    PORTSetPinsDigitalOut(IOPORT_B, BIT_15 | BIT_1);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3 | BIT_13 | BIT_14 | BIT_4);
    // Set up Port G outputs:
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);

#ifdef LINXFOB    
    // Set up Timer 2, no interrupts 
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 1; // 1:256 Prescaler
    T2CONbits.TCKPS1 = 1;
    T2CONbits.TCKPS0 = 1;
    T2CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR2 = 0xFFFF;
    T2CONbits.TON = 1; // Let her rip   
#endif    

#ifdef MANCHESTER
    // Set up Timer 2, no interrupts 
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T2CONbits.TCKPS1 = 1;
    T2CONbits.TCKPS0 = 1;
    T2CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR2 = 0xFFFF;
    T2CONbits.TON = 1; // Let her rip   
#endif    

    // Set up Timer 4 for 10 Khz (100 uS) interrupts
    T4CON = 0x00;
    T4CONbits.TCKPS2 = 0; // 1:8 Prescaler    
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 1;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 1000;
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
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
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
            // UARTtimeout=UART_TIMEOUT;
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

/*
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) {
    mT2ClearIntFlag(); // clear the interrupt flag    
}
*/

void __ISR(_TIMER_4_VECTOR, ipl5) Timer4Handler(void) {
    mT4ClearIntFlag(); // clear the interrupt flag 
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
}

#ifdef LINXFOB
#define FOB_PACKET_BITS 81
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    static unsigned short Timer2Counter = 0;
    static unsigned short i = 0, j = 0;
    unsigned short PORTin, RX_PIN;
    static unsigned char dataIndex = 0, bitIndex = 0;
    static unsigned int mask = 0, dataInt = 0;

    // Step #1 - always clear the mismatch condition first
    PORTin = PORTBbits.RB0;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    Timer2Counter = TMR2;
    TMR2 = 0x0000;

    if (!PORTin) RX_PIN = 0;
    else RX_PIN = 1;

    if (!RX_PIN && !RXstate)
        RXstate++;
    else if (RXstate == 1) {
        if (Timer2Counter > START_ONE) {
            RXstate++;
            dataInt = 0x00;
            dataIndex = 0;
            bitIndex = 0;
            mask = 0x01;
            j = 0;
            i = 0;
        } else RXstate = 0;
    } else if (RXstate == 2 || RXstate == 3) {
        if (bitIndex < MAXBITS && Timer2Counter < START_ONE){
            arrBitTimer[bitIndex++] = Timer2Counter;
            if (bitIndex == FOB_PACKET_BITS){
                numBitsReceived = bitIndex;
                RXstate = 4;
                TEST_OUT = 0;                
            }
        }
        else {
            if (RXstate == 3) {
                numBitsReceived = bitIndex;
                RXstate = 4;
                TEST_OUT = 0;
            } else RXstate = 0;
        }
        if (bitIndex == 16) {
            mask = 0x0001;
            dataInt = 0;
            for (i = 0; i < 16; i++) {
                if (arrBitTimer[i] > 0xC0) dataInt = dataInt | mask;
                mask = mask << 1;
            }
            if (dataInt != 0x54CC) RXstate = 0;
            else {
                RXstate++;
                TEST_OUT = 1;
            }
        }
    }
}
#endif

#ifdef MANCHESTER
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    static unsigned short Timer2Counter = 0;
    static unsigned short byteMask = 0x0001;
    static unsigned short dataInt = 0x00;
    static unsigned char oddFlag = FALSE;
    static unsigned short dataIndex = 0;

    unsigned short PORTin, RX_PIN;

    // Step #1 - always clear the mismatch condition first
    // PORTDin = PORTDbits.RD14;
    PORTin = PORTBbits.RB0;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    Timer2Counter = TMR2 / 100;
    TMR2 = 0x0000;

    if (!PORTin) RX_PIN = 0;
    else RX_PIN = 1;

    if (RXstate < 5) ConfigIntCN(CHANGE_INT_OFF | CHANGE_INT_PRI_2);

    if (RXstate && (Timer2Counter > TIMEOUT)) {
        if (RXstate == 6) {
            timeoutFlag = TRUE;
            numBytesReceived = dataIndex;
        }
        RXstate = 0;
    }// RX_IN goes HIGH when state machine is idle: first START
    else if (RX_PIN && !RXstate)
        RXstate = 1;

    // RX_IN goes LOW after long pulse: second START
    if (!RX_PIN && Timer2Counter > START_ONE && Timer2Counter < START_ONE * 2)
        RXstate = 2;

    // RX_IN goes HIGH: third START      
    if (RX_PIN && RXstate == 2) {
        if (Timer2Counter > START_TWO && Timer2Counter < START_TWO * 2)
            RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: fourth START                 
    } else if (RXstate == 3) {
        if (Timer2Counter > START_THREE && Timer2Counter < START_THREE * 2)
            RXstate++;
        else RXstate = 0;
        // RX_IN goes HIGH: dummy bit even pulse
    } else if (RXstate == 4) {
        if (Timer2Counter > START_FOUR && Timer2Counter < START_FOUR * 2) RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: dummy bit odd pulse                  
    } else if (RXstate == 5) {
        byteMask = 0x01;
        oddFlag = FALSE;
        dataInt = 0x00;
        error = 0;
        dataIndex = 0;
        TEST_OUT = 1;
        RXstate++;
        // RX STATE = 6:
        // DATA BITS get processed here. 
        // Clock state toggles between EVEN and ODD with each transition.
        // Data bit is always read on ODD half of clock cycle, 
        // indicated when oddFlag = TRUE.
        // All LONG pulses begin and end when clock is ODD,
        // so data bit is always read when long pulse is detected.
    } else if (RXstate == 6) {
        // If this a long pulse, data bit always gets read,
        // since all long pulses end on odd clock cycle:
        if (Timer2Counter > MAXBITLENGTH) {
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) {
                    numExpectedBytes = dataInt + 3;
                }
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = (unsigned char) dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
            oddFlag = FALSE;

            // Otherwise, this must be a short pulse,
            // in which case 
        } else if (oddFlag) {
            oddFlag = FALSE;
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) {
                    numExpectedBytes = dataInt + 3;
                }
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = (unsigned char) dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
        } else oddFlag = TRUE;

        if (dataIndex >= numExpectedBytes && numExpectedBytes != 0) {
            numBytesReceived = dataIndex;
            dataIndex = 0;
            RXstate = 0;
            TEST_OUT = 0;
        }
    } // End else
}
#endif
/** EOF main.c *************************************************/





/*********************************************************************************************
 * MAIN file for LINX MANCHESTER RX
 * For UBW32 PIC32MX795F512X boards
 * Compiler: XC32 V1.30
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
 * 12-10-16: Modified RX pin handler so TMR4 = 0x0000
 * 12-12-16: Check flags on incoming data from motion register 0x16
 * 09-08-16: recompiled. Works well with ManchesterTx Wand transmitting accelerometer data and two byte CRC
 *          Also Checked LINXFOB and copied to mainLinxFob.c
 *  7-15-20: Modified to work for 4800 baud Sparkfun transmitter & receiver.
 *           USES POLLING, not interrupts
 * 
 *  7-26-20: Works with Manchester Tx using Sparkfun transmitter and receiver.
 *          Cleaned up, removed accelerometer, added repeats and 100 ms delay. 
 **********************************************************************************************************/
#include <plib.h>
#include <string.h>
#include <ctype.h>

#define MANCHESTER

#define true TRUE
#define false FALSE

#define MAXBITLENGTH 5000 // 40
#define START_ONE 80
#define STOP 3000
#define START_TWO 80
#define START_THREE 40
#define START_FOUR 40
#define TIMEOUT 1000

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

#define MAXBUFFER 128
unsigned char HOSTTxBuffer[MAXBUFFER];
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned short HOSTTxLength;
unsigned short HOSTRxLength = 0;

unsigned char TimeoutFlag = false;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void ConfigAd(void);
unsigned char getCRC7(unsigned char *ptrMessage, short numBytes);
unsigned char reverseByte(unsigned char dataByte);

unsigned char RXstate = 0;

#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];


unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;
#define MAXBITS 256
long arrBitData[MAXBITS];


short numBitsReceived = 0;
short RxTimeout = 0;


    unsigned short trialCounter = 0;
    unsigned short dataInt;
    unsigned short CRCcheck;
    short rawVectx, rawVecty, rawVectz;
    unsigned char motionData = 0;    
    long Timer2Counter = 0;
    short numExpectedBytes = 0;    
    unsigned short byteMask = 0x0001;
    unsigned char oddFlag = FALSE;
    unsigned short dataIndex = 0;
    unsigned short PreviousPORTBread, PORTBin;
    short shortPulseCounter = 0;
    short state = 0, previousState = 0;
#define NUMBER_OF_BITS 16
    long bitData[NUMBER_OF_BITS];
    long FirstPulseCounts = 0;
    long SecondPulseCounts = 0;
    long ThirdPulseCounts = 0;
    short bitIndex = 0, i = 0;
    short timeoutCounter = 0;
    short OKcounter = 0;

int main(void) 
{
    
    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;    

    InitializeSystem();
    DelayMs(200);
    TEST_OUT = 0;
    
    printf("\r\r#1 Manchester RX with interrupts");    

    numBytesReceived = 0;
    
    for (i = 0; i < NUMBER_OF_BITS; i++) bitData[i] = 0;
    bitIndex = 0;

    while(1)
    {             
        /*
        if (previousState != state && state > 1) 
        {
            if (state == 0) printf("\rS%d", state);
            else printf(" S%d", state);
        }
        previousState = state;
        */
        
        if (TimeoutFlag)
        {
            TimeoutFlag = false;
            printf("\rTO");
        }
        if (numBytesReceived) 
        {
            state = 0;            
            CRCcheck = CRCcalculate(&arrData[1], numBytesReceived - 3);
            convert.byte[0] = arrData[numBytesReceived - 2];
            convert.byte[1] = arrData[numBytesReceived - 1];

            // printf("\r#%d: %d bytes received: ", ++trialCounter, numBytesReceived);
            numBytesReceived = 0;
            
            if (convert.integer != CRCcheck) 
                printf(" CRC ERROR: %X != %X, ", convert.integer, CRCcheck);
            else
            {        
                motionData = arrData[1];
                printf("\r#%d: %d, ", OKcounter++, motionData);
            
                convert.byte[0] = arrData[2];
                convert.byte[1] = arrData[3];
                rawVectx = (short) convert.integer;
            
                convert.byte[0] = arrData[4];
                convert.byte[1] = arrData[5];
                rawVectz = (short) convert.integer;
            
                convert.byte[0] = arrData[6];
                convert.byte[1] = arrData[7];
                rawVecty = (short) convert.integer;            
                                
                printf ("X: %d, Y: %d, Z: %d, ", rawVectx, rawVecty, rawVectz);
                if (0b00000010 & motionData){
                    if (0b00000001 & motionData) printf("-X, ");
                    else printf ("+X, ");
                } 
                if (0b00001000 & motionData){
                    if (0b00000100 & motionData) printf("-Y, ");
                    else printf ("+Y, ");
                }       
                if (0b00100000 & motionData){
                    if (0b00010000 & motionData) printf("-Z ");
                    else printf ("+Z ");
                }                                         
                
            }     
            
            // printf ("\r1st: %ld, 2nd: %ld, 3rd: %ld", FirstPulseCounts, SecondPulseCounts, ThirdPulseCounts);
            /*
            for (i = 0; i < NUMBER_OF_BITS; i++)
            {
                if ((i % 8)==0) printf("\r%d ", bitData[i]);
                else printf("%ld ", bitData[i]);
                bitData[i] = 0;
            }
            */
            
        }
        
    }

}//end main



static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_5);
    mCNOpen(CN_ON, CN2_ENABLE, CN2_PULLUP_ENABLE);
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
    T2CONbits.T32 = 1; // Timers 2 & 3 are 32 bit timers
    PR2 = 0xFFFF;
    PR3 = 0xFFFF;
    T2CONbits.TON = 1; // Let her rip      

    // Set up Timer 4 for 100 Hz interrupts
    T4CON = 0x00;
    T4CONbits.TCKPS2 = 1; // 1:16 Prescaler    
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 50000;
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_5);
    T4CONbits.TON = 1; // Let her rip   

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define HOSTuart UART2
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 115200);
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





void __ISR(_TIMER_4_VECTOR, ipl5) Timer4Handler(void) {
    mT4ClearIntFlag(); // clear the interrupt flag 
    
    if (RxTimeout) 
    {        
        if (RxTimeout)
        {
            RxTimeout--;
            if (!RxTimeout) TimeoutFlag = true;
        }
    }
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{
    // Step #1 - always clear the mismatch condition first
    PORTBin = PORTRead(IOPORT_B) & 0x0001;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

        if (PreviousPORTBread != PORTBin)
        {
            PreviousPORTBread = PORTBin;
            Timer2Counter = (long) TMR2;  
            TMR2 = 0x0000;
                        
            if (TimeoutFlag) 
            {
                TimeoutFlag = false;
                state = 0;                    
                shortPulseCounter = 0;
                printf("\rTIMEOUT: %d", timeoutCounter++);
            }          
            
            if (state && Timer2Counter > 70000)
            {
                RXstate = 0;                
                // shortPulseCounter = 0;
            }     
            else if (!state)
            {
                //if (Timer2Counter > 4500 && Timer2Counter < 7500)
                //    shortPulseCounter++;
                //else shortPulseCounter = 0;
                //bitData[bitIndex++] = Timer2Counter;
                //if (bitIndex >= NUMBER_OF_BITS) bitIndex = 0;
                //if (shortPulseCounter > 4) 
                //{
                //    state = 1;
                //    shortPulseCounter = 0;
                RxTimeout = 100;   
                //}
                if (PORTBin) state = 1;
                
            }
            else if (state == 1)
            {
                if (Timer2Counter > 62000 && Timer2Counter < 63000)  
                {
                    state = 2;      
                    FirstPulseCounts = Timer2Counter;
                }    
                else state = 0;
            }
            else if (state == 2)
            {
                if (Timer2Counter > 10000 && Timer2Counter < 20000) 
                {
                    state = 3;
                    SecondPulseCounts = Timer2Counter;
                }
                else
                {
                    state = 0;
                    shortPulseCounter = 0;
                }
            }
            else if (state == 3)
            {
                if (Timer2Counter > 8000 && Timer2Counter < 10000)
                {
                    state = 4;
                    ThirdPulseCounts = Timer2Counter;
                    oddFlag = FALSE;
                    dataInt = 0x00;
                    dataIndex = 0;                        
                    byteMask = 0x01;
                    numExpectedBytes = 0;
                    shortPulseCounter = 0;
                    bitIndex = 0;                                                                
                }
            }
            else if (state == 4 || state == 5 || state == 6)
                state++;
                
            if (state == 7)
            {                    
                // If this a long pulse, data bit always gets read,
                // since all long pulses end on odd clock cycle:
                if (Timer2Counter > MAXBITLENGTH) 
                {
                    if (PORTBin) dataInt = dataInt | byteMask;
                    if (byteMask == 0x80) 
                    {
                        byteMask = 0x01;
                        if (dataIndex == 0) 
                            numExpectedBytes = dataInt + 3;
                        if (dataIndex < MAXDATABYTES) 
                            arrData[dataIndex++] = (unsigned char) dataInt;
                            dataInt = 0x00;
                    } 
                    else byteMask = byteMask << 1;
                    oddFlag = FALSE;
                } 
                // Otherwise, this must be a short pulse,
                // in which case data bits are read on odd pulses,
                // and even pulses are ignored:
                else if (oddFlag) 
                {
                    oddFlag = FALSE;
                    if (PORTBin) dataInt = dataInt | byteMask;
                    if (byteMask == 0x80) 
                    {
                        byteMask = 0x01;
                        if (dataIndex == 0) 
                            numExpectedBytes = dataInt + 3;
                        if (dataIndex < MAXDATABYTES) 
                            arrData[dataIndex++] = (unsigned char) dataInt;                        
                        dataInt = 0x00;
                    } else byteMask = byteMask << 1;
                } 
                else oddFlag = TRUE;                   
                if (numExpectedBytes && dataIndex >= numExpectedBytes) // TODO: Make this more robust?                    
                {
                    numBytesReceived = dataIndex;
                    state++;
                }
            }
        }
    
}


/** EOF main.c *************************************************/







/* 
 * File:   Defs.h
 * Author: Jim
 *
 * Modified 3-25-16 for Rev 2 LED controller
   Modified 4-30-16 for Rev 3 LED controller
 * For DUAL MATRIX option and 32x32 panels
 *  6-7-16: Basic video works great with three serial ports.
 * 
 */

#ifndef DEFS_H
#define	DEFS_H

#define NUMCHANNELS 1 // $$$$
// #define REV2BOARD 
#define REV3BOARD

#define PANELS_ACROSS 1
#define PANELS_STACKED 2
#define NUMPANELS (PANELS_ACROSS * PANELS_STACKED)
#define PANEL32X32 
#define DUALMATRIX


#define USB_STANDBY 0
#define USB_INCOMING 1
#define USB_DONE 2
#define USB_PACKET_OK 3
#define USB_CRC_ERROR 4
#define USB_LENGTH_ERROR 5
#define USB_BITMAP_ERROR 6
#define USB_TIMEOUT_ERROR 7
#define USB_PACKETERROR 8
#define USB_COMMAND_ERROR 9

#define Delayms DelayMs
#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define MAXBITMAP 10000


#ifdef PANEL32X32
#define PANELROWS 32
#define COLORDEPTH 3 // was 5
#define MAXLINE 16   // Equal to number of rows / 2
#define TIMER_ROLLOVER 2000
#define MAXCOL (PANELS_ACROSS*32)
#define MAXROW (PANELS_STACKED*32)  
#else
#define PANELROWS 16
#define COLORDEPTH 3 // was 6
#define MAXLINE 8
#define TIMER_ROLLOVER 2000
#define MAXCOL (PANELS_ACROSS*32)
#define MAXROW (PANELS_STACKED*16)
#endif
 
// NUMWRITES is the number of words written to the output port 
// per each color plane for each line on the matrix
// NUMWRITES = Total columns * total rows / number of rows per panel
// The number of rows per panel = LINES * 2, for either 16x32 or 32x32 panels:
#ifdef DUALMATRIX
#define NUMWRITES (MAXCOL * MAXROW / (MAXLINE * 4))        
#else
#define NUMWRITES (MAXCOL * MAXROW / (MAXLINE * 2))        
#endif


#define PANELCOLS 32
#define PANELSIZE (PANELROWS*PANELCOLS*NUMCHANNELS)
#define COMPRESSED_SIZE (PANELROWS*PANELCOLS*2)
#define BALANCE 48840
#define NUMPOTS 4

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR
#define HOST_VECTOR _UART_2_VECTOR

#define RS485uart UART5

#define XBEE_SLEEP PORTBbits.RB15

#define XBEE_SLEEP_ON PORTSetBits(IOPORT_B, BIT_0)
#define XBEE_SLEEP_OFF PORTClearBits(IOPORT_B, BIT_0)


#define START 1

//#define MAXBUFFER 64 // 512
//#define MAXHOSTBUFFER 128

#define XBEE_SLEEP PORTBbits.RB15

#define RED_LSB (0x01<<(8-COLORDEPTH))
#define GREEN_LSB (RED_LSB*256)
#define BLUE_LSB  (RED_LSB*65536)

#define OEbit 0x0020
#define LATbit 0x0040

#ifdef REV3BOARD
#define OE_HIGH_LATCH_LOW 0x2000
#define OE_LOW_LATCH_HIGH 0x6000
#endif

#ifdef REV2BOARD
#define OE_HIGH_LATCH_LOW 0x80  // OE=1, LATCH=0
#define OE_LOW_LATCH_HIGH 0x40  // OE=0, LATCH=1
#endif

//#define OE_HIGH_LATCH_LOW 0xC0  // 0x40
//#define OE_LOW_LATCH_HIGH 0x80  
//#endif

// PIN DEFINITIONS FOR REV 2 LED MATRIX CONTROLLER


#ifdef REV3BOARD
#define R1bit 0x0100
#define B1bit 0x0001
#define G1bit 0x0200
#define R2bit 0x0002
#define B2bit 0x0800
#define G2bit 0x0400
#define M2R1bit 0x0080
#define M2B1bit 0x0040
#define M2G1bit 0x0020
#define M2R2bit 0x0004
#define M2B2bit 0x0008
#define M2G2bit 0x0010
#endif

#ifdef REV2BOARD
#define R1bit 0x0001
#define B1bit 0x0002
#define G1bit 0x0010
#define R2bit 0x0004
#define B2bit 0x0008
#define G2bit 0x0020

#define M2R1bit 0x0040
#define M2B1bit 0x0080
#define M2G1bit 0x0400
#define M2R2bit 0x0100
#define M2B2bit 0x0200
#define M2G2bit 0x0800
#endif

/*
#define R1bit 0x0001
#define B1bit 0x0002
#define G1bit 0x0004
#define R2bit 0x0008
#define B2bit 0x0010
#define G2bit 0x0020

#define M2R1bit 0x0100
#define M2B1bit 0x0200
#define M2G1bit 0x0400
#define M2R2bit 0x0800
#define M2B2bit 0x1000
#define M2G2bit 0x2000
*/

#define EVEN_MASK ~(R1bit | B1bit | G1bit | R2bit | B2bit | G2bit)
#define ODD_MASK  ~(M2R1bit | M2B1bit | M2G1bit | M2R2bit | M2B2bit | M2G2bit)


#define CLKOUT  0x0010  // PORTDbits.RD4
// #define OE_ENB  0xFFDF  // PORTDbits.RD5
// #define LATCH   0x0040  // PORTDbits.RD6




//#define DARKEN //& 0x7F7F7F
#define MAGENTA 0x8000A0
#define PURPLE 0xB00050
#define CYAN 0x405000
#define LIME 0x00A060
#define YELLOW 0x007090
#define ORANGE 0x0020FF
#define RED 0x0000FF
#define GREEN 0x00FF00
#define BLUE 0xFF0000
#define PINK 0x1020F0
#define LAVENDER 0x400030
#define TURQUOISE 0x30A000
#define WHITE 0x707070
// #define WHITE 0xFFFFFF
#define GRAY 0x303030
#define DARKGRAY 0x101010

#define BLACK 0

#define MAXCOLOR 15


#define MAXRANDOM (RAND_MAX+1)
#define true	TRUE
#define false 	FALSE

#define RS485_CTRL PORTGbits.RG0

// Pin defs for UBW32:
#define CLKOUT  0x0010  // PORTDbits.RD4
// #define OE_ENB  0xFFDF  // PORTDbits.RD5
//#define LATCH   0x0040  // PORTDbits.RD6

#ifdef REV3BOARD
#define OEpin   LATCbits.LATC13
#define LATCH   LATCbits.LATC14
#else
#define OEpin   PORTDbits.RD5
#endif

#define RIGHT 1
#define LEFT  2
#define UP    3
#define DOWN  4
#define UPRIGHT 5
#define UPLEFT 6
#define DOWNRIGHT 7
#define DOWNLEFT 8

#define USE_SD_INTERFACE_WITH_SPI
        #define MDD_USE_SPI_2

		//SPI Configuration
		#define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
        #define SPI_START_CFG_2     (SPI_ENABLE)

        // Define the SPI frequency
        #define SPI_FREQUENCY			(20000000)

            // Description: SD-SPI Chip Select Output bit
            #define SD_CS               LATCbits.LATC4 // LATBbits.LATB9
            // Description: SD-SPI Chip Select TRIS bit
            #define SD_CS_TRIS          TRISCbits.TRISC4 // TRISBbits.TRISB9

            // Description: SD-SPI Card Detect Input bit
            #define SD_CD               PORTGbits.RG9 // PORTGbits.RG0
            // Description: SD-SPI Card Detect TRIS bit
            #define SD_CD_TRIS          TRISGbits.TRISG9 // TRISGbits.TRISG0

            // Description: SD-SPI Write Protect Check Input bit
            #define SD_WE               PORTAbits.RA0 // PORTGbits.RG1
            // Description: SD-SPI Write Protect Check TRIS bit
            #define SD_WE_TRIS          TRISAbits.TRISA0 // TRISGbits.TRISG1

            // Description: The main SPI control register
            #define SPICON1             SPI2CON
            // Description: The SPI status register
            #define SPISTAT             SPI2STAT
            // Description: The SPI Buffer
            #define SPIBUF              SPI2BUF
            // Description: The receive buffer full bit in the SPI status register
            #define SPISTAT_RBF         SPI2STATbits.SPIRBF
            // Description: The bitwise define for the SPI control register (i.e. _____bits)
            #define SPICON1bits         SPI2CONbits
            // Description: The bitwise define for the SPI status register (i.e. _____bits)
            #define SPISTATbits         SPI2STATbits
            // Description: The enable bit for the SPI module
            #define SPIENABLE           SPI2CONbits.ON
            // Description: The definition for the SPI baud rate generator register (PIC32)
            #define SPIBRG			    SPI2BRG

            // Tris pins for SCK/SDI/SDO lines

            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISGbits.TRISG6
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISGbits.TRISG7
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISGbits.TRISG8
            //SPI library functions
            #define putcSPI             putcSPI2
            #define getcSPI             getcSPI2
            #define OpenSPI(config1, config2)   OpenSPI2(config1, config2)
        
        // Will generate an error if the clock speed is too low to interface to the card
        //#if (GetSystemClock() < 100000)
        //    #error Clock speed must exceed 100 kHz
        //#endif

#endif


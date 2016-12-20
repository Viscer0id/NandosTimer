/*
*********************************************************************************************************
* Module     : MAX7219 Interaction Code
* Author     :
* Description: http://www.adnbr.co.uk/articles/max7219-and-7-segment-displays
*********************************************************************************************************
*/


/*
*********************************************************************************************************
* Include Header Files
*********************************************************************************************************
*/
#include "max7219.h"

/*
*********************************************************************************************************
* Macros
*********************************************************************************************************
*/
#define MAX7219_LOAD1 	SPI_PORT |= (1<<PIN_SS)
#define MAX7219_LOAD0 	SPI_PORT &= ~(1<<PIN_SS)

/*
*********************************************************************************************************
* Constants
*********************************************************************************************************
*/
#define ON 1
#define OFF 0
#define MAX7219_MODE_DECODE 0x09
#define MAX7219_MODE_INTENSITY 0x0A
#define MAX7219_MODE_SCAN_LIMIT 0x0B
#define MAX7219_MODE_POWER 0x0C
#define MAX7219_MODE_TEST 0x0F
#define MAX7219_MODE_NOOP 0x00

#define MAX7219_DIGIT0 0x01
#define MAX7219_DIGIT1 0x02
#define MAX7219_DIGIT2 0x03
#define MAX7219_DIGIT3 0x04
#define MAX7219_DIGIT4 0x05
#define MAX7219_DIGIT5 0x06
#define MAX7219_DIGIT6 0x07
#define MAX7219_DIGIT7 0x08

#define MAX7219_CHAR_BLANK 0xF
#define MAX7219_CHAR_NEGATIVE 0xA

char digitsInUse = 8;

/*
*********************************************************************************************************
* Function Prototypes
*********************************************************************************************************
*/
void spiSendByte (char databyte);
void MAX7219_writeData(char data_register, char data);
void MAX7219_clearDisplay();
void MAX7219_displayNumber(volatile long number);
void MAX7219_Init();

// ..................................... Functions ..............................................

void spiSendByte (char databyte)
{
    // Copy data into the SPI data register
	SPI_DATA_REG = databyte;
    // Wait until transfer is complete
    while (!(SPI_STATUS_REG & (1 << SPI_INT_FLAG)));
}

void MAX7219_writeData(char data_register, char data)
{
    MAX7219_LOAD0;
        // Send the register where the data will be stored
        spiSendByte(data_register);

        // Send the data to be stored
        spiSendByte(data);
        //spiSendByte(MAX7219_LookupCode(data));
    MAX7219_LOAD1;
}

void MAX7219_clearDisplay()
{
    char i = digitsInUse;
    // Loop until 0, but don't run for zero
    do {
        // Set each display in use to blank
        MAX7219_writeData(i, MAX7219_CHAR_BLANK);
    } while (--i);
}

void MAX7219_displayNumber(volatile long number)
{
    char negative = 0;

    // Convert negative to positive.
    // Keep a record that it was negative so we can
    // sign it again on the display.
    if (number < 0) {
        negative = 1;
        number *= -1;
    }

    MAX7219_clearDisplay();

    // If number = 0, only show one zero then exit
    if (number == 0) {
        MAX7219_writeData(MAX7219_DIGIT0, 0);
        return;
    }

    // Initialisation to 0 required in this case,
    // does not work without it. Not sure why.
    char i = 0;

    // Loop until number is 0.
    do {
        MAX7219_writeData(++i, number % 10);
        // Actually divide by 10 now.
        number /= 10;
    } while (number);

    // Bear in mind that if you only have three digits, and
    // try to display something like "-256" all that will display
    // will be "256" because it needs an extra fourth digit to
    // display the sign.
    if (negative) {
        MAX7219_writeData(i, MAX7219_CHAR_NEGATIVE);
    }
}

void MAX7219_Init()
	{
	// Decode mode to "Font Code-B"
	MAX7219_writeData(MAX7219_MODE_DECODE, 0xFF);

	// Scan limit runs from 0.
	MAX7219_writeData(MAX7219_MODE_SCAN_LIMIT, digitsInUse - 1);
	MAX7219_writeData(MAX7219_MODE_INTENSITY, 8);
	MAX7219_writeData(MAX7219_MODE_POWER, ON);
}

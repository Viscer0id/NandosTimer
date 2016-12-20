/*
*********************************************************************************************************
* Module     : Nandos Timer
* Author     : Jeremy Dennis
* Description: 7 Day countdown clock indicating Nandos Thursday
*********************************************************************************************************
*/


/*
*********************************************************************************************************
* Include Header Files
*********************************************************************************************************
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include "max7219.h"



/*
*********************************************************************************************************
* Macros
*********************************************************************************************************
*/
#define F_CPU 	1000000UL

/**** Configure IO ****/
#define configure_as_input(bit)				{bit##_DDR &= ~(1<<bit);}
#define configure_as_output(bit)			{bit##_DDR |= (1<<bit);}

#define pullup_on(bit)						{bit##_PORT |= (1<<bit);}
#define pullup_off(bit)						{bit##_PORT &= ~(1<<bit);}

/**** Manipulate Outputs ****/
#define	 set_high(bit)						{bit##_PORT |= (1<<bit);}
#define set_low(bit)						{bit##_PORT &= ~(1<<bit);}
#define toggle(bit)							{bit##_PIN |= (1<<bit);}

/**** Test Inputs ****/
#define is_high(bit)						(bit##_PIN & (1<<bit))
#define is_low(bit)							(!(bit##_PIN & (1<<bit)))

/**** IO ****/
#define PIN_SCK 		PORTB5
#define PIN_MOSI 		PORTB3
#define PIN_SS 			PORTB2

#define BUTTON1			PD6
#define BUTTON1_PORT	PORTD
#define BUTTON1_DDR		DDRD
#define BUTTON1_PIN		PIND

#define BUTTON2			PD7
#define BUTTON2_PORT	PORTD
#define BUTTON2_DDR		DDRD
#define BUTTON2_PIN		PIND


/*
*********************************************************************************************************
* Constants
*********************************************************************************************************
*/
const uint32_t SECONDS_MAX = 604800;


/*
*********************************************************************************************************
* Global Variables
*********************************************************************************************************
*/
volatile uint8_t current_pos;
volatile uint32_t seconds_to_nt;
volatile uint8_t time_to_nt[8];


/*
*********************************************************************************************************
* Function Prototypes
*********************************************************************************************************
*/
void timer_2_setup (void);
void timer_0_setup (void);
void SPI_setup (void);
void verify_button_is_pressed (void);
void MAX7219_writeTime();
void decrement_time (uint8_t current_pos);


void timer_2_setup (void) {

	ASSR = 0b00001000; 	// Enable Timer2 asynchronous mode with an external crystal across TOSC1 and TOSC2
	_delay_ms(250); 	// Let external 32KHz crystal stabilise

	TCNT2 = 0b00000000;	// Clear the register that controls Timer2. This will remove the compare match on the
						// following timer clock

	TCCR2 = 0b00000101; // Set the prescaler (TCCR2 register) to 128: 32.768kHz / 128 = 1Hz overflow

	TIMSK &= ~(1 << OCIE2); // Disable output compare match interrupt on Timer2
	TIMSK |= (1 << TOIE2); // Enable overflow interrupt on Timer2
}

void timer_0_setup (void) {
	TIMSK |= (1 << TOIE0);	// Enable overflow interrupt on Timer0
}

void SPI_setup (void) {
	// Configure the pins used for SPI as output pins
    DDRB |= (1 << PIN_SCK) | (1 << PIN_MOSI) | (1 << PIN_SS);

    // Enable SPI, Master mode
    SPCR |= (1 << SPE) | (1 << MSTR)| (1<<SPR1);
}


void verify_button_is_pressed (void){
	TCNT0 += 0; // Add 0 to Timer0 register, so it has [less/more] clock cycles to count before overflow
	TCCR0 |= (1 << CS02) | (1 << CS00); // Enable Timer0 with clk/1024 prescaling
}


void MAX7219_writeTime(void) {
	MAX7219_writeData(8,0xF);
	MAX7219_writeData(7,time_to_nt[6]);
	MAX7219_writeData(6,time_to_nt[5]);
	MAX7219_writeData(5,time_to_nt[4]);
	MAX7219_writeData(4,time_to_nt[3]);
	MAX7219_writeData(3,time_to_nt[2]);
	MAX7219_writeData(2,time_to_nt[1]);
	MAX7219_writeData(1,time_to_nt[0]);

	if (current_pos != 8) {
		MAX7219_writeData(current_pos,time_to_nt[current_pos-1]+128);
	}
}

void decrement_time (uint8_t current_pos){
	switch(current_pos){
	case 8:
		break;
	case 7:
		seconds_to_nt = seconds_to_nt-86400;
		break;
	case 6:
		seconds_to_nt = seconds_to_nt-36000;
		break;
	case 5:
		seconds_to_nt = seconds_to_nt-3600;
		break;
	case 4:
		seconds_to_nt = seconds_to_nt-600;
		break;
	case 3:
		seconds_to_nt = seconds_to_nt-60;
		break;
	case 2:
		seconds_to_nt = seconds_to_nt-10;
		break;
	case 1:
		seconds_to_nt--;
		break;
	}

	/*
	days = time_in_seconds/86400;
	hours = time_in_seconds%86400/3600;
	mins = time_in_seconds%86400%3600/60;
	secs = time_in_seconds%86400%3600%60;

	days_10 	= days%10;
	days_1 		= days/10;
	hours_10 	= hours%10;
	hours_1 	= hours/10;
	mins_10 	= mins%10;
	mins_1 		= mins/10;
	secs_10 	= secs%10;
	secs_1 		= secs/10;
	*/
	time_to_nt[7] = 0xF;
	time_to_nt[6] = seconds_to_nt/86400%10;
	time_to_nt[5] = seconds_to_nt%86400/3600/10;
	time_to_nt[4] = seconds_to_nt%86400/3600%10;
	time_to_nt[3] = seconds_to_nt%86400%3600/60/10;
	time_to_nt[2] = seconds_to_nt%86400%3600/60%10;
	time_to_nt[1] = seconds_to_nt%86400%3600%60/10;
	time_to_nt[0] = seconds_to_nt%86400%3600%60%10;
}

ISR(TIMER2_OVF_vect) {
	if (seconds_to_nt == 0) {
		seconds_to_nt = SECONDS_MAX;	// Reset
	} else {
		decrement_time(1);
	}
	MAX7219_writeTime();
}

ISR(TIMER0_OVF_vect) {
	TCCR0 = 0;	// Disable the timer

	if (is_high(BUTTON1)){
		decrement_time(current_pos);
		MAX7219_writeData(current_pos,time_to_nt[current_pos-1]+128); //Write the new digit
	}

	if (is_high(BUTTON2)){
		MAX7219_writeData(current_pos,time_to_nt[current_pos-1]); // Clear the previous digit of the decimal
		if (current_pos == 1) {
			current_pos = 8;
		} else {
			current_pos--;
		}
		MAX7219_writeData(current_pos,time_to_nt[current_pos-1]+128); //Add the decimal to the current digit
	}
}

int main(void)
{
	seconds_to_nt = SECONDS_MAX; 	// Set the maximum amount of time
	timer_2_setup(); 	// Configure the internal timer
	timer_0_setup(); 	// Configure Timer0
	SPI_setup(); 		// Enable SPI
	sei(); 				// Global enable interrupts

	current_pos = 8;

    // Configure the buttons as inputs
    configure_as_input(BUTTON1);
    configure_as_input(BUTTON2);

    // Initialise the MAX7219
    MAX7219_Init();
    MAX7219_clearDisplay();

    while(1)
    {
    	if (is_high(BUTTON1) || is_high(BUTTON2)) {
    		// We have detected a button state change
    		verify_button_is_pressed();	// Verify using timer debounce routine
    	}
    }
}

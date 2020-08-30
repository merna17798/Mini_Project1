#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"
/* global variable contain the 7-seg count value */
unsigned char Seconds_counter = 0;
unsigned char Minutes_counter = 0;
unsigned char Hours_counter = 0;
/* External INT0 Interrupt Service Routine */
ISR(INT0_vect)
{
	Seconds_counter = 0;
	Minutes_counter = 0;
	Hours_counter = 0;
}

/* External INT0 enable and configuration function */
void INT0_Init(void)
{
	SREG  &= ~(1<<7);                   // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD2));               // Configure INT0/PD2 as input pin
	PORTD |= (1<<PD2);     // Enable the internal pull up resistor at PD2 pin
	GICR  |= (1<<INT0);                 // Enable external interrupt pin INT0
	MCUCR |= (1<<ISC01);   // Trigger INT0 with the falling edge
	SREG  |= (1<<7);                    // Enable interrupts by setting I-bit
}

/* External INT1 enable and configuration function */
void INT1_Init(void)
{
	SREG  &= ~(1<<7);      // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD3));  // Configure INT1/PD3 as input pin
	GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
	// Trigger INT1 with the rising edge
	MCUCR |= (1<<ISC11)|(1<<ISC10);
	SREG  |= (1<<7);       // Enable interrupts by setting I-bit
}
/* External INT1 Interrupt Service Routine (pause)*/
ISR(INT1_vect)
{
	TCCR1B &= ~(1<<CS12)&~(1<<CS11)&~(1<<CS10);
}
ISR(INT2_vect)
{
	TCCR1B |= (1<<CS12)|(1<<CS10);
	TCCR1B &=~(1<<CS11);
}

/* External INT2 enable and configuration function */
void INT2_Init(void)
{
	SREG   &= ~(1<<7);       // Disable interrupts by clearing I-bit
	DDRB   &= (~(1<<PB2));   // Configure INT2/PB2 as input pin
	PORTB |= (1<<PB2);     // Enable the internal pull up resistor at PB2 pin
	GICR   |= (1<<INT2);	 // Enable external interrupt pin INT2
	MCUCSR &= ~ (1<<ISC2);     // Trigger INT2 with the falling edge
	SREG   |= (1<<7);        // Enable interrupts by setting I-bit
}

void Timer1_CTC_INIT(void){
    // initialise counter
    TCNT1 = 0;
    // initialise compare value
    OCR1A = 1000;
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
    // set timer to compare mode with prescaler 1024
	TCCR1A = (1<<COM1A1)|(1<<COM1A0)|(1<<FOC1A);
	TCCR1B = (1<<WGM12) | (1<<CS12)| (1<<CS10);
	SREG  |= (1<<7);   // Enable global interrupts in MC.
}
ISR (TIMER1_COMPA_vect)
{
	PORTA |= (1<<3);
	if(Seconds_counter == 9)
	{
		Seconds_counter= 0;
		PORTC = (PORTC & 0xF0) | (Seconds_counter & 0x0F);
		Minutes_counter++;
	}
	else
	{
		Seconds_counter++;
		PORTC = (PORTC & 0xF0) | (Seconds_counter & 0x0F);
	}

	PORTA &= (~(1<<3));
	PORTA |= (1<<2);
	if(Minutes_counter == 3)
	{
		Seconds_counter =0;
		Minutes_counter= 0;
		PORTC = (PORTC & 0xF0) | (Minutes_counter & 0x0F);
		Hours_counter++;

	}
	else
	{
		PORTC = (PORTC & 0xF0) | (Minutes_counter & 0x0F);
	}

	PORTA &= (~(1<<2));
	PORTA |= (1<<1);
	if(Hours_counter == 2)
	{
		Seconds_counter= 0;
		Minutes_counter= 0;
		Hours_counter =0;
		PORTC = (PORTC & 0xF0) | (Hours_counter & 0x0F);
	}
	else
	{
		PORTC = (PORTC & 0xF0) | (Hours_counter & 0x0F);
	}
	PORTA &= (~(1<<1));

}

int main(void)
{
	DDRC  |= 0x0F;     // Configure the first four pins in PORTC as output pins.
	PORTC &= 0xF0;     // Initialize the 7-seg display zero at the beginning.
	DDRA |= (1<<1) | (1<<2) | (1<<3);
	PORTA &= ~(1<<1) & ~ (1<<2) & ~ (1<<3);
	//SREG  |= (1<<7);   // Enable global interrupts in MC.
	INT2_Init();           // Enable and configure external INT2
	INT1_Init();           // Enable and configure external INT1
	INT0_Init();           // Enable and configure external INT0
	Timer1_CTC_INIT();
    while(1)
    {
    	unsigned char list[3] ={Seconds_counter,Minutes_counter,Hours_counter};
    	unsigned char  i=1;
    	for(i=1;i<=3;i++){
    		PORTA |= (1<<i);
    		PORTC = (PORTC & 0xF0) | (list[i-1] & 0x0F);
    		_delay_ms(5);
    		PORTA &= (~(1<<i));
    	}

    }
}

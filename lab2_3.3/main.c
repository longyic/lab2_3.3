/*
 * lab2_3.3.c
 *
 * Created: 2018/9/19 20:02:27
 * Author : Lenovo
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "uart.h"
volatile unsigned int edge1, edge2, pulse, memory;
volatile int overflows, rising, counter;
float freq;
void length();
void frequency_continuous();
void frequency_discrete();

int main(void)
{
	uart_init();
	
	DDRC = 0x00;
	DDRB |= (1 << PORTB1) | (1 << PORTB2) | (1 <<PORTB3) | (1 << PORTB4);
	DDRD |= (1 << PORTD6);
	PORTB |= (1 << PORTB1) | (1 << PORTB2) | (1 <<PORTB3) | (1 << PORTB4);
	
	TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);      //edge interrupt and overflow interrupt
	TCCR1B |= (1 << CS10) | (1 << ICES1);      //no prescale and input capture
	
	TCCR0A |= (1 << COM0A0) | (1 << WGM01);    //toggle OC0A and CTC
	TCCR0B |= (1 << CS02);                    //prescale 64
	
	TCNT0 = 0;                         //initial time counter0
	TCNT1 = 0;                         //initial time counter1
	
	edge1 = 0;
	edge2 = 0;
	rising = 0;
	overflows = 0;
	counter = 0;
	memory = 0;

	ADMUX |= (1 << REFS0);                                  //reference 5V
	ADCSRA |= (1 << ADEN) | (1 << ADATE);                   //enable ADC
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  //factor 128
	ADCSRA |= (1 << ADSC);                                  //Start conversion
	ADCSRB &= ~((1 << ADTS0)|(1 << ADTS1)|(1 << ADTS2));    //free-running mode

	sei();
	while (1)
	{
		if (TCNT1 >= 79)
		{
			PORTB ^= (1 << PORTB1);     //change from output to input
			TCNT1 = 0;                  //clear counter
		}
		
		if(ADC > 0 && ADC <= 128)
		{
			PORTB &= ~((1 << PORTB2) | (1 <<PORTB3) | (1 << PORTB4));
		}
		if(ADC > 128 && ADC <= 256)
		{
			PORTB &= ~((1 <<PORTB3) | (1 << PORTB4));
			PORTB |= (1 << PORTB2);
		}
		if(ADC > 256 && ADC <= 384)
		{
			PORTB &= ~((1 <<PORTB2) | (1 << PORTB4));
			PORTB |= (1 << PORTB3);
		}
		if(ADC > 384 && ADC <= 512)
		{
			PORTB &= ~((1 << PORTB4));
			PORTB |= (1 <<PORTB2) | (1 << PORTB3);
		}
		if(ADC > 512 && ADC <= 640)
		{
			PORTB &= ~((1 <<PORTB3) | (1 << PORTB2));
			PORTB |= (1 << PORTB4);
		}
		if(ADC > 640 && ADC <= 768)
		{
			PORTB &= ~((1 << PORTB3));
			PORTB |= (1 <<PORTB2) | (1 << PORTB4);
		}
		if(ADC > 768 && ADC <= 896)
		{
			PORTB &= ~((1 << PORTB2));
			PORTB |= (1 <<PORTB3) | (1 << PORTB4);
		}
		if(ADC > 896 && ADC <= 1024)
		{
			PORTB |= (1 << PORTB2) | (1 <<PORTB3) | (1 << PORTB4);
		}

		length();                                     //calculate pulse width
		TIMSK0 |= (1 << OCIE0A);                      //enable compare match interrupt
		
		if(!(PINB & 0x01))                            //push button to enter discrete freq mode
		{
			frequency_discrete();
		}
		else                                          //release button to enter continuous freq mode
		{
			frequency_continuous();
		}

	}
}

void length()
{
	if(edge2 < edge1)
	{
		overflows--;
	}
	pulse = overflows * 65536 + (edge2 - edge1);        //calculate pulse width
}


ISR(TIMER1_OVF_vect)
{
	overflows++;
}

ISR(TIMER1_CAPT_vect)
{
	if(!rising){
		edge1 = ICR1;
		rising = 1;
		TCCR1B &= ~(1 << ICES1);          //input capture negative edge
		TIFR1 |= (1 << ICR1);             //clear capture flag
	}
	else{
		edge2 = ICR1;
		rising = 0;
		TIFR1 |= (1 << ICR1);             //clear capture flag
		TCCR1B |= (1 << ICES1);           //input capture positive edge
	}
}


ISR(TIMER0_COMPA_vect)
{
	memory = pulse;
	if(memory < 500)
	{
		memory = 500;
	}
	if(memory > 5000)
	{
		memory = 5000;
	}
	counter = 14 + (memory/5000.0) * 16;           //calculate OCR0A register value
	PORTD ^= (1 << PORTD6);                        //output PD6
	TIMSK0 &= ~(1 << OCIE0A);                      //disable match interrupt
}

void frequency_continuous(void)
{
	OCR0A = counter;                               //set up OCR0A value
	freq = 2093 - (memory/5000.0f) * 1046.5;       //calculate freq
	printf("Frequency is %.2f Hz\n", freq);
}

void frequency_discrete(void)
{
	float range = memory/5000.0f;                  //calculate freq range
	if(range > 0 && range <= 0.2125){
		freq = 2093;
		OCR0A = 14;
	}
	if(range > 0.2125 && range <= 0.325){
		freq = 1975.53;
		OCR0A = 15;
	}
	if(range > 0.325 && range <= 0.4375){
		freq = 1760;
		OCR0A = 17;
	}
	if(range > 0.4375 && range <= 0.55){
		freq = 1567.98;
		OCR0A = 19;
	}
	if(range > 0.55 && range <= 0.6625){
		freq = 1396.91;
		OCR0A = 22;
	}
	if(range > 0.6625 && range <= 0.775){
		freq = 1318.51;
		OCR0A = 23;
	}
	if(range > 0.775 && range <= 0.8875){
		freq = 1174.66;
		OCR0A = 26;
	}
	if(range > 0.8875 && range <= 1){
		freq = 1046.5;
		OCR0A = 28;
	}
	printf("Frequency is %.2f Hz\n", freq);
}





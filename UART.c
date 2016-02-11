#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/io.h>
#include <stdio.h>
#include <util/setbaud.h>

#include "UART.h"
void 
UART0Init()
{
/*Set baud rate*/
UBRR0H = UBRRH_VALUE;
UBRR0L = UBRRL_VALUE;
/*Set frame format to 8 data bits, no parity, 1 stop bit*/
UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
/*enable transmission and reception*/
UCSR0B |= (1<<RXEN0)|(1<<TXEN0);

}

int 
UART0SendByte(char u8Data, FILE *stream)
{
    if(u8Data == '\n')
      UART0SendByte('\r', 0);
	/*wait while previous byte is completed*/
	while(!(UCSR0A&(1<<UDRE0))){};
	/*Transmit data*/
	UDR0 = u8Data;
return 0;
}


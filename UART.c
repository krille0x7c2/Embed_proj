#include <avr/io.h>
#include <stdio.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void 
USART0Init(void)
{
// Set baud rate
UBRR0H = (uint8_t)(UBRR_VALUE>>8);
UBRR0L = (uint8_t)UBRR_VALUE;
// Set frame format to 8 data bits, no parity, 1 stop bit
UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
//enable transmission and reception
UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}

int 
USART0SendByte(char u8Data, FILE *stream)
{
   if(u8Data == '\n')
      USART0SendByte('\r', 0);
//wait while previous byte is completed
while(!(UCSR0A&(1<<UDRE0))){};
// Transmit data
UDR0 = u8Data;
return 0;
}


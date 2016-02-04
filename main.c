/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.c
 * Author: krille0x7c2
 *
 * Created on November 21, 2015, 9:11 PM
 */
#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "UART.h"

int entry_state(void);
int ping_state(void);
int send_state(void);
int exit_state(void);

/*State function pointer collection*/
int (*state[]) (void) = {entry_state, ping_state, send_state, exit_state};

/*List the different state codes*/
enum state_codes {entry, ping, send, end};
/*List avaliable return codes*/
enum ret_codes {ok, fail, repeat};

enum event {IN, OUT, NONE};

/*Structure to hold each x-code*/
struct transition {
    enum state_codes src_state;
    enum ret_codes ret_code;
    enum state_codes dst_state;
};

/*Array of transitions, or lookup table with transition rules
 * Each state_transition must contain  one 
 * src_state = which state do we come from.
 * ret_code = what are the current state returning.
 * dst_state = where are we headed next.
 */
struct transition state_transitions[] = {
    {entry, ok, ping},
    {entry, fail, end},
    {ping, ok, send},
    {ping, fail, ping},
    {ping, repeat, ping},
    {send,ok,entry}

};

#define EXIT_STATE end
#define ENTRY_STATE entry

#define PORT_ON( port_letter, number )            port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )           port_letter &= ~(1<<number)
#define PORT_ALL_ON( port_letter, number )      port_letter |= (number)
#define PORT_ALL_OFF( port_letter, number )     port_letter &= ~(number)
#define FLIP_PORT( port_letter, number )          port_letter ^= (1<<number)
#define PORT_IS_ON( port_letter, number )       ( port_letter & (1<<number) )
#define PORT_IS_OFF( port_letter, number )      !( port_letter & (1<<number)

#define PINGPIN_A    6 
#define PINGPIN_B    7 
#define DDR        DDRD
#define PORT       PORTD
#define PIN        PIND
// #define DEBUG 

/*For interrupt vector*/
volatile uint16_t tot_overflow;
/*For debug*/
double pin_6 = 0.0;
double pin_7 = 0.0;

enum event evt = NONE;
//Setup our stream, only write since we don't read
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

/*Here we lookup the coresponding action, related to return codes*/
static enum 
state_codes lookup_transitions(enum state_codes current, enum ret_codes ret) 
{
    int i = 0;
    enum state_codes temp = end;
    for (i = 0;; ++i) {
        if (state_transitions[i].src_state == current && state_transitions[i].ret_code == ret) {
            temp = state_transitions[i].dst_state;
            break;
        }
    }
    return temp;
}

/**********************Timer initialisation************************************/

/* SUMMARY:
 * Initialize timer, interrupt and global variable
 * INFO:
 * Timer 0 and timer 2 are used to meassure the time of the repsponding wave.
 * Since both are 8-bit, they have a mximum count of 255 til overflow generates.
 * The prescalor used is 64, so we have an interrupt each .00102s since
 * 16MHz/64=250KHz -> Ct*(1+MAXc) -> 250KHz*(1+255) = .00102s -> 1.02ms.
 * According to the PING))) datasheet, the maximum period of the signal is
 * 18.5ms, so for maximum period we need to overflow 19 times.
 */
void 
timer0_init(void)
{
    /*Prescaler = 64*/
    TCCR0B |= (1 << CS00) | (1 << CS01);
    TCNT0 = 0;
    /*Enable overflow interrupt*/
    TIMSK0 |= (1 << TOIE0);
    tot_overflow = 0;
}

/**********************End Timer functionality*********************************/

void 
reset_timer_0(void) 
{
    TCNT0 = 0;
    tot_overflow = 0;
}

int 
entry_state() 
{
    return ok;
}



/* SUMMARY:
 * Measure the distance by using ultrasonic waves
 * INFO:
 * The Ping sensor sends a trigger pulse of 5us, which makes the ping sensor
 * send out a ultrasonic burst of 40kHz for 200us. Then we wait for the pulse 
 * to come back. Minimum period is 115us maximum period 18.5ms and the width
 * of the pulse comming back, correspond to 29.033us per centimeter. A one way
 * trip would be
 *
 */
int 
ping_state() 
{

    int elapsed_time;
    double ping_val0,ping_val1;
    /* ------Trigger Pulse A--------------------------*/
    PORT_ON(DDR, PINGPIN_A);   
    PORT_OFF(PORT, PINGPIN_A);   
    _delay_us(2);       
    PORT_ON(PORT, PINGPIN_A);    
    _delay_us(5);       
    PORT_OFF(PORT, PINGPIN_A); 
    /*--------End Trigger Pulse A---------------------*/
    /*--------Meassure pulse A------------------------*/
    FLIP_PORT(DDR, PINGPIN_A);   
    loop_until_bit_is_set(PIN, PINGPIN_A);      
    reset_timer_0();       
    loop_until_bit_is_clear(PIN, PINGPIN_A);    
    /*--------End Meassure pulse A------------------------*/

    /* 255 is count before overflow, dependent on clock*/
    elapsed_time = tot_overflow * 255 + TCNT0;
    tot_overflow = 0;
    ping_val0 = elapsed_time * 0.06667;
    if (ping_val0 <= 90){
        evt = OUT;
    }
    /*Minimum waiting time between measurements is 200us*/
    _delay_us(250);
    /* ------Trigger Pulse B--------------------------*/
    PORT_ON(DDR, PINGPIN_B);
    PORT_OFF(PORT, PINGPIN_B); 
    _delay_us(2); 
    PORT_ON(PORT, PINGPIN_B);
    _delay_us(5); 
    PORT_OFF(PORT, PINGPIN_B);
    /*--------End Trigger Pulse B---------------------*/
    /*--------Meassure pulse B------------------------*/
    FLIP_PORT(DDR, PINGPIN_B);
    loop_until_bit_is_set(PIN, PINGPIN_B);
    reset_timer_0(); 
    loop_until_bit_is_clear(PIN, PINGPIN_B);
    /*--------End Meassure pulse B------------------------*/

    elapsed_time = tot_overflow * 255 + TCNT0;
    tot_overflow = 0;
    ping_val1 = elapsed_time * 0.06667;
    if (ping_val1 <= 90){
        evt = IN;  
    }
    _delay_ms(50);
#ifdef DEBUG
    pin_6 = ping_val0;
    pin_7 = ping_val1;
    return ok;
#else
    return fail ? evt == NONE : ok;
#endif
}

int 
send_state() 
{
#ifdef DEBUG
    printf("pin_6 %.1fcm and pin_7 %.1fcm\n", pin_6, pin_7);
    return ok;
#else
    if( evt == OUT)
        printf("Out\n");
    if( evt == IN)
        printf("In\n");
    evt = NONE;
    return ok;
#endif
}

int 
exit_state() 
{
    printf("We have a problem :-)\n");
    return ok;
}


/*
 * 
 */
int 
main(int argc, char** argv) 
{

    enum state_codes cur_state = ENTRY_STATE;
    enum ret_codes rc;
    int (* state_fun)(void);

    USART0Init();
    sei();

    timer0_init();

    stdin = stdout = &usart0_str;
    
    for (;;) {
        state_fun = state[cur_state];
        rc = state_fun();
        if (EXIT_STATE == cur_state)
            break;
        cur_state = lookup_transitions(cur_state, rc);
    }

    return (EXIT_SUCCESS);
}

/**************************Interrupt service routine*****s*********************/

/* TIMER0 overflow interrupt service routine, called whenever TCNT0 overflows.
 * Used for PING_A 
 */
ISR(TIMER0_OVF_vect)
{
    tot_overflow++;
}

/************************************END***************************************/
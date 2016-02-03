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

/*?*/
int (*state[]) (void) = {entry_state, ping_state, send_state, exit_state};

/*List the different state codes*/
enum state_codes {entry, ping, send, end};
/*List avaliable return codes*/
enum ret_codes {ok, fail, repeat};

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
    /*If we are in entry and we return ok then go to foo*/
    {entry, ok, ping},
    /*If we are in entry and we return fail then go to end*/
    {entry, fail, end},
     /*If we are in foo and we ok then go to bar*/
    {ping, ok, send},
    {ping, fail, ping},
    {ping, repeat, ping},
    {send, ok, entry},
    {send, fail, end},
    {send, repeat, send}
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

static enum state_codes lookup_transitions(enum state_codes current, enum ret_codes ret) {
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

//Globals
volatile uint16_t tot_overflow0;
volatile uint16_t tot_overflow1;

//Setup our stream
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

// initialize timer, interrupt and variable
void timer0_init(void)
{
    // set up timer with prescaler = 64
    TCCR0B |= (1 << CS00) | (1 << CS01);
  
    // initialize counter
    TCNT0 = 0;
  
    // enable overflow interrupt
    TIMSK0 |= (1 << TOIE0);
  
    // enable global interrupts
    sei();
  
    // initialize overflow counter variable
    tot_overflow0 = 0;
}

void timer2_init(void)
{
    // set up timer with prescaler = 64
    TCCR2B |= (1 << CS00) | (1 << CS01);
  
    // initialize counter
    TCNT2 = 0;
  
    // enable overflow interrupt
    TIMSK2 |= (1 << TOIE0);
  
    // initialize overflow counter variable
    tot_overflow1 = 0;
}
void reset_timer_0(void) {
//restart timer count
TCNT0=0x00;//clear timer
//clear timer0's overflow counter.
tot_overflow0 = 0;
}

void reset_timer_2(void) {
//restart timer count
TCNT2=0x00;//clear timer
//clear timer0's overflow counter.
tot_overflow1 = 0;
}
int entry_state(void) {
    return ok;
}

int ping_state(void) 
{
    double ping_val0,ping_val1;
PORT_ON(DDR, PINGPIN_A);   // Switch PingPin to OUPUT
PORT_ON(DDR, PINGPIN_B);
// ------Trigger Pulse--------------
PORT_OFF(PORT, PINGPIN_A);   // Bring PingPin low before starting trigger pulse
PORT_OFF(PORT, PINGPIN_B); 
_delay_us(2);        //  Wait for 2 microseconds
PORT_ON(PORT, PINGPIN_A);    // Bring PingPin High for 5us according to spec sheet.
PORT_ON(PORT, PINGPIN_B);
_delay_us(5);       // Wait for 5 microseconds
PORT_OFF(PORT, PINGPIN_A); //  Bring PingPin Low and standby
PORT_OFF(PORT, PINGPIN_B);
//--------End Trigger Pulse---------------------
FLIP_PORT(DDR, PINGPIN_A);   // Switch PingPin to INPUT
FLIP_PORT(DDR, PINGPIN_B);
loop_until_bit_is_set(PIN, PINGPIN_A);     // Loop until the the PingPin goes high  (macro found in sfr_def.h)
loop_until_bit_is_set(PIN, PINGPIN_B);
//clears timer, reset overflow counter
reset_timer_0();       //reset timer 0
reset_timer_2(); 
loop_until_bit_is_clear(PIN, PINGPIN_A);     // Loop until the the PingPin goes low  (macro found in sfr_def.h)
loop_until_bit_is_clear(PIN, PINGPIN_B);
//read timer0's overflow counter
//255 is count before overflow, dependent on clock
int elapsed_time0=tot_overflow0*255+TCNT0;
tot_overflow0 = 0;
ping_val0 = elapsed_time0 * 0.6667;
int elapsed_time1=tot_overflow1*255+TCNT2;
tot_overflow1 = 0;
ping_val1 = elapsed_time1 * 0.6667;
_delay_ms(50);
return fail ? ping_val0 >=900 && ping_val1 >=900 : ok;
}

int alarm_state(void) {
    
    return ok;
}

int send_state(void) {
    printf("A ");
    return ok;
}

int exit_state(void) {
    printf("We have a problem :-)\n");
    return ok;
}

/*
 * 
 */
int main(int argc, char** argv) {

    enum state_codes cur_state = ENTRY_STATE;
    enum ret_codes rc;
    int (* state_fun)(void);
    //Init UART
    USART0Init();
    //Init timers
    timer0_init();
    timer2_init();
    //assign our stream to standart I/O streams
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

// TIMER0 overflow interrupt service routine
// called whenever TCNT0 overflows
ISR(TIMER0_OVF_vect)
{
    // keep a track of number of overflows
    tot_overflow0++;
}

ISR(TIMER2_OVF_vect)
{
    // keep a track of number of overflows
    tot_overflow1++;
}

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
/**********************Includes************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "UART.h"
/**********************End Includes********************************************/

/**********************Prototypes**********************************************/
uint8_t entry_state(void);
uint8_t ping_state(void);
uint8_t send_state(void);
uint8_t exit_state(void);

void reset_timer_0(void);
void timer0_init(void);
/**********************End Prototypes******************************************/

/**********************Structures and Variables********************************/
/*State function pointer collection*/
static uint8_t (*state[]) (void) = {entry_state, ping_state, send_state, exit_state};

/*For interrupt vector*/
volatile uint8_t tot_overflow;

/*From datasheet, convertion factor*/
static const float TO_CM = 0.0667;
static const float TO_MM = 0.667;

/*---------------------Startup values, assigned in entry state----------------*/

/*How long will the sensors ping, in cm*/
static uint8_t threshold;
/*What max distance for each ping should we include*/
static uint16_t ping_distance_max;
/*What min distance for each ping should we include*/
static uint16_t ping_distance_min;

/*---------------------End Startup values-------------------------------------*/

/*For debug*/
static double pin_6 = 0.0;
static double pin_7 = 0.0;

/*Define the posible events that can occur*/
enum event {SENSOR_A_EVT, SENSOR_B_EVT, NONE};

/*Used to (hold)identify which sensor that tripped first*/
static enum event evt = NONE;

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

/* Array of transitions, or lookup table with transition rules
 * Each transition must contain  one 
 * src_state = which state do we come from.
 * ret_code = what are the current state returning.
 * dst_state = where are we headed next.
 */
struct transition state_transitions[] = {
    {entry, ok,     ping},
    {entry, fail,   end},
    {entry, repeat, entry},
    {ping,  ok,     send},
    {ping,  fail,   end},
    {ping,  repeat, ping},
    {send,  ok,     ping},
    {send,  fail,   end},
    {send,  repeat, send}

};

//Setup our stream, only write since we don't read
static FILE uart0_str = FDEV_SETUP_STREAM(UART0SendByte, NULL, _FDEV_SETUP_WRITE);

/**********************End Structures and Variables****************************/

/**********************Defines*************************************************/
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
#define MAXCOUNTER 255
#define DEBUG 

/**********************End Defines*********************************************/

/**********************Private functions***************************************/

/* SUMMARY:
 * Here we lookup the coresponding action, related to return codes
 * INFO:
 * [1] Assign the first temporary state as end
 * LOOKUP LOOP:
 * [1] If the current state equals a state in the table
 * and the return code match a return code in the table.
 * [2] Assign the new state to the temporary state
 * [3] Repeat if not found
 * [4] If found, break and return the next state
 */
static enum 
state_codes lookup_transitions(enum state_codes current, enum ret_codes ret) 
{
    int i = 0;
    enum state_codes temp = end;
    for (i = 0;; ++i) {
        if (state_transitions[i].src_state == current && 
            state_transitions[i].ret_code == ret) {
            temp = state_transitions[i].dst_state;
            break;
        }
    }
    return temp;
}

/* SUMMARY:TODO fix calculations
 * Use the PING))) device to meassure the distance
 * INFO:
 * C_CPU / 64 = 250kH one clock cycle 1/250kH = 4us
 * Resolution 4us*(1+255) = 1.024ms/255 = 4.016us each tick
 * The width of the pulse per centimeter is according to the datasheet
 * 29.033us/cm so maximum width of 300cm would be 29.033 * 600cm = 17.41ms round
 * trip time yeilding a upper limit of elapsed time to 17 * 255 + (255 * 0.41) =
 * 4439
 * minimum width of the pulse would be 29.033 * 4cm = 1.161us yeilding a lower
 * limit to (1.024ms/0.1161ms) = 255 / 8.81 = ~28
 * Speed of sound depends on temperature E.g. C = 331.5 + (0.6 x Tc)m/s
 * Meassure by pulling, not exact for meassurements but for our implementation
 * this is enough.
 * 
 */

static void
echo(double *ping_value,const uint8_t pingpin)
{
    /*Round-trip-time, unit microseconds*/
    uint16_t elapsed_time;
    /* ------Trigger Pulse--------------------------*/
    PORT_ON(DDR, pingpin);   
    PORT_OFF(PORT, pingpin);   
    _delay_us(2);       
    PORT_ON(PORT, pingpin);    
    _delay_us(5);       
    PORT_OFF(PORT, pingpin); 
    /*--------End Trigger Pulse---------------------*/
    /*--------Meassure pulse------------------------*/
    FLIP_PORT(DDR, pingpin);   
    loop_until_bit_is_set(PIN, pingpin);      
    reset_timer_0();       
    loop_until_bit_is_clear(PIN, pingpin);
    /*--------Stop Meassure pulse-------------------*/
    
    /* MAXCOUNTER is dependent on timer */
    elapsed_time = (tot_overflow * MAXCOUNTER) + TCNT0;

    /*Simple band-pass filter*/
    if (elapsed_time >= ping_distance_max)
    	*ping_value = 0;
    else if (elapsed_time <= ping_distance_min)
    	*ping_value = 0;
    else
		*ping_value = (elapsed_time * TO_CM);
    _delay_us(250);
    
}

/**********************End Private functions***********************************/

/**********************Timer initialisation************************************/

/* SUMMARY:
 * Initialize timer, interrupt and global variable
 * INFO:
 * Timer 0 are used to meassure the time of the repsponding wave.
 * Since timer 0 has a 8-bit resolution, it have a mximum count of 255 
 * til overflow generates. The prescalor used is 64, so we have an interrupt 
 * each .00102s since 16MHz/64=250KHz -> Ct*(1+MAXc) -> 250KHz*(1+255) = 
 * .00102s -> 1.02ms. According to the PING))) datasheet, the maximum width 
 * of the signal is 18.5ms, so for maximum width we need to overflow 19 times.
 * each cm is 29.034us
 */
void 
timer0_init()
{
    /*Prescaler = 64*/
    TCCR0B |= (1 << CS00) | (1 << CS01);
    /*Enable overflow interrupt*/
    TIMSK0 |= (1 << TOIE0);
    reset_timer_0();
}


/* SUMMARY:
 * Here we reset timer register and the overflow counter
 * INFO:
 * 
 */
void 
reset_timer_0() 
{
    TCNT0 = 0;
    tot_overflow = 0;
}

/**********************End Timer functionality*********************************/

/**********************States**************************************************/

/* SUMMARY: TODO
 * Used to set up our inital state
 * INFO:
 * 2016-02-09 threshold only used 
 * 2016-02-10 ping distance added
 */
uint8_t
entry_state() 
{
    threshold = 90;
    ping_distance_max = 4439;
    ping_distance_min = 28;

    return ok;
}

/* SUMMARY:
 * Measure the distance by using ultrasonic waves
 * INFO:
 * The Ping sensor sends a trigger pulse of 5us, which makes the ping sensor
 * send out a ultrasonic burst of 40kHz for 200us. Then we wait for the pulse 
 * to come back.
 * Since the implementations is targeting slow humans, some toughness is 
 * introduced in the system, by using _delay functions (just count clock,cy).
 * 
 *
 */
uint8_t
ping_state() 
{

    double sensor_A_val,sensor_B_val;   

    echo(&sensor_A_val,PINGPIN_A);
    echo(&sensor_B_val,PINGPIN_B);

    pin_6 = sensor_A_val;
    pin_7 = sensor_B_val; 

#ifdef DEBUG
    return ok;
#endif
        
    if(sensor_A_val <= threshold){
        do{
            echo(&sensor_A_val,PINGPIN_A);
            _delay_ms(50);
        }while(sensor_A_val <= threshold);
        do{
            echo(&sensor_B_val,PINGPIN_B);
            _delay_ms(50);
        }while(sensor_B_val <= threshold);
            _delay_ms(500);
        evt = SENSOR_A_EVT;
        return ok;
    }else if(sensor_B_val <= threshold){
        do{
            echo(&sensor_B_val,PINGPIN_B);
            _delay_ms(50);
        }while(sensor_B_val <= threshold);
        do{
            echo(&sensor_A_val,PINGPIN_A);
            _delay_ms(50);
        }while(sensor_A_val <= threshold);
        _delay_ms(500);
        evt = SENSOR_B_EVT;
        return ok;
    }else{
        _delay_ms(50);
        return repeat;
    }  
    return fail;
}

/* SUMMARY:
 * Send in or out
 * INFO:
 * Reports the current event and prints distance in cm if debug is defined.
 * WARNING:
 * Printing the float's will link the floating point version of printf,
 * this will an extra ~4k bytes to the resulting binary and should be alterd in
 * the makefile before submission.
 */
uint8_t
send_state() 
{
#ifdef DEBUG
    printf("pin_6 %.1fcm and pin_7 %.1fcm\n", pin_6, pin_7);
    return ok;
#else
    if( evt == SENSOR_A_EVT)
        printf("A\n");
    if( evt == SENSOR_B_EVT)
        printf("B\n");
    evt = NONE;
    return ok;
#endif
}

/* SUMMARY:
 * Used for testing/debuging, should not end up here
 * INFO:
 * Will only be used for testing/debuging. If we end up here, the program will
 * halt.
 */
uint8_t
exit_state() 
{
    printf("We have a problem :-)\n");
    return ok;
}
/**********************End states**********************************************/

/**********************Main****************************************************/

/* SUMMARY:
 * Main function
 * INFO:
 * [1] Start by init the watchdog, see wdt.h for macro options. 
 * [2] Init the current state to the entry state and the collection of return 
 * codes. 
 * [3] Setup the function pointer for the diffrent states. 
 * [4] Init the serial communication and enable global interrupts and 
 * init timer0. 
 * [5] Redirect the stdout and the stdin for serial streams through uart.
 * MAIN LOOP:
 * [1] Assign the function pointer a current state
 * [2] Call the pointed function, and collect it's return code
 * [3] Reset the watchdog
 * [4] Check if the current state is exit and break(This will terminate)
 * [5] If not, then move to the next state, declared in the transition table.
 * [6] Repeat
 */
int
main(void) 
{
    enum state_codes cur_state = ENTRY_STATE;
    enum ret_codes rc;
    uint8_t (* state_fun)(void);

    UART0Init();
    sei();
    timer0_init();

    stdin = stdout = &uart0_str;
    
    for (;;) {
        state_fun = state[cur_state];
        rc = state_fun();
        
        if (EXIT_STATE == cur_state)
            break;
        cur_state = lookup_transitions(cur_state, rc);
    }
    return 0;
}

/**********************End Main************************************************/

/**************************Interrupt service routine*****s*********************/

/* SUMMARY:
 * Interupt service routine for timer 0
 * INFO:
 * TIMER0 overflow interrupt service routine, called whenever TCNT0 overflows.
 * Used for PING_A 
 */
ISR(TIMER0_OVF_vect)
{
    tot_overflow++;
    /*Integer overflow protection (Not needed but just in case)*/
    if (tot_overflow >= 20)
        tot_overflow = 0;
}

/************************************END***************************************/
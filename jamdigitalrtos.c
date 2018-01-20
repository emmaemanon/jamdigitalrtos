// ------- Preamble -------- //
#include <stdio.h>
#include <avr/io.h> /* Defines pins, ports, etc */
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h> /* Functions to waste time */
#include "uart/uart.h"

/* Scheduler include files. */
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/time.h"

// system states
#define DEFAULT					0
#define PRE_DATE				1
#define DATE					2
#define POST_DATE				3
#define PRE_CHANGE_TIME			4
#define CHANGE_TIME_h			5
#define POST_CHANGE_TIME_h		6
#define CHANGE_TIME_COUNTER_h	7
#define CHANGE_TIME_m			8
#define POST_CHANGE_TIME_m		9
#define CHANGE_TIME_COUNTER_m	10
#define CHANGE_TIME_s			11
#define POST_CHANGE_TIME_s		12
#define CHANGE_TIME_COUNTER_s	13
#define PRE_CHANGE_DATE			14
#define CHANGE_DATE_D			15
#define POST_CHANGE_DATE_D		16
#define CHANGE_DATE_COUNTER_D	17
#define CHANGE_DATE_M			18
#define POST_CHANGE_DATE_M		19
#define CHANGE_DATE_COUNTER_M	20
#define CHANGE_DATE_Y			21
#define POST_CHANGE_DATE_Y		22
#define CHANGE_DATE_COUNTER_Y	23

#define DDRLED	DDRC
#define PRTLED	PORTC
#define PN_LED	PC4

#define DDRBUTTON	DDRC
#define PRTBUTTON	PORTC
#define PINBUTTON	PINC
#define PN_BUTTON	PC2

#define DDRBUTTON2	DDRC
#define PRTBUTTON2	PORTC
#define PINBUTTON2	PINC
#define PN_BUTTON2	PC3

// UNTUK DISPLAY
#define DDRSEG_A	DDRD
#define PRTSEG_A	PORTD
#define PINSEG_A	PIND
#define PN_SEG_A	PD2

#define DDRSEG_B	DDRD
#define PRTSEG_B	PORTD
#define PINSEG_B	PIND
#define PN_SEG_B	PD3

#define DDRSEG_C	DDRD
#define PRTSEG_C	PORTD
#define PINSEG_C	PIND
#define PN_SEG_C	PD4

#define DDRSEG_D	DDRD
#define PRTSEG_D	PORTD
#define PINSEG_D	PIND
#define PN_SEG_D	PD5

#define DDRSEG_E	DDRD
#define PRTSEG_E	PORTD
#define PINSEG_E	PIND
#define PN_SEG_E	PD6

#define DDRSEG_F	DDRD
#define PRTSEG_F	PORTD
#define PINSEG_F	PIND
#define PN_SEG_F	PD7

#define DDRSEG_G	DDRB
#define PRTSEG_G	PORTB
#define PINSEG_G	PINB
#define PN_SEG_G	PB0

#define DDRSEG_DP	DDRB
#define PRTSEG_DP	PORTB
#define PINSEG_DP	PINB
#define PN_SEG_DP	PB1

// DEFINITIONS FOR EACH DIGIT
#define DDRDIG_00	DDRC
#define PRTDIG_00	PORTC
#define PINDIG_00	PINC
#define PN_DIG_00	PC0

#define DDRDIG_01	DDRC
#define PRTDIG_01	PORTC
#define PINDIG_01	PINC
#define PN_DIG_01	PC1

#define DDRDIG_02	DDRB
#define PRTDIG_02	PORTB
#define PINDIG_02	PINB
#define PN_DIG_02	PB5

#define DDRDIG_03	DDRB
#define PRTDIG_03	PORTB
#define PINDIG_03	PINB
#define PN_DIG_03	PB4

#define DDRDIG_04	DDRB
#define PRTDIG_04	PORTB
#define PINDIG_04	PINB
#define PN_DIG_04	PB3

#define DDRDIG_05	DDRB
#define PRTDIG_05	PORTB
#define PINDIG_05	PINB
#define PN_DIG_05	PB2

// UNTUK SERIAL COMM
#define UART_BAUD_RATE 9600

// Push Button States
#define RELEASED_STATE		0
#define MAYBE_PUSH_STATE	1
#define PUSHED_STATE		2
#define MAYBE_REL_STATE		3

// Keypad Button Number
#define NO_BUTTON_PUSHED	0xFF
#define BUTTON_00_PUSHED	0x0F
#define BUTTON_01_PUSHED	0xF0

// Push Button Flags
#define NOT_PUSHED_FLAG		0
#define PUSHED_FLAG			1


#define DEBOUNCE_TIME 1000 //debounce time in microseconds
#define TASK_PERIOD_DEBOUNCE_MS 30 // long press time in milliseconds
#define TASK_PERIOD_TIMER_MS 1000 // rate of clock, in miliseconds

volatile uint8_t sys_state = DEFAULT;
volatile uint16_t task_timer = TASK_PERIOD_TIMER_MS;

// Keypad scanning and debounce variables
volatile uint8_t butnum;			// The button number
volatile uint8_t last_butnum;		// Last stored button number

volatile uint16_t year = 2018;
volatile uint8_t month = 01;
volatile uint8_t day = 01;
volatile uint16_t year_t = 18;
volatile uint8_t month_t = 01;
volatile uint8_t day_t = 01;

volatile uint8_t hour = 0;
volatile uint8_t minute = 0;
volatile uint8_t second = 0;
volatile uint8_t hour_t = 0;
volatile uint8_t minute_t = 0;
volatile uint8_t second_t = 0;

volatile uint8_t dpdig00 = 0;
volatile uint8_t dpdig01 = 0;
volatile uint8_t dpdig02 = 1;
volatile uint8_t dpdig03 = 0;
volatile uint8_t dpdig04 = 1;
volatile uint8_t dpdig05 = 0;

// static void TaskButton(void *pvParameters); //
static void TaskState(void *pvParameters); //
static void TaskDisplay(void *pvParameters); //

void set_dp(uint8_t dp_stat) {
	if (dp_stat == 1) {
		PRTSEG_DP |= (1 << PN_SEG_DP);
	}
	else {
		PRTSEG_DP &= ~(1 << PN_SEG_DP);
	}
}

void set_seg_num(uint8_t seg_num) {
	switch(seg_num) {
		case 0:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E |= (1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G &= ~(1 << PN_SEG_G);
		break;
		case 1:
			PRTSEG_A &= ~(1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D &= ~(1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F &= ~(1 << PN_SEG_F);
			PRTSEG_G &= ~(1 << PN_SEG_G);
		break;
		case 2:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C &= ~(1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E |= (1 << PN_SEG_E);
			PRTSEG_F &= ~(1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 3:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F &= ~(1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 4:
			PRTSEG_A &= ~(1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D &= ~(1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 5:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B &= ~(1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 6:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B &= ~(1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E |= (1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 7:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D &= ~(1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F &= ~(1 << PN_SEG_F);
			PRTSEG_G &= ~(1 << PN_SEG_G);
		break;
		case 8:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E |= (1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		case 9:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E &= ~(1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
		default:
			PRTSEG_A |= (1 << PN_SEG_A);
			PRTSEG_B |= (1 << PN_SEG_B);
			PRTSEG_C |= (1 << PN_SEG_C);
			PRTSEG_D |= (1 << PN_SEG_D);
			PRTSEG_E |= (1 << PN_SEG_E);
			PRTSEG_F |= (1 << PN_SEG_F);
			PRTSEG_G |= (1 << PN_SEG_G);
		break;
	}
}

uint8_t read_button(void) {
	last_butnum = butnum;
	
	if ((((PINBUTTON & (1 << PN_BUTTON)) == 0))  && ((PINBUTTON2 & (1 << PN_BUTTON2)) != 0)) {
		_delay_us(DEBOUNCE_TIME);
		if ((PINBUTTON & (1 << PN_BUTTON)) == 0) { /* still pressed */
			return (BUTTON_00_PUSHED);
		}
	}
	if (((PINBUTTON & (1 << PN_BUTTON)) != 0) && ((PINBUTTON2 & (1 << PN_BUTTON2)) == 0)) {
		_delay_us(DEBOUNCE_TIME);
		if ((PINBUTTON2 & (1 << PN_BUTTON2)) == 0) { /* still pressed */
			return (BUTTON_01_PUSHED);
		}
	}
	return NO_BUTTON_PUSHED;
}

void display_segments_time(void) {
	// ACTIVATE DIGIT 00
	set_seg_num(second%10);
	set_dp(dpdig00);
	PRTDIG_00 &= ~(1 << PN_DIG_00);
	_delay_ms(1);
	// DEACTIVATE DIGIT 00
	PRTDIG_00 |= (1 << PN_DIG_00);
	
	// ACTIVATE DIGIT 01
	set_seg_num(second/10);
	set_dp(dpdig01);
	PRTDIG_01 &= ~(1 << PN_DIG_01);
	_delay_ms(1);
	// DEACTIVATE DIGIT 01
	PRTDIG_00 |= (1 << PN_DIG_01);
	
	// ACTIVATE DIGIT 02
	set_seg_num(minute%10);
	set_dp(dpdig02);
	PRTDIG_02 &= ~(1 << PN_DIG_02);
	_delay_ms(1);
	// DEACTIVATE DIGIT 02
	PRTDIG_02 |= (1 << PN_DIG_02);
	
	// ACTIVATE DIGIT 03
	set_seg_num(minute/10);
	set_dp(dpdig03);
	PRTDIG_03 &= ~(1 << PN_DIG_03);
	_delay_ms(1);
	// DEACTIVATE DIGIT 03
	PRTDIG_03 |= (1 << PN_DIG_03);
	
	// ACTIVATE DIGIT 04
	set_seg_num(hour%10);
	set_dp(dpdig04);
	PRTDIG_04 &= ~(1 << PN_DIG_04);
	_delay_ms(1);
	// DEACTIVATE DIGIT 04
	PRTDIG_04 |= (1 << PN_DIG_04);
	
	// ACTIVATE DIGIT 05=
	set_seg_num(hour/10);
	set_dp(dpdig05);
	PRTDIG_05 &= ~(1 << PN_DIG_05);
	_delay_ms(1);
	// DEACTIVATE DIGIT 05
	PRTDIG_05 |= (1 << PN_DIG_05);
}


void display_segments_change_time(void) {
	// ACTIVATE DIGIT 00
	set_seg_num(second_t%10);
	set_dp(dpdig00);
	PRTDIG_00 &= ~(1 << PN_DIG_00);
	_delay_ms(1);
	// DEACTIVATE DIGIT 00
	PRTDIG_00 |= (1 << PN_DIG_00);
	
	// ACTIVATE DIGIT 01
	set_seg_num(second_t/10);
	set_dp(dpdig01);
	PRTDIG_01 &= ~(1 << PN_DIG_01);
	_delay_ms(1);
	// DEACTIVATE DIGIT 01
	PRTDIG_00 |= (1 << PN_DIG_01);
	
	// ACTIVATE DIGIT 02
	set_seg_num(minute_t%10);
	set_dp(dpdig02);
	PRTDIG_02 &= ~(1 << PN_DIG_02);
	_delay_ms(1);
	// DEACTIVATE DIGIT 02
	PRTDIG_02 |= (1 << PN_DIG_02);
	
	// ACTIVATE DIGIT 03
	set_seg_num(minute_t/10);
	set_dp(dpdig03);
	PRTDIG_03 &= ~(1 << PN_DIG_03);
	_delay_ms(1);
	// DEACTIVATE DIGIT 03
	PRTDIG_03 |= (1 << PN_DIG_03);
	
	// ACTIVATE DIGIT 04
	set_seg_num(hour_t%10);
	set_dp(dpdig04);
	PRTDIG_04 &= ~(1 << PN_DIG_04);
	_delay_ms(1);
	// DEACTIVATE DIGIT 04
	PRTDIG_04 |= (1 << PN_DIG_04);
	
	// ACTIVATE DIGIT 05=
	set_seg_num(hour_t/10);
	set_dp(dpdig05);
	PRTDIG_05 &= ~(1 << PN_DIG_05);
	_delay_ms(1);
	// DEACTIVATE DIGIT 05
	PRTDIG_05 |= (1 << PN_DIG_05);
}

void display_segments_date(void) {
	// ACTIVATE DIGIT 00
	set_seg_num(year%10);
	set_dp(dpdig00);
	PRTDIG_00 &= ~(1 << PN_DIG_00);
	_delay_ms(1);
	// DEACTIVATE DIGIT 00
	PRTDIG_00 |= (1 << PN_DIG_00);
	
	// ACTIVATE DIGIT 01
	set_seg_num(year/10 - 200);
	set_dp(dpdig01);
	PRTDIG_01 &= ~(1 << PN_DIG_01);
	_delay_ms(1);
	// DEACTIVATE DIGIT 01
	PRTDIG_00 |= (1 << PN_DIG_01);
	
	// ACTIVATE DIGIT 02
	set_seg_num(month%10);
	set_dp(dpdig02);
	PRTDIG_02 &= ~(1 << PN_DIG_02);
	_delay_ms(1);
	// DEACTIVATE DIGIT 02
	PRTDIG_02 |= (1 << PN_DIG_02);
	
	// ACTIVATE DIGIT 03
	set_seg_num(month/10);
	set_dp(dpdig03);
	PRTDIG_03 &= ~(1 << PN_DIG_03);
	_delay_ms(1);
	// DEACTIVATE DIGIT 03
	PRTDIG_03 |= (1 << PN_DIG_03);
	
	// ACTIVATE DIGIT 04
	set_seg_num(day%10);
	set_dp(dpdig04);
	PRTDIG_04 &= ~(1 << PN_DIG_04);
	_delay_ms(1);
	// DEACTIVATE DIGIT 04
	PRTDIG_04 |= (1 << PN_DIG_04);
	
	// ACTIVATE DIGIT 05
	set_seg_num(day/10);
	set_dp(dpdig05);
	PRTDIG_05 &= ~(1 << PN_DIG_05);
	_delay_ms(1);
	// DEACTIVATE DIGIT 05
	PRTDIG_05 |= (1 << PN_DIG_05);
}


void display_segments_change_date(void) {
	// ACTIVATE DIGIT 00
	set_seg_num(year_t%10);
	set_dp(dpdig00);
	PRTDIG_00 &= ~(1 << PN_DIG_00);
	_delay_ms(1);
	// DEACTIVATE DIGIT 00
	PRTDIG_00 |= (1 << PN_DIG_00);
	
	// ACTIVATE DIGIT 01
	set_seg_num(year_t/10 - 200);
	set_dp(dpdig01);
	PRTDIG_01 &= ~(1 << PN_DIG_01);
	_delay_ms(1);
	// DEACTIVATE DIGIT 01
	PRTDIG_00 |= (1 << PN_DIG_01);
	
	// ACTIVATE DIGIT 02
	set_seg_num(month_t%10);
	set_dp(dpdig02);
	PRTDIG_02 &= ~(1 << PN_DIG_02);
	_delay_ms(1);
	// DEACTIVATE DIGIT 02
	PRTDIG_02 |= (1 << PN_DIG_02);
	
	// ACTIVATE DIGIT 03
	set_seg_num(month_t/10);
	set_dp(dpdig03);
	PRTDIG_03 &= ~(1 << PN_DIG_03);
	_delay_ms(1);
	// DEACTIVATE DIGIT 03
	PRTDIG_03 |= (1 << PN_DIG_03);
	
	// ACTIVATE DIGIT 04
	set_seg_num(day_t%10);
	set_dp(dpdig04);
	PRTDIG_04 &= ~(1 << PN_DIG_04);
	_delay_ms(1);
	// DEACTIVATE DIGIT 04
	PRTDIG_04 |= (1 << PN_DIG_04);
	
	// ACTIVATE DIGIT 05=
	set_seg_num(day_t/10);
	set_dp(dpdig05);
	PRTDIG_05 &= ~(1 << PN_DIG_05);
	_delay_ms(1);
	// DEACTIVATE DIGIT 05
	PRTDIG_05 |= (1 << PN_DIG_05);
}


int main(void) {
	// -------- Inits --------- //
	DDRC |= (1 << PC4);
	PORTC &= ~(1 << PC4);
	
	// DDRLED |= (1 << PN_LED);
	// PRTLED &= ~(1 << PN_LED);
	
	DDRBUTTON &= ~(1 << PN_BUTTON);
	PRTBUTTON |= (1 << PN_BUTTON);
	
	DDRBUTTON2 &= ~(1 << PN_BUTTON2);
	PRTBUTTON2 |= (1 << PN_BUTTON2);
	
	// INISIASI PIN2 SEVEN SEGMENT
	// SEGMENT PINS AS OUTPUT
	DDRSEG_A |= (1 << PN_SEG_A);
	DDRSEG_B |= (1 << PN_SEG_B);
	DDRSEG_C |= (1 << PN_SEG_C);
	DDRSEG_D |= (1 << PN_SEG_D);
	DDRSEG_E |= (1 << PN_SEG_E);
	DDRSEG_F |= (1 << PN_SEG_F);
	DDRSEG_G |= (1 << PN_SEG_G);
	DDRSEG_DP |= (1 << PN_SEG_DP);
	
	// DIGIT GND PINS AS OUTPUT
	DDRDIG_00 |= (1 << PN_DIG_00);
	DDRDIG_01 |= (1 << PN_DIG_01);
	DDRDIG_02 |= (1 << PN_DIG_02);
	DDRDIG_03 |= (1 << PN_DIG_03);
	DDRDIG_04 |= (1 << PN_DIG_04);
	DDRDIG_05 |= (1 << PN_DIG_05);
	
	// TURN OFF ALL SEGMENTS
	PRTSEG_A &= ~(1 << PN_SEG_A);
	PRTSEG_B &= ~(1 << PN_SEG_B);
	PRTSEG_C &= ~(1 << PN_SEG_C);
	PRTSEG_D &= ~(1 << PN_SEG_D);
	PRTSEG_E &= ~(1 << PN_SEG_E);
	PRTSEG_F &= ~(1 << PN_SEG_F);
	PRTSEG_G &= ~(1 << PN_SEG_G);
	PRTSEG_DP &= ~(1 << PN_SEG_DP);
	
	// DEACTIVATE ALL DIGITS
	PRTDIG_00 |= (1 << PN_DIG_00);
	PRTDIG_01 |= (1 << PN_DIG_01);
	PRTDIG_02 |= (1 << PN_DIG_02);
	PRTDIG_03 |= (1 << PN_DIG_03);
	PRTDIG_04 |= (1 << PN_DIG_04);
	PRTDIG_05 |= (1 << PN_DIG_05);
	
	// ------------------- timer init code --------------------
	TCCR1B = (1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); //CS12 untuk prescaler 1024, WGM12 untuk mode operasi CTC
	OCR1A = 249; // nilai OCRA dengan F_CPU 16MHz dan untuk waktu countdown 1 detik
	TIMSK1 = (1<<OCIE1A); // mengaktifkan Timer/Counter2 Compare Match A
	
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	sei(); // serial library Fleury ini harus pakai interrupt
	
	
    // xTaskCreate(
		// TaskButton
		// ,  (const portCHAR *)"TaskButton"
		// ,  256
		// ,  NULL
		// ,  3
		// ,  NULL ); // */
	
    xTaskCreate(
		TaskState
		,  (const portCHAR *)"TaskState"
		,  256
		,  NULL
		,  3
		,  NULL ); // */
	
    xTaskCreate(
		TaskDisplay
		,  (const portCHAR *)"TaskDisplay"
		,  256
		,  NULL
		,  3
		,  NULL ); // */
	
	vTaskStartScheduler();
	
	
	// ------ Event loop ------ //
	while (1) {
		
		
		
		
		
	}
	
	return (0); /* This line is never reached */
}


// static void TaskButton(void *pvParameters) {
    // (void) pvParameters;
    // TickType_t xLastWakeTime;
	// xLastWakeTime = xTaskGetTickCount();
    // for(;;) {
		// // butnum = read_button();	// Scan button
		
		// // vTaskDelayUntil( &xLastWakeTime, ( 1 / portTICK_PERIOD_MS ));
    // }
// }

static void TaskState(void *pvParameters) {
    (void) pvParameters;
    TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    for(;;) {
		// PORTC |= (1 << PC4);
		butnum = read_button();	// Scan button
		switch (sys_state) {
			// DEFAULT: menampilkan waktu
			case DEFAULT:
				if (butnum == BUTTON_00_PUSHED) { sys_state = PRE_DATE; }
				if (butnum == BUTTON_01_PUSHED) { sys_state = PRE_CHANGE_TIME; }
			break;
			case PRE_DATE:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = DATE; }
			break;
			case DATE: // menampilkan tanggal
				// display_segments_date();
				if (butnum == BUTTON_00_PUSHED) { sys_state = POST_DATE; }
				if (butnum == BUTTON_01_PUSHED) { sys_state = PRE_CHANGE_DATE; }
			break;
			case POST_DATE:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = DEFAULT; }
			break;
			case PRE_CHANGE_TIME:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_TIME_h; }
			break;
			
			// Change Hour
			case CHANGE_TIME_h:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					hour_t++;
					if (hour_t > 23) { hour_t = 0; }
					sys_state = CHANGE_TIME_COUNTER_h;
				}
				if (butnum == BUTTON_01_PUSHED) { sys_state = POST_CHANGE_TIME_h; }
			break;
			case CHANGE_TIME_COUNTER_h:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_TIME_h; }
			break;
			case POST_CHANGE_TIME_h:
				if (butnum == NO_BUTTON_PUSHED) { dpdig04 = 0; sys_state = CHANGE_TIME_m; }
			break;
			
			// Change Minute
			case CHANGE_TIME_m:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					minute_t++;
					if (minute_t > 59) { minute_t = 0; }
					sys_state = CHANGE_TIME_COUNTER_m;
				}
				if (butnum == BUTTON_01_PUSHED) { sys_state = POST_CHANGE_TIME_m; }
			break;
			case CHANGE_TIME_COUNTER_m:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_TIME_m; }
			break;
			case POST_CHANGE_TIME_m:
				if (butnum == NO_BUTTON_PUSHED) { dpdig02 = 0; sys_state = CHANGE_TIME_s; }
			break;
			
			// Change Second
			case CHANGE_TIME_s:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					second_t++;
					if (second_t > 59) { second_t = 0; }
					sys_state = CHANGE_TIME_COUNTER_s;
				}
				if (butnum == BUTTON_01_PUSHED) { task_timer = TASK_PERIOD_TIMER_MS; hour = hour_t; minute = minute_t; second = second_t; sys_state = POST_CHANGE_TIME_s; }
			break;
			case CHANGE_TIME_COUNTER_s:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_TIME_s; }
			break;
			case POST_CHANGE_TIME_s:
				if (butnum == NO_BUTTON_PUSHED) { dpdig00 = 0; sys_state = DEFAULT; }
			break;
			
			/////////////////////////////////////////////////
			case PRE_CHANGE_DATE:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_DATE_D; }
			break;
			
			// Change Day
			case CHANGE_DATE_D:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					day_t++;
					if (day_t > 31) { day_t = 0; }
					sys_state = CHANGE_DATE_COUNTER_D;
				}
				if (butnum == BUTTON_01_PUSHED) { sys_state = POST_CHANGE_DATE_D; }
			break;
			case CHANGE_DATE_COUNTER_D:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_DATE_D; }
			break;
			case POST_CHANGE_DATE_D:
				if (butnum == NO_BUTTON_PUSHED) { dpdig04 = 0; sys_state = CHANGE_DATE_M; }
			break;
			
			// Change Month
			case CHANGE_DATE_M:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					month_t++;
					if (month_t > 12) { month_t = 0; }
					sys_state = CHANGE_DATE_COUNTER_M;
				}
				if (butnum == BUTTON_01_PUSHED) { sys_state = POST_CHANGE_DATE_M; }
			break;
			case CHANGE_DATE_COUNTER_M:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_DATE_M; }
			break;
			case POST_CHANGE_DATE_M:
				if (butnum == NO_BUTTON_PUSHED) { dpdig02 = 0; sys_state = CHANGE_DATE_Y; }
			break;
			
			// Change Year
			case CHANGE_DATE_Y:
				if ((butnum == BUTTON_00_PUSHED) && (last_butnum == NO_BUTTON_PUSHED)) {
					year_t++;
					if (year_t > 2099) { year_t = 2000; }
					sys_state = CHANGE_DATE_COUNTER_Y;
				}
				if (butnum == BUTTON_01_PUSHED) { day = day_t; month = month_t; year = year_t; sys_state = POST_CHANGE_DATE_Y; }
			break;
			case CHANGE_DATE_COUNTER_Y:
				if (butnum == NO_BUTTON_PUSHED) { sys_state = CHANGE_DATE_Y; }
			break;
			case POST_CHANGE_DATE_Y:
				if (butnum == NO_BUTTON_PUSHED) { dpdig00 = 0; sys_state = DEFAULT; }
			break;
			default: break;
		}
		// PORTC &= ~(1 << PC4);
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_PERIOD_MS ));
    }
}

static void TaskDisplay(void *pvParameters) {
    (void) pvParameters;
    TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    for(;;) {
		
		// PORTC |= (1 << PC4);
		switch (sys_state) {
			// DEFAULT: menampilkan waktu
			case DEFAULT:
				dpdig00 = 0;
				dpdig01 = 0;
				dpdig03 = 0;
				dpdig05 = 0;
				display_segments_time();
			break;
			case PRE_DATE:
				display_segments_date();
			break;
			case DATE: // menampilkan tanggal
				display_segments_date();
			break;
			case POST_DATE: break;
			case PRE_CHANGE_TIME:
				dpdig02 = 0;
				dpdig04 = 0;
				hour_t = hour;
				minute_t = minute;
				second_t = second;
				display_segments_change_time();
			break;
			
			// Change Hour
			case CHANGE_TIME_h:
				dpdig04 = 1;
				display_segments_change_time();
			break;
			case CHANGE_TIME_COUNTER_h:
				display_segments_change_time();
			break;
			case POST_CHANGE_TIME_h:
				display_segments_change_time();
			break;
			
			// Change Minute
			case CHANGE_TIME_m:
				dpdig02 = 1;
				display_segments_change_time();
			break;
			case CHANGE_TIME_COUNTER_m:
				display_segments_change_time();
			break;
			case POST_CHANGE_TIME_m:
				display_segments_change_time();
			break;
			
			// Change Second
			case CHANGE_TIME_s:
				dpdig00 = 1;
				display_segments_change_time();
			break;
			case CHANGE_TIME_COUNTER_s:
				display_segments_change_time();
			break;
			case POST_CHANGE_TIME_s:
				display_segments_change_time();
			break;
			
			/////////////////////////////////////////////////
			case PRE_CHANGE_DATE:
				dpdig02 = 0;
				dpdig04 = 0;
				day_t = day;
				month_t = month;
				year_t = year;
				display_segments_change_date();
			break;
			
			// Change Day
			case CHANGE_DATE_D:
				dpdig04 = 1;
				display_segments_change_date();
			break;
			case CHANGE_DATE_COUNTER_D:
				display_segments_change_date();
			break;
			case POST_CHANGE_DATE_D:
				display_segments_change_date();
			break;
			
			// Change Month
			case CHANGE_DATE_M:
				dpdig02 = 1;
				display_segments_change_date();
			break;
			case CHANGE_DATE_COUNTER_M:
				display_segments_change_date();
			break;
			case POST_CHANGE_DATE_M:
				display_segments_change_date();
			break;
			
			// Change Year
			case CHANGE_DATE_Y:
				dpdig00 = 1;
				display_segments_change_date();
			break;
			case CHANGE_DATE_COUNTER_Y:
				display_segments_change_date();
			break;
			case POST_CHANGE_DATE_Y:
				display_segments_change_date();
			break;
			default: break;
		}
		// PORTC &= ~(1 << PC4);
		vTaskDelayUntil( &xLastWakeTime, ( 7 / portTICK_PERIOD_MS ));
    }
}

// interrupt penghitung waktu
ISR(TIMER1_COMPA_vect) {
	if (task_timer > 0) {
		task_timer--;
	}
	
	if (task_timer == 0) {
		second++;
		
		if (((sys_state == DEFAULT) || (sys_state == DATE)) && (second%2 == 0)) {
			dpdig02 = 1;
			dpdig04 = 1;
		}
		if (((sys_state == DEFAULT) || (sys_state == DATE)) && (second%2 == 1)) {
			dpdig02 = 0;
			dpdig04 = 0;
		}
		
		if (second > 59) {
			second = 0;
			minute++;
		}
		if (minute > 59) {
			minute = 0;
			hour++;
		}
		if (hour > 23) {
			hour = 0;
			day++;
		}
		if ((month == 2) && (((year%4 == 0) && (year%100 != 0)) || (year%400 == 0)) && (day > 29)) {
			day = 1;
			month++;
		}
		if ((month == 2) && ((year%4 != 0) || ((year%100 == 0) && (year%400 != 0))) && (day > 28)) {
			day = 1;
			month++;
		}
		if (((month == 1) || (month == 3) || (month == 5) || (month == 7) || (month == 8) || (month == 10) || (month == 12)) && (day > 31)) {
			day = 1;
			month++;
		}
		if (((month == 4) || (month == 6) || (month == 9) || (month == 11)) && (day > 30)) {
			day = 1;
			month++;
		}
		if (month > 12) {
			month = 1;
			year++;
		}
		task_timer = TASK_PERIOD_TIMER_MS;
	}
}

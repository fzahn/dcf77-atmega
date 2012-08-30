/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */


#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "ds1307.h"


uint32_t millis();
void processTelegram();
void userInterface();
void updateRTC();
uint8_t debounce(volatile uint8_t *port, uint8_t pin);

volatile uint32_t milliseconds=0;

volatile uint32_t risingEdge,lastComparison,newMinuteMark,buttonPressedTime;
uint32_t fallingEdge;
volatile uint8_t impulseCounter=0;
char Buffer[20];
char Zeit[15];

volatile struct {
    unsigned newMinute:1;
    unsigned newImpulse:1;
    unsigned unWritten:1;
    unsigned telegramHourParityOK:1;
    unsigned telegramMinuteParityOK:1;
    unsigned telegramDateParityOK:1;
} status;

uint8_t statemachine;

struct ds1307_time_t time_info,time_info_temp;


volatile int8_t counter;
uint8_t telegram[60];

uint8_t stunde,minute;

int main(void)
{   
    //Buttons on PA0-PA2
    DDRA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2));
    //internal pullup
    PORTA |= ((1<<PA0) | (1<<PA1) | (1<<PA2));
    //LED at PIN PB7
    DDRB |= (1 << PB7);
    //Blink LED for one second
    PORTB |= (1<<PB7);
    _delay_ms(1000);
    PORTB &= ~(1<<PB7);
    //PD3 (INT1) as input for DCF77 signal
    DDRD &= ~(1<<PD3);
    //Disable Pullup, external pullup used...
    PORTD &= (1<<PD3);
    
    //falling edge on PD4 to enable DCF-receiver
    DDRD |= (1<<PD4);
    PORTD |= (1<<PD4);
    _delay_ms(1000);
    PORTD &= ~(1<<PD4);
    
    //initialize LCD DISPLAY
    lcd_init(LCD_DISP_ON);
    lcd_puts("Funkuhr v1.0");
    _delay_ms(1000);
    lcd_clrscr();
    //enable interrupt on rising edge on INT 1
    GICR |= (1<<INT1);
    MCUCR |= ((1<<ISC11) | (1<<ISC10));
    
    //enable timer prescaler 64 -> 187 ticks per millisecond)
    TCCR2 |= (1<<WGM01);
    TCCR2 |= (1<<CS22);
    OCR2 = 249;
    TIMSK |= (1<<OCIE2);
    i2c_init();
    _delay_ms(500);
    ds1307_init(0);
    sei();
    status.newMinute=status.newImpulse=0;    
    //Main program
    for(;;){
        //new impulse detected in ISR....
        if (status.newImpulse==1) {
			//disable interrupt-handling, while in routine
            GICR &= ~(1<<INT1);
			//for 500ms check PIN every 10ms and check for high or low....
            if (millis()-risingEdge <=250) {
                    if (millis()-lastComparison >10) {
                        if ((PIND & (1<<PIND3))) {
                            impulseCounter++;
							//status LED: blink like signal
                            PORTB |= (1<<PB7);
                            lastComparison=millis();
                            if (status.newMinute==1) {
                                processTelegram();
                                status.newMinute = 0;
                            };
                         //if low detected, set falling-edgevar to millis()   
                        } else {
                            fallingEdge=millis();
                            lastComparison=millis();
							//status LED: blink like signal
                            PORTB &= ~(1<<PB7);
                            status.newMinute = 0;
                        }
                }
				//after received signal set unwritten to 1, to write it down in telegram-array
               } else {
                   status.newImpulse=0;
                   status.unWritten=1;
                   GICR |= (1<<INT1);

				   //reenable interrupt-handling
                }
        }
		//if pause between impulses is bigger than 1000ms: second 59, new minute will follow
        if (millis()-fallingEdge >1000) {
            status.newMinute = 1;
            counter=0;
        }
        //write bit from telegram in appropriate field (but not in second 59...)
		if ((status.unWritten==1)&& (status.newMinute==0)) {
            //lcd_puts(Buffer);
            lcd_gotoxy(15,0);
			//telegram-length <140ms: 0
            if (impulseCounter<=14) {
                lcd_puts("0");
                telegram[counter++] = 0x00;

            }
			//telegram-length >140ms: 1
            if (impulseCounter>14) {
                lcd_puts("1");
                telegram[counter++] = 0x01;

            }
            status.unWritten=0;
        }
        
	//output time on LCD...	
        userInterface();   
    }
    return 0;   /* never reached */
}

//Interrupt on rising-edge, means beginning of new impulse
ISR(INT1_vect)
{   
    status.newImpulse=1;
    lastComparison=millis();
    risingEdge=millis();
    impulseCounter=0;
}


//Timer routine, counts milliseconds since program start
ISR(TIMER2_COMP_vect) {
    milliseconds++;
}

uint32_t millis() {
    
    return milliseconds;
}

//checks if telegram is valid and updates RTC-chip

void processTelegram() {
	//check parity of received signal
    uint8_t i=0;
    uint8_t parity=0;
    for (i=21; i<28; i++) {
        if (telegram[i]==1) parity++;
    }
    if (parity%2 == telegram[28]) {
        status.telegramMinuteParityOK = 1;
  
    } else {
        status.telegramMinuteParityOK = 0;
    }
    parity=0;
    for (i=29; i<35; i++) {
        if (telegram[i]==1) parity++;
    }   
    if (parity%2 == telegram[35]) {
        status.telegramHourParityOK = 1;
    } else {
        status.telegramHourParityOK = 0;
    }
    time_info.seconds = 0;
    parity=0;
    for (i=36; i<58; i++) {
        if (telegram[i]==1) parity++;
    }   
    if (parity%2 == telegram[58]) {
        status.telegramDateParityOK = 1;
    } else {
        status.telegramDateParityOK = 0;
    }
	//if parity-checks are okay, write received info in time_info
    if ((status.telegramMinuteParityOK==1)&&(status.telegramHourParityOK==1)&&(status.telegramDateParityOK==1)) {
		//copy old time_info to time_info_temp
		time_info_temp.minutes=time_info.minutes;
		time_info_temp.hours=time_info.hours;
		time_info_temp.date=time_info.date;
		time_info_temp.day=time_info.day;
		time_info_temp.month=time_info.month;
		time_info_temp.year=time_info.year;
		//update information in time_info
        time_info.minutes = ((telegram[27]<<2) | (telegram[26] <<1) | (telegram[25] <<0)) * 10;
        time_info.minutes = time_info.minutes + ((telegram[24] << 3) | (telegram[23] << 2) |(telegram[22] << 1) | (telegram[21] << 0)); 
        time_info.hours = ((telegram[34]<<1) | (telegram[33]<<0))*10;
        time_info.hours = time_info.hours + ((telegram[32]<<3) | (telegram[31]<<2) | (telegram[30]<<1) | (telegram[29]<<0));    
        time_info.date = ((telegram[41]<<1) | (telegram[40] <<0)) * 10;
        time_info.date = time_info.date + ((telegram[39] <<3) | (telegram[38] <<2) | (telegram[37] <<1) | (telegram[36] <<0));
        time_info.day = ((telegram[44] <<2) | (telegram[43] <<1) | (telegram[42]<<0));
        time_info.month = (telegram[49]<<0) * 10;
        time_info.month = time_info.month + ((telegram[48] <<3) | (telegram[47] <<2) | (telegram[46] <<1) | (telegram[45] <<0));
        time_info.year = ((telegram[57] <<3) | (telegram[56] <<2) | (telegram[55] <<1) | (telegram[54] <<0)) * 10;
        time_info.year = time_info.year + ((telegram[53] <<3) | (telegram[52] <<2) | (telegram[51] <<1) | (telegram[50] <<0));			   
    }
	//simple validity check (values in ranges, that make sense)
    if ((time_info.hours < 24)&&(time_info.minutes <60)&&(time_info.date<32)&&(time_info.month<13)&&(time_info.day<8)&&(time_info.year<100)) {
       //compare last received time_info with saved one... update only, when last received is saved+1min
	   //will not work at midnight and in minute 59, but who cares?
	   if ((time_info.hours==time_info_temp.hours)&&(time_info.date==time_info_temp.date)&&(time_info.day==time_info_temp.day)&&(time_info.month==time_info_temp.month)&&(time_info.year==time_info_temp.year)&&(time_info.minutes==(time_info_temp.minutes+1))) {
           //enter code to update RTC
           updateRTC(); //place holder....
	   }
        
    }
}

//update RTC via I2C
void updateRTC() {
    //displayTime();
    ds1307_write(&time_info);
    
}

void userInterface() {
    //go to default, if no button pressed for 8 seconds
    if ((millis()-buttonPressedTime > 8000)) {
        statemachine=0;
        lcd_clrscr();
    }
    switch (statemachine) {
        case 0:
            //DisplayTime
            buttonPressedTime=millis();
            lcd_gotoxy(0,0);
            strcpy(Zeit,get_time_string());
            lcd_puts(Zeit);
            lcd_gotoxy(0,1);
            strcpy(Zeit,get_date_string());
            lcd_puts(Zeit);
            if (debounce(&PINA,PA0)!=0) {
                lcd_clrscr();
                buttonPressedTime=millis();
                statemachine=1; 
            }  
            break;
        case 1:
            lcd_gotoxy(0,0);
            lcd_puts("Set Time");
            if (debounce(&PINA,PA0)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                statemachine=2;
            }
            if (debounce(&PINA,PA1)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                stunde=minute=0;
                statemachine=10;
                
            }
            break;
        case 2:
            lcd_gotoxy(0,0);
            lcd_puts("Menu End");
            if (debounce(&PINA,PA0)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                statemachine=1;
            }
            break;
        case 10:
            lcd_gotoxy(0,0);
            lcd_puts("Stunde:");
            lcd_gotoxy(0,1);
            itoa(stunde,Buffer,10);
            lcd_puts(Buffer);
            if (debounce(&PINA,PA1)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                stunde++;
            }
            if (debounce(&PINA,PA2)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                stunde--;
            }
            if (debounce(&PINA,PA0)) {
                lcd_clrscr();
                buttonPressedTime=millis();
                lcd_gotoxy(0,0);
                statemachine = 255;
            }
            break;
        case 255:
            lcd_gotoxy(0,0);
            lcd_puts("not implemented");
            _delay_ms(1000);
            lcd_clrscr();
            statemachine=0;
            break;
        default:
            break;
    }

        

}

/* Einfache Funktion zum Entprellen eines Tasters */
uint8_t debounce(volatile uint8_t *port, uint8_t pin)
{
    if ( !(*port & (1 << pin)) )
    {
        /* Pin wurde auf Masse gezogen, 100ms warten   */
        _delay_ms(50);   // Maximalwert des Parameters an _delay_ms 
        _delay_ms(50);   // beachten, vgl. Dokumentation der avr-libc
        if ( *port & (1 << pin) )
        {
            /* Anwender Zeit zum Loslassen des Tasters geben */
            _delay_ms(50);
            _delay_ms(50); 
            return 1;
        }
    }
    return 0;
}


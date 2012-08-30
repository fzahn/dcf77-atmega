/*
 * DS1307 RTC driver for an ATMEGA328 (really any AVR with TWI hardware).
 *
 * (C)opyright 2010, 2011 Peter Gammie, peteg42 at gmail dot com. All rights reserved.
 * Commenced September 2010.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *i2c_rep_start
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changed to work with Peter Fleury's I2C Library
 * Added string functions for date and time for easy LCD and UART handling
 * (C)opyright 2011 Mosci.to
 */

#ifndef _ds1307_H_
#define _ds1307_H_

#include <stdbool.h>

#include "i2cmaster.h"
#include <string.h>

/* **************************************** */

/* DS1307-specifics: twi address 0b1101000. Note: shifted left 1. */
#define DS1307_ADDR  0xD0

/* reg0: Turns the clock oscillator on/off. */
#define CLOCK_HALT   7

/* reg7: Square-wave output. */
#define SQW_CONTROL_REG 0x07

#define SQW_OUT         7
#define SQW_SQWE        4
#define SQW_RS1_RS0_1Hz 0x0

/* reg2: if TWELVE_HOUR
 *         then 12hr mode with AMPM indictor,
 *         otherwise 24hr mode. */
#define AMPM         _BV(5)
#define TWELVE_HOUR  _BV(6)

/* Unsigned 8-bit BCD operations. */
#define fromBCD(x) (((x) >> 4) * 10 + ((x) & 0xF))
#define toBCD(x)   ((((x) / 10) << 4) | ((x) % 10))

/* FIXME add bit widths */
/*
struct ds1307_time_t {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
};
*/

struct ds1307_time_t {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
};


/* **************************************** */

static inline bool ds1307_read(struct ds1307_time_t *time_data)
{
    uint8_t data;
    
    /* Tell the DS1307 we want to read starting at address 0. */
    i2c_start_wait(DS1307_ADDR+I2C_WRITE);
    i2c_write(0x00);
    /* Commence the read. */
    i2c_rep_start(DS1307_ADDR+I2C_READ);
    
    data=i2c_readAck();
    time_data->seconds =  fromBCD(data & ~_BV(CLOCK_HALT));
    
    data=i2c_readAck();
    time_data->minutes = fromBCD(data);
    
    /* Read the hours register. */
    data=i2c_readAck();
    if(data & TWELVE_HOUR) {
        uint8_t hours = fromBCD(data & 0x1F);
        if(data & AMPM) {
            hours += 12;
        }
        time_data->hours = hours;
    } else {
        time_data->hours = fromBCD(data);
    }
    
    data=i2c_readAck();
    time_data->day = fromBCD(data);
    
    data=i2c_readAck();
    time_data->date = fromBCD(data);
    
    data=i2c_readAck();
    time_data->month = fromBCD(data);
    
    /* Last read, send NACK. */
    data=i2c_readNak();
    time_data->year = fromBCD(data);
    
    i2c_stop();
    return true;
    
}

/* FIXME Assumes the time is sane and is a bit hardwired. */
static inline bool ds1307_write(struct ds1307_time_t *time_data)
{
    
    /* Tell the DS1307 we want to start writing at address 0. */
    i2c_start_wait(DS1307_ADDR+I2C_WRITE);
    i2c_write(0x00);
    /* Keep the oscillator running. */
    i2c_write(toBCD(time_data->seconds) & ~_BV(CLOCK_HALT));
    i2c_write(toBCD(time_data->minutes));
    /* FIXME 24hr time. */
    i2c_write(toBCD(time_data->hours));
    i2c_write(toBCD(time_data->day));
    i2c_write(toBCD(time_data->date));
    i2c_write(toBCD(time_data->month));
    i2c_write(toBCD(time_data->year));
    
    i2c_stop();
    return true;
    
}

/* Initialise the DS11307. Assumes the TWI interface is already initialised. */
static inline bool ds1307_init(bool interrupts)
{
    /* Tell the DS1307 to start the oscillator (turn off CLOCK HALT) in
     * case it has lost power. */
    uint8_t reg0;
    
    /* Address, say read from memory slot 0, then read data. */
    i2c_start_wait(DS1307_ADDR+I2C_WRITE);
    i2c_write(0x0);
    i2c_rep_start(DS1307_ADDR+I2C_READ);
    reg0=i2c_readNak();
    
    /* Turn off the clock halt if necessary. */
    if(reg0 & _BV(CLOCK_HALT)) {
        i2c_start_wait(DS1307_ADDR+I2C_WRITE);
        i2c_write(0x0);
        i2c_write(reg0 & ~_BV(CLOCK_HALT));
    }
    
    /* Fire up the 1Hz interrupt. */
    if(interrupts) {
        i2c_start_wait(DS1307_ADDR+I2C_WRITE);
        i2c_write(SQW_CONTROL_REG);
        i2c_write(_BV(SQW_SQWE) | SQW_RS1_RS0_1Hz);
    }
    
    i2c_stop();
    
    return true;
    
}


void set_time_string(char *time_str2)
{
	/* Function will use a formatted string containing the current time to set RTC clock 
     Format: hh:mm:ss
     */
	
	char time_str1 [3];																					// Temporary string array
	struct ds1307_time_t t;																			// Declare structure "t" as a copy of structure ds1307_time_t
	
	ds1307_read(&t);																						// Read DS1307 and put data in "t struct" so we can write back the date
	memset( time_str1, 0, sizeof( time_str1));									// Empty the temporary string
	time_str1[0]=time_str2[0];																	// Copy first character from source string to temporary string
	time_str1[1]=time_str2[1];																	// Copy second character from source string to temporary string
	t.hours=atoi (time_str1);																		// Convert temporary string to integer value and store in t structure
	time_str1[0]=time_str2[3];																	//
	time_str1[1]=time_str2[4];																	// ... same for the rest of the string
	t.minutes=atoi (time_str1);																	//
	time_str1[0]=time_str2[6];																	//
	time_str1[1]=time_str2[7];																	//
	t.seconds=atoi (time_str1);																	//
	ds1307_write(&t);																						// Write structure to RTC
    
	return;																											// Return
}

void set_date_string(char *date_str2)
{
	/* Function will use a formatted string containing the current time to set RTC clock 
     Format: dd.mm.yyyy
     */
	
	char date_str1 [3];																					// Temporary string array
	struct ds1307_time_t t;																			// Declare structure "t" as a copy of structure ds1307_time_t
	
	ds1307_read(&t);																						// Read DS1307 and put data in "t struct" so we can write back the time
	memset( date_str1, 0, sizeof( date_str1));									// Empty the string
	date_str1[0]=date_str2[0];																	// Copy first character from source string to temporary string
	date_str1[1]=date_str2[1];																	// Copy second character from source string to temporary string
	t.date=atoi (date_str1);																		// Convert temporary string to integer value and store in t structure
	date_str1[0]=date_str2[3];																	//	
	date_str1[1]=date_str2[4];																	// ... same for the rest of the string
	t.month=atoi (date_str1);																		//
	date_str1[0]=date_str2[8];																	//
	date_str1[1]=date_str2[9];																	//
	t.year=atoi (date_str1);																		//
	ds1307_write(&t);																						// Write structure to RTC
    
	return;																											// Return
}


char* get_time_string(void)
{
	/* Function will return a formatted string containing the current time for easy use on LCD and UART 
     Format: hh:mm:ss
     */
	
	char time_str1 [3];																					// Temporary string array
	static char time_str2 [11];																	// Needs to be static to survive function exit
	struct ds1307_time_t t;																			// Declare structure "t" as a copy of structure ds1307_time_t
	
	memset( time_str2, 0, sizeof( time_str2));									// Empty the string to avoid appending
	
	ds1307_read(&t);																						// Read DS1307 and put data in "t struct"
	if (t.hours<10) strcat (time_str2,"0");										// Add leading zero if required
	itoa (t.hours,time_str1,10);																// Convert hours into ASCII
	strncat (time_str2,time_str1,2);														// Append hours string to output string
	strcat (time_str2,":");																			// Append ":"
	if (t.minutes<10) strcat (time_str2,"0");									// Add leading zero if required
	itoa (t.minutes,time_str1,10);															// Convert minutes to ASCII
	strncat (time_str2,time_str1,2);														// Append minutes string to output string
	strcat (time_str2,":");																			// Append ":"
	if (t.seconds<10) strcat (time_str2,"0");									// Add leading zero if required
	itoa (t.seconds,time_str1,10);															// Convert seconds to ASCII
	strncat (time_str2,time_str1,2);														// Append seconds string to output string
    
	return time_str2;																						// Return output string
}


char* get_date_string(void)
{
	/* Function will return a formatted string containing the current date for easy use on LCD and UART 
     Format: dd.mm.yyyy
     */
	
	char date_str1 [3];																					// Temporary string array
	static char date_str2 [11];																	// Needs to be static to survive function exit
	struct ds1307_time_t t;																			// Declare structure "t" as a copy of structure ds1307_time_t
	
	memset( date_str2, 0, sizeof( date_str2));									// Empty the string to avoid appending
	
	ds1307_read(&t);																						// Read DS1307 and put data in "t struct"
	if (t.date<10) strcat (date_str2,"0");											// Add leading zero if required
	itoa (t.date,date_str1,10);																// Convert date into ASCII
	strncat (date_str2,date_str1,2);														// Append date string to output string
	strcat (date_str2,".");																			// Append "."
	if (t.month<10) strcat (date_str2,"0");										// Add leading zero if required
	itoa (t.month,date_str1,10);																// Convert month to ASCII
	strncat (date_str2,date_str1,2);														// Append month string to output string
	strcat (date_str2,".20");																		// Append ".20"
	if (t.year<10) strcat (date_str2,"0");											// Add leading zero if required
	itoa (t.year,date_str1,10);																// Convert year to ASCII
	strncat (date_str2,date_str1,2);														// Append year string to output string
    
	return date_str2;																						// Return output string
}

#endif /* _ds1307_H_ */
/*
 * I2CS.c
 *
 * I2C client
 * 
 * This software is covered by a modified MIT License, see paragraphs 4 and 5
 *
 * Copyright (c) 2019 Dieter Reinhardt
 * Copyright (c) 2021 m5evt
 *
 * 1. Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * 2. The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * 3. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * 4. This software is strictly NOT intended for safety-critical or life-critical applications.
 * If the user intends to use this software or parts thereof for such purpose additional
 * certification is required.
 *
 * 5. Parts of this software are adapted from Microchip I2C client driver sample code.
 * Additional license restrictions from Microchip may apply.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include "I2CS.h"
#include "port.h"

static volatile uint8_t	read_addr;				// address received from controller
volatile uint8_t  timeout_cnt;				// 1ms timeout tick counter
volatile uint8_t	num_bytes = 0;		// number of bytes sent/received in transaction

// Timer for read operations 
void I2c_timeout_counter_inc(void) {
  timeout_cnt++;
}

// Initialize I2C interface
void I2CS_init(void) {
	PORTB_set_pin_dir(0, PORT_DIR_OUT);							// SCL = PB0
	PORTB_set_pin_level(0, false);
	PORTB_set_pin_pull_mode(0, PORT_PULL_OFF);
	PORTB_pin_set_inverted(0, false);
	PORTB_pin_set_isc(0, PORT_ISC_INTDISABLE_gc);
	PORTB_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PB1
	PORTB_set_pin_level(1, false);
	PORTB_set_pin_pull_mode(1, PORT_PULL_OFF);
	PORTB_pin_set_inverted(1, false);
	PORTB_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);

	TWI0.CTRLA = 1 << TWI_FMPEN_bp								// FM Plus Enable: disabled
	             | TWI_SDAHOLD_50NS_gc					  // Typical 50ns hold time
	             | TWI_SDASETUP_8CYC_gc;				  // SDA setup time is 8 clock cycles

	TWI0.SADDR = CLIENT_ADDRESS << TWI_ADDRMASK_gp  // client Address (8 bit address, i. e. bit 0 = 0, will be substituted by R/W bit)
	             | 0 << TWI_ADDREN_bp;							// General Call Recognition Enable: disabled

	TWI0.SCTRLA = 1 << TWI_APIEN_bp								// Address/Stop Interrupt Enable: enabled
	              | 1 << TWI_DIEN_bp							// Data Interrupt Enable: enabled
	              | 1 << TWI_ENABLE_bp						// Enable TWI client: enabled
	              | 1 << TWI_PIEN_bp							// Stop Interrupt Enable: enabled
	              | 0 << TWI_PMEN_bp							// Promiscuous Mode Enable: disabled
	              | 0 << TWI_SMEN_bp;							// Smart Mode Enable: disabled
  read_addr = 0;
  timeout_cnt = 0;
}

static uint8_t (*i2c_read_callback)(uint8_t, uint8_t);
static uint8_t (*i2c_write_callback)(uint8_t, uint8_t);

void I2c_set_callbacks(uint8_t (*read)(uint8_t, uint8_t),
                       uint8_t (*write)(uint8_t, uint8_t)) {
  i2c_read_callback = read;
  i2c_write_callback = write;
}

// send ACK
void I2c_ack(void) {
  TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;	 
}

// send NACK	
void I2c_nack(void) {
  TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_RESPONSE_gc; 
}

// error handler, should reset I2C client internal state	
static void I2C_error_handler(void) {
	TWI0.SSTATUS |= (TWI_APIF_bm | TWI_DIF_bm);					// clear interrupt flags

	PORTB_set_pin_dir(0, PORT_DIR_IN);							// SCL = PB0
	PORTB_set_pin_dir(1, PORT_DIR_IN);							// SDA = PB1
	TWI0.SCTRLA = 0;											// disable client
	_delay_us(10);												// SCL, SDA tristated high
	PORTB_set_pin_dir(0, PORT_DIR_OUT);							// SCL = PB0
	PORTB_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PB1
	// re-enable client
	TWI0.SCTRLA = 1 << TWI_APIEN_bp								// Address/Stop Interrupt Enable: enabled
				| 1 << TWI_DIEN_bp								// Data Interrupt Enable: enabled
				| 1 << TWI_ENABLE_bp							// Enable TWI client: enabled
				| 1 << TWI_PIEN_bp								// Stop Interrupt Enable: enabled
				| 0 << TWI_PMEN_bp								// Promiscuous Mode Enable: disabled
				| 0 << TWI_SMEN_bp;								// Smart Mode Enable: disabled
  I2c_nack();
}

void I2cRead(void) {
  timeout_cnt = 0;									
  // wait until Clock Hold flag set
  while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {
    if (timeout_cnt > 2) return;					
  }
  if (i2c_read_callback) TWI0.SDATA = i2c_read_callback(read_addr, num_bytes);
	num_bytes++;									
}

void I2cWrite(void) {
  // controller wishes to write to client
	if (num_bytes == 0) read_addr = TWI0.SDATA;
  if (num_bytes > 0) {
	   uint8_t data = TWI0.SDATA;
	  if (i2c_write_callback) TWI0.SDATA = i2c_write_callback(read_addr, data);
  }
  num_bytes++;									
  if (num_bytes > MAX_TRANSACTION) {		
    I2c_nack();
  }	
  else {
    I2c_ack();
  }			
}

// I2C IRQ handler
// WARNING - do NOT clear interrupt flags during transaction, this will lead to false data when controller reads
ISR(TWI0_TWIS_vect) {
  // APIF && DIF, invalid state 
	if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_DIF_bm)) { 
		I2C_error_handler();								 
		return;
	}
  // Collision - client has not been able to transmit a high data or NACK bit
	if (TWI0.SSTATUS & TWI_COLL_bm) {							
		I2C_error_handler();	
		return;
	}
  // Bus Error - illegal bus condition
	if (TWI0.SSTATUS & TWI_BUSERR_bm) {							
		I2C_error_handler();				
		return;
	}

  // APIF && AP - valid address has been received
	if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_AP_bm)) { 
    num_bytes = 0;									
    I2c_ack();
	  // controller wishes to read from client (else return and wait for data from controller)
    if (TWI0.SSTATUS & TWI_DIR_bm) {							
      I2cRead();
      I2c_ack();
	 	}
		return;
	}
  // DIF - Data Interrupt Flag - client byte transmit or receive completed
	if (TWI0.SSTATUS & TWI_DIF_bm) {							
    // controller wishes to read from client
		if (TWI0.SSTATUS & TWI_DIR_bm) {						
			if (!(TWI0.SSTATUS & TWI_RXACK_bm)) {				
        I2cRead();
        I2c_ack();
			} 
      else {										
        // Received NACK from controller 
        // Reset module
				TWI0.SSTATUS |= (TWI_DIF_bm | TWI_APIF_bm);		
				TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;				
			}
		} 
    else {												
      I2cWrite();
		}
		return;
	}
	// APIF && !AP - Stop has been received
	if ((TWI0.SSTATUS & TWI_APIF_bm) && (!(TWI0.SSTATUS & TWI_AP_bm))) { 
		TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
		return;
	}
}


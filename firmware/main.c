/*
 *
 * This software is covered by a modified MIT License, see paragraphs 4
 *
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
 * 4. Parts of this software are adapted from Microchip sample code.
 * Additional license restrictions from Microchip may apply.
 */

#define PERIOD_EXAMPLE_VALUE (0x1388)

#include <avr/io.h>
#include <avr/interrupt.h>

#include "helper.h"
#include "I2CS.h"
#include "hl2_extpamon.h"

// Globals
volatile uint8_t temperature_raw;
volatile uint8_t current_raw;
volatile uint8_t status_reg;
volatile uint8_t status_reg_b;
volatile uint8_t do_adc_flag;

void TCA0_init(void) {
    // enable overflow interrupt 
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    // set Normal mode 
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
    // disable event counting 
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
    // set the period 
    TCA0.SINGLE.PER = PERIOD_EXAMPLE_VALUE;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
}

void RTC_init(void) {
  //Wait for all register to be synchronized 
  while (RTC.STATUS > 0) {};
  
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc   // 1 
            | 1 << RTC_RTCEN_bp     // Enable: enabled 
            | 0 << RTC_RUNSTDBY_bp; // Run In Standby: disabled
  // Wait for all register to be synchronized 
  while (RTC.PITSTATUS > 0) {} 
  RTC.PITCTRLA = RTC_PERIOD_CYC32_gc  // RTC Clock  Cycles 32 
               | 1 << RTC_PITEN_bp; // Enable: enabled 
 
  RTC.PITINTCTRL = 1 << RTC_PI_bp; //Periodic Interrupt: enabled 
}

void PORT_init(void) {
  // set pin 0 of PORT A as output
  PORTA.DIR |= PIN4_bm;
  PORTA.DIR |= PIN3_bm;
  PORTA.DIR |= PIN1_bm;
  PORTA.DIR |= PIN2_bm;
}

void ADC0_init(void) {
  // Disable digital input buffer
  PORTA.PIN6CTRL &= ~PORT_ISC_gm;
  PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN5CTRL &= ~PORT_ISC_gm;
  PORTA.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;

  // Disable pull-up resistor 
  PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;
  PORTA.PIN5CTRL &= ~PORT_PULLUPEN_bm;

  ADC0.CTRLC = ADC_PRESC_DIV4_gc
             | ADC_REFSEL_VDDREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm
             | ADC_RESSEL_10BIT_gc;
}

uint16_t ADC0_read(uint8_t muxpos) {
  // Select analog input
  ADC0.MUXPOS = (muxpos<<0); 
   //Start conversion
  ADC0.COMMAND = ADC_STCONV_bm;
  // Wait til done
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {;}
  //Done, clear INT
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  // 12 bit result
  return ADC0.RES;
}

// Send data to the HL2
uint8_t i2cGetData(uint8_t read_addr, uint8_t this_byte){
  switch (read_addr) {
    case FLAGS_STATUS:
      return status_reg_b;
      break;
    case MRF101_STATUS:
      switch (this_byte) {
        case 0:
          return temperature_raw;
        case 1:
          return current_raw;
        case 2:
          return status_reg_b;
        case 3:
          return status_reg;
        default: break;
      break;
    }
  }
  return 0x1f;
}

void ChangeIOState(void) {
  IOPORT = status_reg_b;
}
// Read byte sent from HL2
uint8_t i2cWriteData(uint8_t write_addr, uint8_t data) {
  switch (write_addr) {
    case SET_CFG:
      status_reg_b = data;
      ChangeIOState();
      break;
    case SET_CURRENT_TRIP:
      // current * 10, e.g. 4 A = 40
      break;
    case SET_TEMP_TRIP:
      //PORTA.OUTTGL = PIN1_bm;
      // temp -> 26 deg C = 26
      break;
    default: break;
  }
  return status_reg_b;
}

void SetupProtectedWrites(void) {
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, !CLKCTRL_CLKOUT_bm | CLKCTRL_CLKSEL_OSC20M_gc);
    _PROTECTED_WRITE(CLKCTRL.OSC32KCTRLA, CLKCTRL_RUNSTDBY_bp);
}

void InitUc(void) {
    SetupProtectedWrites();
    PORT_init();
    TCA0_init();
    RTC_init();
    ADC0_init();
    I2CS_init();
}

void InitStatusFlags(void) {
    // Fans off, ADC mux 0, CN8 disabled, PWR disabled 
    status_reg_b = 0x00;
    ChangeIOState();
}

int main(void) {
    // Setup micro specifics 
    InitUc();
    // I2C callbacks to application code
    I2c_set_callbacks(i2cGetData, i2cWriteData);
    // Setup application code
    InitStatusFlags();
    temperature_raw = 0;
    current_raw = 0;
    
    PORTA.OUTCLR = PIN1_bm;
    status_reg = 0;
    // enable global interrupts */
    sei();

    temperature_raw = ADC0_read(TEMPERATURE);

    for (;;) {
       if (do_adc_flag) {
         if ((status_reg & ADCSAMPLE_M) == ADCSAMPLE_M) {
           temperature_raw = ADC0_read(TEMPERATURE);
           CBI(status_reg, 0);      
         } 
         else {
           uint16_t temp = ADC0_read(CURRENT);
           current_raw = (temp >> 2) & 0xFF; 
           SBI(status_reg, 0);      
         }
       }
       do_adc_flag = 0;
    }
}

// ADC sampling timer
ISR(TCA0_OVF_vect) {
    do_adc_flag = 1;
    // Clear flag 
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// 1 ms tick 
ISR(RTC_PIT_vect) {					
    I2c_timeout_counter_inc();
    // Clear flag
  	RTC.PITINTFLAGS = RTC_PI_bm;		
}

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

#define TEN_MS_PERIOD (0x1388)

#include <avr/io.h>
#include <avr/interrupt.h>

#include "helper.h"
#include "I2CS.h"
#include "hl2_extpamon.h"

// Globals
volatile uint8_t temperature_raw;
volatile uint8_t current_raw;
volatile uint8_t status_reg;
volatile uint8_t fan_counter;
volatile uint8_t trip_current;
volatile uint8_t trip_temperature;


void TCA0_init(void) {
    // enable overflow interrupt 
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    // set Normal mode 
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
    // disable event counting 
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
    // set the period 
    TCA0.SINGLE.PER = TEN_MS_PERIOD;
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

uint16_t ADC0_read(const uint8_t muxpos) {
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
uint8_t i2cGetData(const uint8_t read_addr, const uint8_t this_byte){
  switch (read_addr) {
    case FLAGS_STATUS:
      return IOPORT;
      break;
    case MRF101_STATUS:
      switch (this_byte) {
        case 0:
          return temperature_raw;
        case 1:
          return current_raw;
        case 2:
          return IOPORT;
        case 3:
          return status_reg;
        default: break;
      break;
    }
  }
  return 0x1f;
}

// Read byte sent from HL2
uint8_t i2cWriteData(const uint8_t write_addr, const uint8_t data) {
  switch (write_addr) {
    case SET_CFG:
      IOPORT = data;
      break;
    case SET_CURRENT_TRIP:
      trip_current = data;
      break;
    case SET_TEMP_TRIP:
      trip_temperature = data;
      break;
    default: break;
  }
  return IOPORT;
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
    IOPORT = 0x00;
}

void GetTemperature(void) {
  uint16_t temp_raw = ADC0_read(TEMPERATURE);
  float this_temperature = ((float)temp_raw * (3.3 / 1024)) * 100;
            
  //temperature_raw = (uint8_t)(((float)temp_raw * (3.3 / 1024)) * 100);
  temperature_raw = (uint8_t)this_temperature;
  // Every 4080 ms sample the temperature to decide
  // if; fan turns on, fan stays off, fan turns off (after being on)
  if (fan_counter == 0) {
    if (temperature_raw > FAN_TEMP_THRESHOLD) {
      FAN1 = 1;
    }
    else {
      FAN1 = 0;
    }
    fan_counter++;
  } 
  else if (fan_counter == 255) {
    fan_counter = 0;
  }
  else if (fan_counter > 0) {
    fan_counter++;
  }
  if (temperature_raw > trip_temperature) {
    ENPWR = 0;
  }
  CBI(status_reg, ADCSAMPLE_M);      
}

void GetCurrent(void) {
  uint16_t temp = ADC0_read(CURRENT);
  // Convert ADC bits to real value e.g. 4.00 A 
  float this_c = ((float)temp * (3.3 /1024) / 20) / CURRENT_SENSE_R;
  // To fit into an 8 bit reg, measured value to full scale
  current_raw = (uint8_t)((this_c / CURRENT_FULL_SCALE) * 255);
  if (current_raw > trip_current) {
    ENPWR = 0;
  }
  SBI(status_reg, 0);      
}

int main(void) {
    // Setup micro specifics 
    InitUc();
    // I2C callbacks to application code
    I2c_set_callbacks(i2cGetData, i2cWriteData);
    // Setup application code
    InitStatusFlags();
   
    // Init globals
    temperature_raw = 0;
    current_raw = 0;
    status_reg = 0;
    fan_counter = 0;
    trip_current = 178;
    trip_temperature = 45;

    // enable global interrupts */
    sei();
    
    temperature_raw = ADC0_read(TEMPERATURE);

    for (;;) {
       if ((status_reg & DO_ADC_M) == DO_ADC_M) {
     PORTA.OUTTGL = PIN1_bm;
         if ((status_reg & ADCSAMPLE_M) == ADCSAMPLE_M) {
           PORTA.OUTSET = PIN1_bm;
           GetTemperature();
           PORTA.OUTCLR = PIN1_bm;
         } 
         else {
           GetCurrent();
         }
         CBI(status_reg, 1);
       }
    }
}

// ADC sampling timer (8 ms tick)
ISR(TCA0_OVF_vect) {
    // Set ADC sample flag
    SBI(status_reg, 1);
    // Clear flag 
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// 1 ms tick 
ISR(RTC_PIT_vect) {					
    I2c_timeout_counter_inc();
    // Clear flag
  	RTC.PITINTFLAGS = RTC_PI_bm;		
}

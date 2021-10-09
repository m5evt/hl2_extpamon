/*
 * main.c 
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

// i2c memory map - data to send to HL2
#define FLAGS_STATUS       0x00 
#define MRF101_STATUS      0x01
// i2c memory map - config sent by HL2
#define SET_CFG            0x02
#define SET_CURRENT_TRIP   0x03
#define SET_TEMP_TRIP      0x04
// Outputs
#define IOPORT             PORTA_OUT
#define CN8                SBIT(PORTA_OUT, 1) 
#define ENPWR              SBIT(PORTA_OUT, 2) 
#define FAN1               SBIT(PORTA_OUT, 3) 
#define FAN2               SBIT(PORTA_OUT, 4) 
// Inputs
#define VIPA_FIL           SBIT(PORT_IN, 5) 
#define VTEMP_FIL          SBIT(PORT_IN, 6) 
// ADC channels
#define CURRENT            0x05
#define TEMPERATURE        0x06

#define STATUSFLAGS_M      0b00011110
#define ADCSAMPLE_M        0b00000001
#define FAN1_M             0b00000010
#define FAN2_M             0b00000100
#define CN8_M              0b00001000
#define ENPWR_M            0b00010000

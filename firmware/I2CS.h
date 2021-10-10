/*
 * I2CS.h
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
 * 5. Parts of this software are adapted from Microchip I2C slave driver sample code.
 * Additional license restrictions from Microchip may apply.
 */

#ifndef I2CS_H_
#define I2CS_H_

#define CLIENT_ADDRESS	0x2A		// 8 bit address, i.e. bit 0 = 0, will be substituted by R/W bit
//#define SLAVE_ADDRESS	0x55		// 8 bit address, i.e. bit 0 = 0, will be substituted by R/W bit
#define MAX_TRANSACTION 2		// maximum number of bytes sent/received in single transaction

void I2CS_init(void);	
void I2c_set_callbacks(uint8_t (*read)(uint8_t, uint8_t),
                       uint8_t (*write)(uint8_t, uint8_t));
void I2c_timeout_counter_inc(void);
#endif /* I2CS_H_ */

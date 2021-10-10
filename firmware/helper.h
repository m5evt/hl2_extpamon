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

#ifndef HELPER_h
#define HELPER_h

#include <avr/io.h>
// 			Access bits like variables:
struct bits {
  uint8_t b0:1;
  uint8_t b1:1;
  uint8_t b2:1;
  uint8_t b3:1;
  uint8_t b4:1;
  uint8_t b5:1;
  uint8_t b6:1;
  uint8_t b7:1;
} __attribute__((__packed__));

#define SBIT_(port,pin) ((*(volatile struct bits*)&port).b##pin)

#define	SBIT(x,y)	SBIT_(x,y)
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define SBI(var, mask)   ((var) |= (uint8_t)(1UL << mask))
#define CBI(var, mask)   ((var) &= (uint8_t)~(1UL << mask))

#define TRUE 1
#define FALSE 0

#endif

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

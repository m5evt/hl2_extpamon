# hl2_extpamon

## Overview

Power amplifier instrumentation board designed to couple to the Hermes-Lite2 SDR. 

The firmware/hardware is designed to couple to an LM35 temperature sensor.

The hardware should cope with upto 8 A of current to a power amplifer. However, at present the current is scaled to a full scale of 5 A. This can be changed in [hl2_extpamon.h](https://github.com/m5evt/hl2_extpamon/blob/main/firmware/hl2_extpamon.h). Other parameters such as automatic temperature turn on for the fan can be configured here.

The module uses an ATTINY414 device with an i2c address of 0x2A. 

## Reading data from device

Data can be read from the device over i2c by send the first data frame as 0x01.

For example, to read temperature and current from the device send:

```
07AA0100
```

This will return a 16 bit number comprised of the following bytes:

| Byte 3  | Byte 2 | Byte 1  | Byte 0 |
| ----------- | ----------- |----------- | ----------- |
| Device status      | PORTA status       |Current      | Temperature |

Temperature = e.g. 26 is 26 deg C

Current is scaled to a maximum current of 5 A represented in 255 bits. A return value of 128 = (128 / 255) * 5 = 2.51 A.

PORTA status corresponds to:

| Bit | Description |
|----------- | ----------- |
| 0 | Reserved |
| 1 | HL2 CN8 control |
| 2 | Enable PA power |
| 3 | FAN 1 |
| 4 | FAN 2 |
| 5 | N/a
| 6 | N/a
| 7 | Reserved |


Device status correspod to:

| Bit | Description |
|----------- | ----------- |
| 0 | Sampling ADC5 |
| 1 | Sampling ADC6 |


## Writing data to device

PORTA I/O can be set/clear by writing to address 0x02 in the device. For example, to enable the PA and turn on the fans send:

```
06AA021E
```

## Programming the ATTINY414

To program, the Makefile uses [pymcuprog](https://github.com/microchip-pic-avr-tools/pymcuprog) and UPDI. This does not require any specialist tools other than a TTL USB adapter and a resistor. See [here](https://github.com/mraardvark/pyupdi).

To program the device, from a command line run:
```
make clean && make erase && make program
```

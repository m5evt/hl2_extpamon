# hl2_extpamon
Power amplifier instrumentation board designed to couple to the Hermes-Lite2
SDR. 

The module uses an ATTINY414 device with an i2c address of 0x2A. 

Data can be read from the device over i2c by send the first data frame as 0x01.

For example, to read temperature and current from the device send:

07AA0100

This will return a 16 bit number comprised of the following bytes:

| Byte 3  | Byte 2 | Byte 1  | Byte 0 |
| ----------- | ----------- |----------- | ----------- |
| Device status      | PORTA status       |Current      | Temperature |

Temperature = e.g. 26 is 26 deg C

Current is scaled to a maximum current of 5 A represented in 255 bits. A return value of 128 = (128 / 255) * 5 = 2.51 A.

PORTA status corresponds to:

| Bit 0 | Reserved |
| Bit 1 | HL2 CN8 control |
| Bit 2 | Enable PA power |
| Bit 3 | FAN 1 |
| Bit 4 | FAN 2 |
| Bit 5 | N/a
| Bit 6 | N/a
| Bit 7 | Reserved |

Device status correspod to:

| Bit 0 | Sampling ADC5 |
| Bit 1 | Sampling ADC6 |

PORTA I/O can be set/clear by writing to address 0x02 in the device. For
example, to enable the PA and turn on the fans send:

06AA021E

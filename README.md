# NEWOPTO XYC-ALS21C-K1 Ambient Light Sensor

![breakout board](doc/xyc_als21c_k1.jpg)

This is a driver for the [NEWOPTO](http://www.xycgd.com/) XYC-ALS21C-K1 Ambient Light Sensor for the rt-thread operating system.

## Description

The XYC-ALS21C-K1 is

- digital
- visible light sensor
- I2C
- with an interrupt line
- the size of an 1206 resistor

The driver is written in c/c++. The driver is integer-only, no floating point is used.

## Breakout board

The [breakout board](http://oshwlab.com/koendv/xyc_als21c_k1) is assembled at jlcpcb.

## Reverse mounting

Notes about [reverse mounting the XYC-ALS21C-K1](https://github.com/koendv/xyc-als21c-k1/blob/main/REVERSE.md)

## Other os

The driver is easily adapter to other operating systems, like [Arduino](https://github.com/koendv/xyc-als21c-k1)

## See also

This driver may also be of interest for the following similar IC's:

- GOODTAKE [GT442-DALS-Z1](doc/GT442-DALS-Z1.pdf) / GT4421-DALS-Z1
- GOODTAKE [SDF-DALS-Z1-TT/TR](doc/SDF-DALS-Z1.pdf) (same footprint as VEML7700)
- [ALS-AK610P-DF](doc/ALS-AK610P-DF.pdf)
- [WH11867UF](doc/WH11867UF.pdf) 

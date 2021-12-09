I2C communication with HYB HPSD 303x and HPSD 8100
Calculation of calibrated temperature and pressure value from HYB digital pressure sensor (HPSD303x and HPSD8100) written in C++ for Arduino.

Introduction
This document describes basic I2C communication and pressure sensor signal calculation of HYB pressure sensor families HPSD330x and HPSD8100.
 	
General description

The digital pressure sensor consists of a silicon piezo-resistive sensor and a ΔΣ A/D sensor interface integrated circuit integrated on a ceramic substrate.
The sensor interface's main function is to convert the raw uncompensated analog output voltage from the pressure sensor bridge to 24-bit digital value.
The temperature of the sensor is converted to digital value as well. Both values are available via a digital interface. The sensor uses two stages,
a low noise analog front-end (AFE) and an Analog-to-Digital Converter (ADC). The AFE comprises of a low noise chopper amplifier and the ADC employs delta-sigma (ΔΣ) A/D conversion
technique with several oversampling ratio (OSR) options for optimization between speed, power consumption and resolution. The sensor input 1-sigma noise resolution is down
to 0.12 µVrms which can be further reduced with four digital low pass filtering user selectable options.
The sensor outputs raw measurement results and calibrated and temperature compensated results need to be calculated in the application host system using the sensor calibration data.
During the production, every transducer is individually calibrated at three temperatures and three pressures. Calibration is performed at seven different points resulting in
3 temperature and 7 pressure calibration coefficients. Values of the trimming and sensor calibration coefficients are stored in the internal 512-bit EEPROM. These coefficients 
must be read by the external microcontroller and used in the algorithm to convert the raw pressure and temperature readouts into compensated and calibrated temperature and pressure readouts. 
A serial bus compatible with a 4-wire or 3-wire SPI bus or 2-wire I2C bus is used for reading calibration coefficients, configuring conversion parameters, starting conversion 
and reading out the A/D conversion result. 
The measurements can be run either at forced mode (single command-based measurement) or at normal mode (automated measurements) at eight different output rates with user-selectable 
delays between measurements from 0.5ms up to 4000ms. An internal clock oscillator makes the external clock unnecessary.

Detailed operation principle
After connecting the power supply the sensor is reset by on-chip power on reset (POR) circuit and then set to standby mode. There are no limitations for the slope and sequence of 
rising of the power supply voltage. However, it is recommended to reset the device manually after every power up to make sure it is reset properly. This is accomplished via serial 
bus by writing any data byte to the Reset register. The POR or writing to the Reset register will reset all internal registers.

Digital interface
The sensor supports I2C and SPI digital interfaces. The I2C interface is a 2-wire serial bus that is selected by leaving the CSB pin unconnected (floating) or by connecting it to VDDIO. 
The CSB pin has an internal 250 kΩ pull-up resistor to VDDIO. 
The SPI interface supports 3-wire and 4- wire serial bus communication. Selection between 3- and 4-wire SPI bus modes is done by WIRE bit in the Configuration register (0xEE). 
After power up the WIRE=0 which selects the 4-wire SPI bus mode. To select 3-wire SPI bus mode it is necessary to first set WIRE=1 by writing to the Configuration register.
The SPI communication is selected by pulling the CSB pin low. It has an additional SPI mode lock in feature. By pulling CSB low and giving at least four SCK clock pulses makes the 
digital interface to lock into SPI communication mode. This is done in order to avoid inadvertently decoding SPI traffic to another slave device as I2C data. 
After entering SPI lock mode the I2C communication is possible only after applying power on reset.
All addresses presented in this document use 8-bit notation. The I2C bus device address consists of 7 address bits and 8th LSB bit which selects between write (LSB=0) and read (LSB=1) 
operation. Thus sensor device address for write is 0xEA and for read 0xEB. When the I2C bus is used the A7 MSB register address bit value is always one (A7=1).

Operating modes
Only two registers are needed for configuring and running the measurements. The Configuration register (I2C: 0xEE) contains only measurement configuration settings. 
The Control register (I2C: 0xEF) contains measurement and operating mode selection settings. Writing to the Control register starts the selected measurements.
Sensor has three selectable operating modes; sleep mode, forced mode and normal mode. The operating mode is selected by MODE bits in the Control register (I2C: 0xEF). 
In the sleep mode (MODE=00) the device does not perform any measurements and only a very small sleep current is drawn from the supplies.
In the forced mode (MODE=01, 10) selected measurements are run only once after which the device returns automatically to the sleep mode. Every new measurement requires 
writing a new forced mode command into the Control register. In the forced mode an internal clock oscillator is turned on only during the measurement. The forced mode 
is recommended in applications that use a low measurement rate or require host based synchronization of measurements.
In the normal mode (MODE=11) selected measurements are performed automatically in a loop at a selected rate until the sleep mode (MODE=00) is selected. The normal mode 
measurement cycle comprises of a measurement and a standby period. Selected measurements are performed during the measurement period. To minimize current consumption the device 
enters standby mode for the time between measurements during which only the internal clock oscillator is running. The standby mode time is defined by a DELAY bit setting in the 
Configuration register (I2C: 0xEE). There are eight delay settings available; 0.5ms, 62.5ms, 125ms, 250ms, 500ms, 1000ms, 2000ms and 4000ms. The normal mode measurement cycle is a 
sum of selected measurements’ A/D conversion time and the delay setting. The normal mode is recommended in applications 
where IIR filter is used like for filtering short-term sensor signal disturbances. 
Selection of temperature and pressure measurements and their resolution can be done independently using OSRT and OSRP oversampling ratio settings in the Control register (I2C: 0xEF). 
Each oversampling ratio has seven settings 1/4x, 1/2x, 1x, 2x, 4x, 8x and 16x in addition to no measurement setting. The highest value setting gives the highest resolution but it has 
the longest A/D conversion time and the highest power consumption. Similar way the lowest value setting gives the lowest resolution but it has the shortest A/D conversion time and the 
lowest power consumption. Thus the multiple oversampling ratio settings allow optimizing measurements between speed, power consumption and resolution.

Digital filtering
The Configuration register (I2C: 0xEE) has an additional selection for a digital infinite impulse response (IIR) type low pass filter option with four different filter coefficients 2, 4, 8 and 16.
The IIR low pass filter can be used to damp sudden variations in the sensor signal and to further improve noise resolution by additional filtering of the noise. The filter does not 
affect output data rate but only signal bandwidth and step response delay. When selected the filtering is applied to both temperature and pressure signals. The filtered temperature and pressure 
conversion results are stored into the Temperature result registers (I2C: 0xF4…0xF6) and the Pressure result registers (I2C: 0xF1…0xF3) respectively.

Measurement result
Pressure and temperature measurement results in the sensor are 24-bit unsigned numbers each of which are stored into three 8-bit result registers. The pressure result addresses are MSB byte (0xF1), LSB byte (0xF2) and XLSB byte (0xF3). The temperature result addresses are MSB byte (0xF4), LSB byte (0xF5) and XLSB byte (0xF6).
Sensor A/D conversion and result status can be monitored from the Status register (0xF0) which contains RDYT and RDYP flags to indicate when there are unread temperature (T) and/or pressure (P) 
measurement results available in the pressure and temperature result registers. The corresponding RDYx (x=T or P) flag is set (1) when a new measurement is ready for a 
read. Reading the measurement result will clear (0) the corresponding RDYx (x=T or P) flag.
The decision when to read measurement results can be made by polling the Status register and waiting until the flag(s) of selected measurement(s) (RDYT and/or RDYP flags) have been set high. 
This method can be used in both forced and normal modes. In forced mode, the other choice is to wait at least the maximum A/D conversion time before reading the result. In normal mode, it is also possible to read results at a rate when new results are expected to be ready. This is possible since sensor 
has an internal A/D conversion result memory buffer. If a new result is finished during read of result registers the new value is updated result registers only after the serial bus
communication has been released. Important note: In normal mode, the results must be always read using incremental read (all three bytes of each result or all six bytes of both results at a 
single read sequence) to maintain A/D conversion data consistency. The incremental read is recommended to be used also in the forced mode.
 


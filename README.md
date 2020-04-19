# ENVIRONMENTAL STATION
A prototype platform for measuring environmental values as temperature,
humidity, gas, etc ...
Based on Open Source tools for developping the embedded system.

## Development environment
### Hardware
- Nucleo Board
- Microcontroller : STM32F446RE
- Processor : Cortex M4
### Software
- HAL (Hardware Abstraction Layer) : libopencm3
- Embedded OS (Operation System) : FreeRTOS
- Toolchain : ARM with GCC, GDB and OpenOCD

## Embedded System
### Sensors
- BME680 : Temperature, humidity, pressure and gas (with i2c bus).
- SGP30 : Gas (with an equivalent level of CO2 with i2c bus).
- GA1A1S202WP : Luminosity (with analog pin).
### Communication
- HC05 : Bluetooth module with SPP (Serial Port Protocol).

The prototype sends data to a python script which receive and store sensor datas.

## Communication protocol
Protocol between embedded system and python scrip is a personal protocol.
- "D=2020-4-19 18:10:13,T=20,P=999,H=50,L=758,G=0\n"
Each frame must finish by an ASCII character '\n', used for delimiting each message frame.
Sensor datas have a specific operator as :
- Horodatage : 'D' (gather date and time)
- Temperature : 'T'
- Pressure : 'P'
- Humidity : 'H'
- Luminosity : 'L'
- Gas : 'G'

## Bugs
- Luminosity sensor has sent as "voltage values", so the python script computes data (refer to GA1A1S202WP datasheet) => pow operation does not working.
- SGP30 sensor does not working with the i2c BSP.

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
- BME680 : Temperature, humidity, pressure.
- LPS33W : Pressure and temperature.
- SGP30 : Gas (with an equivalent level of CO2).
- VEML7700 : Luminosity.
### Communication
- RN4871 : BLE module with GATT protocol.
- EINK : Display sensor data on the screen
The prototype sends data to a python script which receive and store sensor datas.

## Bugs
- SGP30 and VEML7700 sensor does not working with the i2c BSP.

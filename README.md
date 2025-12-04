# STM32 Automotive Gauge Project

An embedded automotive gauge built on the **STM32F407VET6** development board.  
The project reads multiple engine and environmental parameters and displays them on an SPI-driven LCD.  
Future goals include designing a **custom PCB** and a **vehicle-ready enclosure**.

---

## Features

- **STM32F407VET6** as main MCU  
- **SPI display** using ILI9341 for real-time gauge graphics  
- **MAX31856 thermocouple amplifier** for EGT using SPI **(TODO)**
- **AHT20** I2C temperature sensor for ambient temperature and humidity  
- **Analog oil temperature sensor** via ADC  
- **Analog oil pressure sensor** via ADC  

---

## Hardware Overview

### MCU
- STM32F407VET6 development board  

### Display
- SPI TFT display (ILI9341, GC9A01 in the future)  

### Sensors

#### MAX31856 Thermocouple Amplifier
- Interface: SPI  
- Purpose: EGT measurement  
- Outputs temperature data in digital format

#### AHT20 (Ambient Sensor)
- Interface: I2C  
- Measures ambient temperature and humidity

#### Oil Temperature Sensor
- Interface: ADC  
- Basic analog scaling + optional calibration table

#### Oil Pressure Sensor
- Interface: ADC  
- Supports simple linearization or custom calibration curves


---

## Planned Work

- Create custom PCB replacing the dev board  
- Mechanical design & enclosure for automotive installation  
---

## License
TBD


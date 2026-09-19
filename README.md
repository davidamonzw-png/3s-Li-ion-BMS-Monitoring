# 3S Li-Ion Battery Management & IoT Monitoring System

An ESP32-based monitoring system developed for a 3S Li-ion battery pack. The system combines battery voltage and temperature monitoring with local display, LED status indication, and remote IoT monitoring through Blynk.

## Project Overview

The system monitors the operating condition of a 3-cell series Li-ion battery pack using an ESP32 and a 3S BMS.

Battery parameters are displayed locally on a 16×2 I²C LCD and transmitted to a Blynk IoT dashboard for remote monitoring.

## Key Features

- Real-time battery pack voltage monitoring
- Temperature monitoring using an NTC thermistor
- Voltage-based battery status and percentage estimation
- 16×2 I²C LCD display
- LED-based battery status indication
- Blynk IoT dashboard
- Remote battery status monitoring
- Push notifications based on programmed battery conditions
- Voltage measurement validation using a multimeter

## Hardware

- ESP32
- 3S Li-ion battery pack
- 3S BMS
- 16×2 I²C LCD
- NTC thermistor
- Voltage divider circuit
- Status LEDs
- 7805 voltage regulator
- DC fan/load

## Software & Technologies

- Arduino IDE
- C/C++
- ESP32
- Blynk IoT
- I²C communication
- ADC-based voltage sensing

## System Architecture

Battery Pack → 3S BMS → Voltage & Temperature Sensing → ESP32 → LCD / LEDs / Blynk IoT

## Testing & Validation

The ESP32 voltage measurement was compared with a multimeter reading at the battery output to validate the voltage-sensing system.

The Blynk dashboard was also tested for real-time parameter updates and programmed push notifications.

## Current Limitation

The present battery percentage estimation is voltage-based and does not use coulomb counting or a dedicated current sensor.

## Future Improvements

- Add current sensing
- Implement coulomb counting
- Improve state-of-charge estimation
- Add cell-level voltage monitoring
- Improve data logging and battery performance analysis
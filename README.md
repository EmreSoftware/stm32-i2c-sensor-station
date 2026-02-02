# STM32 I2C Sensor Station

A multi-sensor I2C project for STM32F103C8T6 (Blue Pill) that reads acceleration, gyroscope, temperature, and pressure data, displaying it on a 16x2 LCD in real-time.

## Features

- 3 I2C devices on a single bus
- DMA-based I2C transfers (non-blocking)
- Startup device detection with animated LCD feedback
- Auto-recovery for MPU6050 sleep issues
- Real-time sensor display

## Hardware

| Component | Description |
|-----------|-------------|
| STM32F103C8T6 | Blue Pill development board |
| MPU6050 | 6-axis accelerometer/gyroscope |
| BMP180 | Temperature and pressure sensor |
| 16x2 LCD | HD44780 with PCF8574 I2C backpack |
| 2x 4.7kΩ resistors | I2C pull-ups |

## Wiring

| Signal | STM32 Pin | MPU6050 | BMP180 | LCD |
|--------|-----------|---------|--------|-----|
| SCL | PB6 | SCL | SCL | SCL |
| SDA | PB7 | SDA | SDA | SDA |
| VCC | 3.3V | VCC | VCC | - |
| VCC | 5V | - | - | VCC |
| GND | GND | GND | GND | GND |

Pull-up resistors: 4.7kΩ from SDA to 3.3V, 4.7kΩ from SCL to 3.3V.

## I2C Addresses

| Device | Address |
|--------|---------|
| MPU6050 | 0x68 |
| BMP180 | 0x77 |
| LCD (PCF8574) | 0x27 |

## Photos

| Startup Screen | Running |
|----------------|---------|
| ![Startup](https://github.com/user-attachments/assets/309b91d6-9000-4b63-abd7-76caf3a32f22) | ![Running](https://github.com/user-attachments/assets/dba60b80-e11d-4071-b4e6-0ac831a69851) |

## How It Works!


1. System initializes I2C with DMA
2. LCD shows startup animation and checks each device
3. Main loop reads sensors using DMA transfers
4. Temperature, pressure, and acceleration displayed on LCD
5. Auto-recovery if MPU6050 goes to sleep

## Build

1. Open project in STM32CubeIDE
2. Build and flash to Blue Pill
3. Connect hardware as shown above

## What I Learned

- I2C protocol fundamentals (START, STOP, ACK/NACK)
- Multi-device I2C bus management
- DMA transfers and callbacks
- LCD HD44780 4-bit mode via I2C backpack
- Sensor data processing (combining bytes, calibration)
- Pull-up resistor requirements for I2C

## License

MIT License. STM32 HAL code is under ST's license.

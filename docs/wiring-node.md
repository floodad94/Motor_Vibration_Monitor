# Sensor Node Wiring

## ADXL345 (I2C)

| Sensor | ESP32 |
|--------|------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

## Piezo Sensor

| Signal | ESP32 |
|--------|------|
| Output | GPIO 34 |
| GND | GND |

## WS2812 LED

| LED | ESP32 |
|-----|------|
| DIN | GPIO 4 |
| VCC | 5V |
| GND | GND |

## MAX485 (RS-485)

| MAX485 | ESP32 |
|--------|------|
| RO | GPIO 16 |
| DI | GPIO 17 |
| DE | GPIO 18 |
| RE | GPIO 18 |
# SCD41 + HC-SR04 Test

Two sensors running as independent FreeRTOS tasks on the Adafruit ESP32-C6 Feather.

## Sensors

- **SCD41** — CO2, temperature, humidity (I2C via STEMMA QT)
- **HC-SR04** — Ultrasonic distance sensor

## Wiring

| Signal | GPIO | Notes |
|--------|------|-------|
| I2C SDA | 19 | STEMMA QT |
| I2C SCL | 18 | STEMMA QT |
| I2C Power | 20 | Must be set HIGH to enable STEMMA QT |
| HC-SR04 Trig | 14 | Voltage divider (3.3V safe) |
| HC-SR04 Echo | 15 | |
| HC-SR04 VCC | 5V | |

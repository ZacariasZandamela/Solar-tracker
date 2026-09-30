# Dual-Axis Solar Tracker

An ESP32-based solar tracker that follows a light source using 4 LDR sensors
and two servo motors, with real-time panel power monitoring, storm-safe
shutdown, and a Random Forest model trained on real solar sensor data.

## Features

- **Dual-axis tracking** — azimuth (SG90) + zenith (MG996R)
- **Sensor fusion** — 4× LDR + DHT11 + ACS712 + voltage divider
- **Storm-safe shutdown** — rapid humidity/temperature change detection
- **Live power monitoring** — panel voltage, current, wattage on I2C LCD
- **Machine learning** — Random Forest regressors trained in Python,
  exported to C headers for ESP32 deployment

## Hardware

| Component | Purpose |
|---|---|
| ESP32 | Main controller |
| 4× LDR | Light direction sensing |
| DHT11 | Temperature and humidity |
| ACS712 | Panel current sensing |
| Voltage divider (2× 10kΩ) | Panel voltage sensing |
| SG90 servo | Azimuth axis |
| MG996R servo | Zenith axis |
| 16×2 I2C LCD | Live display |
| 6V 1W solar panel | Power source |

## Pin Assignments

| Signal | ESP32 Pin |
|---|---|
| DHT11 | GPIO 4 |
| LDR1–LDR4 | GPIO 34, 35, 32, 33 |
| Servo azimuth | GPIO 23 |
| Servo zenith | GPIO 19 |
| ACS712 | GPIO 27 |
| Panel voltage | GPIO 39 |
| LCD SDA / SCL | GPIO 21 / 22 |

## Project Structure



## Machine Learning Pipeline

Open `Notebook/training.ipynb` in Jupyter. The notebook:

1. Loads the solar sensor dataset
2. Engineers 14 features (LDR differences, rates of change, environmental)
3. Trains two Random Forest regressors (one per axis)
4. Evaluates MAE and R² on a held-out test set
5. Exports models to C headers via `emlearn` for ESP32 deployment

## Getting Started

Install the required Arduino libraries:
- `ESP32Servo` by Kevin Harrington
- `LiquidCrystal I2C` by Frank de Brabander
- `DHT sensor library for ESPx`

Then open `Arduino/Final_project.ino` in Arduino IDE, select your ESP32
board, and upload.

## Author

**Zacarias** — BTech Electronic Engineering, Durban University of Technology

## License

MIT
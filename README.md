# Quitelands AQ Monitoring

A compact indoor air-quality monitoring project for the Quitelands Villa environment. The system combines an ESP32-based air-quality node, SD-card logging, ThingSpeak cloud upload, and a browser-based live dashboard for AQI, PM2.5, PM10, CO2, temperature, and humidity.

## Overview

This repository contains:

- A live web dashboard for air-quality readings (`index.html`)
- A calibrated ESP32 firmware for the deployed AQ node
- A hardware test sketch for validation and troubleshooting
- Historical logged air-quality data in CSV format

The system is designed to keep sensing and logging even when Wi-Fi is unavailable, then sync data to the cloud when connectivity returns.

## Features

- Real-time AQI estimation using CPCB-style calculations
- Monitoring of:
  - PM2.5
  - PM10
  - CO₂
  - Temperature
  - Humidity
- Calibrated sensor values with per-node correction factors
- Offline logging to microSD card
- ThingSpeak bulk upload support
- Interactive dashboard with time-range filters and charting
- LED-based status indications for operational state

## Repository structure

```text
Quitelands_AQ_monitoring/
├── DATA/
│   └── aq_log24sep_5oct.csv
├── Hardware_codes/
│   ├── aq_node_calibrated/
│   │   └── aq_node_calibrated.ino
│   └── aq_node_hw_test/
│       └── aq_node_hw_test.ino
├── index.html
├── README.md
└── .gitignore (if present)
```

## Hardware and firmware

The field firmware is in:

- `Hardware_codes/aq_node_calibrated/aq_node_calibrated.ino`

This firmware is for the deployed ESP32 air-quality node and includes:

- SDS011 particulate sensor acquisition
- AHT10 temperature/humidity readings
- NDIR CO₂ sensing
- DS3231 RTC timestamping
- Wi-Fi + NTP synchronization
- SD-card raw/calibrated data logging
- ThingSpeak backlog upload

The testing firmware is here:

- `Hardware_codes/aq_node_hw_test/aq_node_hw_test.ino`

Use this sketch to validate sensor behavior and debugging before deployment.

## Dashboard

The dashboard is implemented in:

- `index.html`

It reads live data from ThingSpeak and displays:

- current AQI status
- pollutant cards
- historical trend charts
- time-range selection
- setup guide and LED interpretation

### Run locally

You can open it directly in a browser or serve it locally:

```bash
cd Quitelands_AQ_monitoring
python3 -m http.server 8000
```

Then open:

```text
http://localhost:8000
```

## Data flow

```text
Sensors -> ESP32 Firmware -> SD Card -> ThingSpeak -> Live Dashboard
```

The firmware stores each reading with a timestamp and calibrated values, then uploads backlog data when Wi-Fi is available.

## Notes

- The dashboard is configured for the Quitelands Villa node location and may need adjustments for a different deployment site.
- The firmware uses per-node calibration values defined in the `NODE_CAL` section.
- The system is designed for robustness in the field: sensor readings continue even when connectivity is lost.

## License

This project is intended for monitoring and research use. Please check the repository license file if one is added later.

## Contact / usage

This repository is intended as a personal monitoring setup for indoor air quality. It can be adapted for other homes, offices, or campus rooms by updating the node configuration, channel keys, and location metadata.

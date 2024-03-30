# Raspberry Pi Environment Controller

This Python script for Raspberry Pi uses an SHT31-D sensor to monitor temperature and humidity, controlling a fan, heater, and humidifier based on the readings. It logs the data, including device states, to a CSV file on a USB drive.

## Features

- **Temperature and Humidity Monitoring**: Utilizes an SHT31-D sensor for accurate environmental readings.
- **Device Control**: Automatically controls a fan, heater, and humidifier based on predefined environmental thresholds.
- **Data Logging**: Logs temperature, humidity, and device states to a CSV file for record-keeping and analysis.
- **Rounded Readings**: Rounds temperature and humidity readings to three decimal places for consistency and readability.

## Hardware Requirements

- Raspberry Pi (tested on Pi 4 and Pi Zero)
- SHT31-D sensor
- Devices for environmental control (fan, heater, humidifier)
- USB drive for data logging

## Software Requirements

- Python 3
- Adafruit Blinka library
- Adafruit CircuitPython SHT31D library

## Installation

1. **Enable I2C on Raspberry Pi**: Use `raspi-config` to enable I2C interface.
2. **Install Required Libraries**: Run the following commands to install necessary Python packages:
   ```bash
   pip3 install adafruit-blinka
   pip3 install adafruit-circuitpython-sht31d
   ```
3. **Mount USB Drive**: Ensure the USB drive is mounted correctly on the Raspberry Pi. The script assumes the mount point is `/media/johnhenry/EC18-177D`.

## Usage

1. **Connect the Hardware**: Connect the SHT31-D sensor and control devices (fan, heater, humidifier) to the Raspberry Pi according to the GPIO configuration in the script.
2. **Configure the Script**: Adjust the `TEMP_UPPER_BOUND`, `TEMP_LOWER_BOUND`, `HUMIDITY_UPPER_BOUND`, and `HUMIDITY_LOWER_BOUND` in the script as needed for your environment.
3. **Run the Script**: Execute the script with Python 3:
   ```bash
   python3 environment_controller.py
   ```
4. **Data Logging**: Ensure the USB drive is attached. The script logs data to `koji_data.csv` on the USB drive.

## Customization

- Modify GPIO pin assignments for fan, heater, and humidifier as per your hardware setup.
- Adjust temperature and humidity thresholds according to your environmental control needs.--

To use this README, simply create a `README.md` file in your project directory and copy the content into it. You might need to adjust paths, hardware setup, and customization details based on your specific project needs.

- Change the CSV file path if using a different mount point for the USB drive.

## Safety

- Ensure proper electrical safety when connecting devices to the Raspberry Pi.
- Use appropriate power supplies for the Raspberry Pi and connected devices to prevent damage.

## License

This project is open-source and available under the [MIT License](LICENSE.md).

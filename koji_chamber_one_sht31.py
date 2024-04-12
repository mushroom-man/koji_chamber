import board
import busio
import adafruit_sht31d
import time
import RPi.GPIO as GPIO
import csv
import os
import glob
import logging

# Setup logging
logging.basicConfig(filename='/media/johnhenry/EC18-177D/koji_log.log', level=logging.INFO, format='%(asctime)s %(levelname)s:%(message)s')

# Initialize One-Wire for DS18B20
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

# Discover DS18B20 device
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    try:
        with open(device_file, 'r') as f:
            valid, temp_line = f.readlines()
        return valid, temp_line
    except FileNotFoundError:
        logging.error("DS18B20 sensor file not found.")
        return None, None

def read_temp():
    valid, temp_line = read_temp_raw()
    while valid and 'YES' not in valid:
        time.sleep(0.2)
        valid, temp_line = read_temp_raw()
    if temp_line:
        equals_pos = temp_line.find('t=')
        if equals_pos != -1:
            temp_string = temp_line[equals_pos + 2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
    return None  # Return None if there was a problem reading the temperature

# GPIO setup
FAN_PIN = 17
HEATER_PIN = 27
HUMIDIFIER_PIN = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(HEATER_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(HUMIDIFIER_PIN, GPIO.OUT, initial=GPIO.LOW)

# Environmental thresholds
TEMP_UPPER_BOUND = 32
TEMP_LOWER_BOUND = 28
HUMIDITY_UPPER_BOUND = 85
HUMIDITY_LOWER_BOUND = 80

# Fan timing
FAN_INTERVAL = 3 * 60  # 3 minutes in seconds
last_fan_activation = time.time() - FAN_INTERVAL
FAN_DURATION = 60
next_fan_deactivation = 0

# CSV file setup
file_path = '/media/johnhenry/EC18-177D/koji_data.csv'
if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
    with open(file_path, 'a', newline='') as csvfile:
        data_writer = csv.writer(csvfile)
        data_writer.writerow(['Time', 'Temperature', 'Humidity', 'Rice Temperature', 'Fan', 'Heater', 'Humidifier'])

# Initialize I2C and SHT31-D sensor
i2c = busio.I2C(board.SCL, board.SDA)
sht31d = adafruit_sht31d.SHT31D(i2c)

try:
    while True:
        temperature = round(sht31d.temperature, 3)
        humidity = round(sht31d.relative_humidity, 3)
        rice_temp = read_temp()
        if rice_temp is not None:
            rice_temp = round(rice_temp, 3)
        else:
            logging.error("Failed to read rice temperature")
            continue  # Skip this loop iteration if temperature read failed
        
        # Active low relay control logic adjustment
        heater_state = not GPIO.input(HEATER_PIN)  # Invert the logic
        humidifier_state = not GPIO.input(HUMIDIFIER_PIN)  # Invert the logic

        if temperature < TEMP_LOWER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.LOW)  # Active low: LOW turns ON the relay
            heater_state = 1
        elif temperature > TEMP_UPPER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.HIGH)  # Active low: HIGH turns OFF the relay
            heater_state = 0

        if humidity < HUMIDITY_LOWER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)  # Active low: LOW turns ON the relay
            humidifier_state = 1
        elif humidity > HUMIDITY_UPPER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)  # Active low: HIGH turns OFF the relay
            humidifier_state = 0

        current_time = time.time()

        # Check if it's time to activate the fan
        if current_time - last_fan_activation >= FAN_INTERVAL and current_time > next_fan_deactivation:
            GPIO.output(FAN_PIN, GPIO.LOW)  # Active low: LOW turns ON the relay
            fan_state = 1
            last_fan_activation = current_time
            next_fan_deactivation = current_time + FAN_DURATION  # Schedule when the fan should be turned off

        # Check if it's time to deactivate the fan
        elif current_time > next_fan_deactivation:
            GPIO.output(FAN_PIN, GPIO.HIGH)  # Active low: HIGH turns OFF the relay
            fan_state = 0

        # Print status
        print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}, Temperature: {temperature} °C, Humidity: {humidity}%, Rice: {rice_temp} °C, Fan: {'ON' if fan_state else 'OFF'}, Heater: {'ON' if heater_state else 'OFF'}, Humidifier: {'ON' if humidifier_state else 'OFF'}")

        # Log data to CSV
        with open(file_path, 'a', newline='') as csvfile:
            data_writer = csv.writer(csvfile)
            # Convert boolean states to integers (0 or 1)
            heater_state_int = int(heater_state)
            humidifier_state_int = int(humidifier_state)
            fan_state_int = int(fan_state)
            # Write the converted states along with other data to the CSV file
            data_writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), temperature, humidity, fan_state_int, heater_state_int, humidifier_state_int])


        # Delay before next reading
        time.sleep(5)

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    GPIO.cleanup()  # Ensure GPIO resources are released properly
    logging.info("System shut down and GPIO cleaned up.")


import board
import busio
import adafruit_sht31d
import time
import RPi.GPIO as GPIO
import csv
import os

# GPIO setup
FAN_PIN = 17
HEATER_PIN = 27
HUMIDIFIER_PIN = 22

# Set up GPIO pins
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
last_fan_activation = time.time() - FAN_INTERVAL  # Ensure the fan can activate immediately

# CSV file setup
file_path = '/media/johnhenry/EC18-177D/koji_data.csv'
# Write headers if file is new
if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
    with open(file_path, 'a', newline='') as csvfile:
        data_writer = csv.writer(csvfile)
        data_writer.writerow(['Time', 'Temperature', 'Humidity', 'Fan', 'Heater', 'Humidifier'])

# Initialize I2C and SHT31-D sensor
i2c = busio.I2C(board.SCL, board.SDA)
sht31d = adafruit_sht31d.SHT31D(i2c)

# Main loop
try:
    while True:
        # Read sensor data
        temperature = round(sht31d.temperature, 3)
        humidity = round(sht31d.relative_humidity, 3)
        
        # Control logic
        heater_state = GPIO.input(HEATER_PIN)
        humidifier_state = GPIO.input(HUMIDIFIER_PIN)

        if temperature < TEMP_LOWER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.HIGH)
            heater_state = 1
        elif temperature > TEMP_UPPER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.LOW)
            heater_state = 0

        if humidity < HUMIDITY_LOWER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)
            humidifier_state = 1
        elif humidity > HUMIDITY_UPPER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)
            humidifier_state = 0

        fan_state = GPIO.input(FAN_PIN)
        if time.time() - last_fan_activation >= FAN_INTERVAL:
            GPIO.output(FAN_PIN, GPIO.HIGH)
            fan_state = 1
            last_fan_activation = time.time()

        # Print status
        print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}, Temperature: {temperature} °C, Humidity: {humidity}%, Fan: {'ON' if fan_state else 'OFF'}, Heater: {'ON' if heater_state else 'OFF'}, Humidifier: {'ON' if humidifier_state else 'OFF'}")

        # Log data to CSV
        with open(file_path, 'a', newline='') as csvfile:
            data_writer = csv.writer(csvfile)
            data_writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), temperature, humidity, fan_state, heater_state, humidifier_state])

        # Delay before next reading
        time.sleep(5)


except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    GPIO.cleanup()  # Clean up GPIO resources

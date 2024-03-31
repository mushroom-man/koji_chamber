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
FAN_DURATION = 60  # Duration the fan stays on in seconds, adjust as needed
next_fan_deactivation = 0  # Time by which the fan should be turned off
# Note that the fan interval must be greater than the fan duration in order for the fan to turn
# turn on and off.

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
        print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}, Temperature: {temperature} °C, Humidity: {humidity}%, Fan: {'ON' if fan_state else 'OFF'}, Heater: {'ON' if heater_state else 'OFF'}, Humidifier: {'ON' if humidifier_state else 'OFF'}")

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
    GPIO.cleanup()  # Clean up GPIO resources


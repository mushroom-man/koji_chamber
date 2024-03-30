import board
import busio
import adafruit_sht31d  # Import the SHT31D library
import time
import RPi.GPIO as GPIO

# GPIO setup
HUMIDIFIER_PIN = 17  # Example GPIO pin
HEATER_PIN = 27  # Example GPIO pin
FAN_PIN = 22  # Example GPIO pin

# GPIO access
GPIO.setmode(GPIO.BCM)
GPIO.setup(HUMIDIFIER_PIN, GPIO.OUT)
GPIO.setup(HEATER_PIN, GPIO.OUT)
GPIO.setup(FAN_PIN, GPIO.OUT)

# Bounds
TEMP_UPPER_BOUND = 32  # value in Celsius
TEMP_LOWER_BOUND = 28  # value in Celsius
HUMIDITY_UPPER_BOUND = 80  # value in %
HUMIDITY_LOWER_BOUND = 85  # value in %

# Fan timing
FAN_INTERVAL = 3 * 10  # 3 minutes expressed in seconds
last_fan_activation = time.time() - FAN_INTERVAL  # Ensures the fan can activate immediately

# Create I2C bus
try:
    i2c = busio.I2C(board.SCL, board.SDA)
except Exception as e:
    print("Error initializing I2C bus:", e)
    exit(1)

# Create the SHT31-D sensor object
try:
    sht31 = adafruit_sht31d.SHT31D(i2c)
except Exception as e:
    print("Error initializing SHT31-D sensor:", e)
    exit(1)

# Main loop
try:
    while True:
        current_time = time.time()
        # Read temperature and humidity from SHT31
        temperature = sht31.temperature
        humidity = sht31.relative_humidity

        # Temperature control logic
        if temperature > TEMP_UPPER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.LOW)  # Turn off the heater
        elif temperature < TEMP_LOWER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.HIGH)  # Turn on the heater
        else:
            GPIO.output(HEATER_PIN, GPIO.LOW)  # Ensure heater is off

        # Humidity control logic
        if humidity > HUMIDITY_UPPER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)  # Turn off the humidifier
        elif humidity < HUMIDITY_LOWER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)  # Turn on the humidifier
        else:
            GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)  # Ensure humidifier is off

        # Print the sensor readings
        print(f"Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %")
        

        # Automated fan activation based on the interval
        if current_time - last_fan_activation >= FAN_INTERVAL:
            GPIO.output(FAN_PIN, GPIO.HIGH)  # Turn on the fan
            print("Fan is ON")
            last_fan_activation = current_time  # Reset the last activation time
        else:
            # Check if the fan should be turned off after running for a certain duration
            # Adjust the duration according to your needs
            if current_time - last_fan_activation >= 60:  # Fan runs for 1 minute
                GPIO.output(FAN_PIN, GPIO.LOW)  # Turn off the fan
                print("Fan is OFF")

        # Check the status of the heater
        if GPIO.input(HEATER_PIN) == GPIO.LOW:  # If the heater pin is LOW, heater is OFF
            print("Heater is OFF")
        else:  # Otherwise, the heater pin is HIGH, and the heater is ON
            print("Heater is ON")
            
        # Check the status of the humidifier
        if GPIO.input(HUMIDIFIER_PIN) == GPIO.LOW:  # If the heater pin is LOW, heater is OFF
            print("Humidifier is OFF")
        else:  # Otherwise, the heater pin is HIGH, and the heater is ON
            print("Humidifier is ON")
            print()

        # Delay between readings
        time.sleep(5)
        
except RuntimeError as e:
    # Handle runtime errors related to GPIO access
    print(f"Runtime error accessing GPIO: {e}")
    
except KeyboardInterrupt:
    print("Program terminated by user")
    GPIO.cleanup()  # Clean up GPIO pins before exiting


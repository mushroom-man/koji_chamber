import RPi.GPIO as GPIO
import time

# Define pins
HEATER_PIN = 27  # BCM numbering
HUMIDIFIER_PIN = 22

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(HEATER_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(HUMIDIFIER_PIN, GPIO.OUT, initial=GPIO.HIGH)

print("Starting relay test. Press Ctrl+C to stop.")

try:
    while True:
        # Heater ON, Humidifier OFF
        GPIO.output(HEATER_PIN, GPIO.LOW)         # Active LOW = ON
        GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)    # Active LOW = OFF
        print("Heater ON, Humidifier OFF")
        time.sleep(5)

        # Heater OFF, Humidifier ON
        GPIO.output(HEATER_PIN, GPIO.HIGH)
        GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)
        print("Heater OFF, Humidifier ON")
        time.sleep(5)

except KeyboardInterrupt:
    print("Test interrupted by user.")

finally:
    GPIO.output(HEATER_PIN, GPIO.HIGH)
    GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)
    GPIO.cleanup()
    print("GPIO cleaned up. Test complete.")

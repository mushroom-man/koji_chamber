import board
import busio
import adafruit_sht31d
import time
import RPi.GPIO as GPIO
import csv
import os
import logging
import traceback
import random  # For the random fan activation
from datetime import datetime
from RPLCD.i2c import CharLCD

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/media/johnhenry/EC18-177D/koji_log.txt'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('koji_chamber')

# Set up error log - separate file for errors only
error_handler = logging.FileHandler('/media/johnhenry/EC18-177D/koji_errors.log')
error_handler.setLevel(logging.ERROR)
error_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(error_handler)

# Log startup
logger.info("=== Koji Chamber Control System Starting ===")
logger.info(f"Script started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

# GPIO setup with error handling
try:
    HEATER_PIN = 27
    HUMIDIFIER_PIN = 22
    FAN_PIN = 17  # Choose an available GPIO pin

    # Set up GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HEATER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(HUMIDIFIER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = fan OFF (assuming active LOW relay)
    logger.info(f"GPIO pins configured: Heater pin {HEATER_PIN}, Humidifier pin {HUMIDIFIER_PIN}, Fan pin {FAN_PIN}")
except Exception as e:
    logger.error(f"Failed to initialize GPIO: {e}")
    raise SystemExit("GPIO initialization failed, exiting program")

# Environmental thresholds
class EnvironmentalSettings:
    def __init__(self):
        self.TEMP_UPPER_BOUND = 32
        self.TEMP_LOWER_BOUND = 28
        self.HUMIDITY_UPPER_BOUND = 85
        self.HUMIDITY_LOWER_BOUND = 80

# Create settings object
settings = EnvironmentalSettings()

# Initialize device states
heater_state = 0
humidifier_state = 0
fan_state = 0
fan_next_activation = time.time() + random.uniform(30, 180)  # 30 seconds to 3 minutes
fan_duration = 0  # Will be set randomly when activated

# LCD Display setup
try:
    lcd_addresses = [0x27, 0x3F, 0x20]
    lcd = None
    
    for addr in lcd_addresses:
        try:
            logger.info(f"Attempting to initialize LCD at address 0x{addr:02X}")
            lcd = CharLCD(i2c_expander='PCF8574', address=addr, cols=16, rows=2, backlight_enabled=True)
            logger.info(f"LCD successfully initialized at address 0x{addr:02X}")
            break
        except Exception as e:
            logger.debug(f"LCD not found at address 0x{addr:02X}: {e}")
    
    if lcd is None:
        logger.warning("Could not initialize LCD display. No address worked.")
except Exception as e:
    logger.error(f"Error setting up LCD: {e}")
    lcd = None

# CSV file setup
try:
    data_dir = '/media/johnhenry/EC18-177D'
    file_path = os.path.join(data_dir, 'koji_data.csv')
    log_dir = data_dir
    
    os.makedirs(data_dir, exist_ok=True)
    logger.info(f"Using data directory: {data_dir}")
    
    if os.path.exists(file_path):
        backup_time = datetime.now().strftime('%Y%m%d%H%M%S')
        backup_path = os.path.join(data_dir, f'koji_data_backup_{backup_time}.csv')
        try:
            with open(file_path, 'r') as src, open(backup_path, 'w') as dst:
                dst.write(src.read())
            logger.info(f"Created backup of existing data file at {backup_path}")
        except Exception as e:
            logger.error(f"Failed to create backup: {e}")
    
    if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
        try:
            with open(file_path, 'a', newline='') as csvfile:
                data_writer = csv.writer(csvfile)
                data_writer.writerow(['Time', 'Temperature', 'Humidity', 'Heater', 'Humidifier', 'Fan'])
            logger.info(f"Created new data file at {file_path}")
        except Exception as e:
            logger.error(f"Failed to create CSV file: {e}")
            raise IOError(f"Could not create or access data file: {e}")
except Exception as e:
    logger.error(f"Fatal error setting up data files: {e}")
    GPIO.cleanup()
    raise SystemExit(f"Failed to set up data files: {e}")

# Initialize I2C and SHT31-D sensor
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sht31d = adafruit_sht31d.SHT31D(i2c)
    logger.info("SHT31-D sensor initialized successfully")
    
    initial_temp = round(sht31d.temperature, 3)
    initial_humidity = round(sht31d.relative_humidity, 3)
    logger.info(f"Initial sensor reading - Temperature: {initial_temp}°C, Humidity: {initial_humidity}%")
except Exception as e:
    logger.error(f"Failed to initialize SHT31-D sensor: {e}")
    GPIO.cleanup()
    raise SystemExit(f"Sensor initialization failed: {e}")

def log_data(timestamp, temperature, humidity, heater, humidifier, fan):
    """Log data to CSV file with error handling."""
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            with open(file_path, 'a', newline='') as csvfile:
                data_writer = csv.writer(csvfile)
                data_writer.writerow([timestamp, temperature, humidity, heater, humidifier, fan])
            return True  # Success
        except IOError as e:
            retry_count += 1
            logger.warning(f"IOError writing to CSV (attempt {retry_count}/{max_retries}): {e}")
            time.sleep(1)  # Wait before retrying
        except Exception as e:
            logger.error(f"Unexpected error writing to CSV: {e}")
            return False  # Don't retry for non-IO errors
            
    logger.error(f"Failed to write to CSV after {max_retries} attempts")
    return False  # Failed all retries

def get_sensor_readings():
    """Get temperature and humidity readings with error handling."""
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            temperature = round(sht31d.temperature, 3)
            humidity = round(sht31d.relative_humidity, 3)
            
            if temperature < -40 or temperature > 125 or humidity < 0 or humidity > 100:
                raise ValueError(f"Sensor readings out of range: temp={temperature}°C, humidity={humidity}%")
                
            return temperature, humidity
        except ValueError as e:
            retry_count += 1
            logger.warning(f"Invalid sensor reading (attempt {retry_count}/{max_retries}): {e}")
            time.sleep(1)
        except Exception as e:
            retry_count += 1
            logger.error(f"Failed to read from sensor (attempt {retry_count}/{max_retries}): {e}")
            time.sleep(1)
            
    logger.error(f"Failed to get valid sensor readings after {max_retries} attempts")
    return None, None

def control_heater(temperature):
    """Control heater with error handling."""
    try:
        if temperature < settings.TEMP_LOWER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.LOW)
            return 1
        elif temperature > settings.TEMP_UPPER_BOUND:
            GPIO.output(HEATER_PIN, GPIO.HIGH)
            return 0
        return GPIO.input(HEATER_PIN) == GPIO.LOW
    except Exception as e:
        logger.error(f"Error controlling heater: {e}")
        return None

def control_humidifier(humidity):
    """Control humidifier with error handling."""
    try:
        if humidity < settings.HUMIDITY_LOWER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.LOW)
            return 1
        elif humidity > settings.HUMIDITY_UPPER_BOUND:
            GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)
            return 0
        return GPIO.input(HUMIDIFIER_PIN) == GPIO.LOW
    except Exception as e:
        logger.error(f"Error controlling humidifier: {e}")
        return None

def control_fan():
    """Control the fan with random intervals and durations."""
    global fan_state, fan_next_activation, fan_duration
    
    current_time = time.time()
    
    if fan_state == 0 and current_time >= fan_next_activation:
        GPIO.output(FAN_PIN, GPIO.LOW)
        fan_state = 1
        fan_duration = current_time + random.uniform(10, 20)
        logger.info(f"Fan activated for {fan_duration - current_time:.1f} seconds")
    elif fan_state == 1 and current_time >= fan_duration:
        GPIO.output(FAN_PIN, GPIO.HIGH)
        fan_state = 0
        fan_next_activation = current_time + random.uniform(30, 180)
        logger.info(f"Fan deactivated, next activation in {fan_next_activation - current_time:.1f} seconds")
    
    return fan_state

def update_lcd(temperature, humidity, heater, humidifier, fan):
    """Update the LCD display with current readings and status."""
    if lcd is None:
        return
        
    try:
        lcd.clear()
        temp_str = f"T:{temperature:.1f}C"
        hum_str = f"H:{humidity:.1f}%"
        heater_status = "•" if heater else "o"
        humid_status = "•" if humidifier else "o"
        fan_status = "•" if fan else "o"
        
        lcd.cursor_pos = (0, 0)
        lcd.write_string(f"{temp_str} {hum_str}")
        
        lcd.cursor_pos = (1, 0)
        lcd.write_string(f"T:{heater_status} H:{humid_status} F:{fan_status}")
        
    except Exception as e:
        logger.error(f"Error updating LCD: {e}")

def create_custom_chars():
    """Create custom characters for the LCD if needed."""
    if lcd is None:
        return
        
    try:
        thermometer = (
            0b00100,
            0b01010,
            0b01010,
            0b01010,
            0b01010,
            0b10001,
            0b10001,
            0b01110
        )
        
        droplet = (
            0b00100,
            0b00100,
            0b01010,
            0b01010,
            0b10001,
            0b10001,
            0b10001,
            0b01110
        )
        
        lcd.create_char(0, thermometer)
        lcd.create_char(1, droplet)
        
        logger.info("Custom LCD characters created successfully")
    except Exception as e:
        logger.error(f"Error creating custom LCD characters: {e}")

try:
    logger.info("Starting koji chamber monitoring")
    
    if lcd is not None:
        create_custom_chars()
        lcd.clear()
        lcd.cursor_pos = (0, 0)
        lcd.write_string("Koji Chamber")
        lcd.cursor_pos = (1, 0)
        lcd.write_string("Starting...")
        time.sleep(2)
    
    consecutive_failures = 0
    total_readings = 0
    failed_readings = 0
    last_valid_temperature = None
    last_valid_humidity = None
    last_successful_write = time.time()
    last_lcd_update = 0
    lcd_update_interval = 2
    
    while True:
        loop_start_time = time.time()
        
        temperature, humidity = get_sensor_readings()
        total_readings += 1
        
        if temperature is None or humidity is None:
            failed_readings += 1
            consecutive_failures += 1
            
            if consecutive_failures >= 3:
                logger.warning(f"Multiple consecutive sensor failures ({consecutive_failures}). Using last valid values.")
                
                if last_valid_temperature is not None and last_valid_humidity is not None:
                    temperature = last_valid_temperature
                    humidity = last_valid_humidity
                    logger.info(f"Using fallback values: Temperature={temperature}°C, Humidity={humidity}%")
                else:
                    logger.error("No valid sensor readings available. Waiting before retry.")
                    
                    if lcd is not None:
                        lcd.clear()
                        lcd.cursor_pos = (0, 0)
                        lcd.write_string("Sensor Error!")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("Retrying...")
                    
                    time.sleep(5)
                    continue
        else:
            consecutive_failures = 0
            last_valid_temperature = temperature
            last_valid_humidity = humidity
        
        heater_state = control_heater(temperature)
        humidifier_state = control_humidifier(humidity)
        fan_state = control_fan()
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        status_message = f"Temperature: {temperature}°C, Humidity: {humidity}%, " \
                         f"Heater: {'ON' if heater_state else 'OFF'}, " \
                         f"Humidifier: {'ON' if humidifier_state else 'OFF'}, " \
                         f"Fan: {'ON' if fan_state else 'OFF'}"
        logger.info(status_message)
        
        current_time = time.time()
        if lcd is not None and (current_time - last_lcd_update) >= lcd_update_interval:
            update_lcd(temperature, humidity, heater_state, humidifier_state, fan_state)
            last_lcd_update = current_time

        write_success = log_data(timestamp, temperature, humidity, heater_state, humidifier_state, fan_state)
        
        if write_success:
            last_successful_write = time.time()
        elif time.time() - last_successful_write > 300:
            logger.error("No successful data writes for 5 minutes - possible file system issue")
            try:
                emergency_log = os.path.join(os.path.dirname(file_path), 'emergency_koji_data.csv')
                with open(emergency_log, 'a', newline='') as csvfile:
                    csv.writer(csvfile).writerow([timestamp, temperature, humidity, 
                                                  heater_state, humidifier_state, fan_state])
                logger.info(f"Created emergency log at {emergency_log}")
            except Exception as e:
                logger.error(f"Emergency logging also failed: {e}")

        loop_execution_time = time.time() - loop_start_time
        
        if total_readings % 100 == 0:
            failure_rate = (failed_readings / total_readings) * 100
            logger.info(f"Performance stats: {total_readings} total readings, " \
                        f"{failed_readings} failures ({failure_rate:.1f}%), " \
                        f"avg loop time: {loop_execution_time:.3f}s")
        
        sleep_time = max(0.5, 5 - loop_execution_time)
        time.sleep(sleep_time)

except KeyboardInterrupt:
    logger.info("Program terminated by user.")
except Exception as e:
    logger.error(f"Unexpected error: {e}", exc_info=True)
finally:
    try:
        GPIO.output(HEATER_PIN, GPIO.HIGH)
        GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)
        GPIO.output(FAN_PIN, GPIO.HIGH)
        
        if lcd is not None:
            lcd.clear()
            lcd.cursor_pos = (0, 0)
            lcd.write_string("System Shutdown")
            lcd.cursor_pos = (1, 0)
            lcd.write_string("Cleaning up...")
            time.sleep(2)
            lcd.clear()
            lcd.backlight_enabled = False
        
        GPIO.cleanup()
        logger.info("GPIO cleanup completed, program ended")
    except Exception as cleanup_error:
        logger.error(f"Error during GPIO cleanup: {cleanup_error}")
    
    logger.info(f"Session complete: {total_readings} total readings, {failed_readings} failures")
    logger.info("=== Koji Chamber Control System Stopped ===")

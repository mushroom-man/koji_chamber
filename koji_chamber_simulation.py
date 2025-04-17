import os
import csv
import time
import glob
import logging
import json
import signal
import RPi.GPIO as GPIO
import math
try:
    import board
    import busio
    import adafruit_sht31d
except ImportError:
    board = busio = adafruit_sht31d = None



class Config:
    """Configuration settings for the koji chamber."""
    
    # MQTT settings
    ENABLE_MQTT = False
    MQTT_BROKER = "localhost"
    MQTT_PORT = 1883
    MQTT_TOPIC = "koji/status"
    MQTT_RECONNECT_INTERVAL = 60  # seconds
    
    # GPIO Pins
    FAN_PIN = 17
    HEATER_PIN = 27
    HUMIDIFIER_PIN = 22
    
    # Environmental thresholds
    TEMP_UPPER_BOUND = 32
    TEMP_LOWER_BOUND = 28
    HUMIDITY_UPPER_BOUND = 85
    HUMIDITY_LOWER_BOUND = 80
    
    # Fan timing
    FAN_INTERVAL = 3 * 60  # 3 minutes
    FAN_DURATION = 60  # 1 minute
    
    # Paths
    EXTERNAL_PATH = '/media/johnhenry/EC18-177D'
    LOCAL_PATH = os.path.expanduser('~/Documents/koji_chamber')
    
    # Logging
    LOG_LEVEL = logging.INFO
    LOG_FORMAT = '%(asctime)s %(levelname)s:%(message)s'
    
    # Sample interval (seconds)
    SAMPLE_INTERVAL = 5
    
    # Recovery settings
    SENSOR_RETRY_INTERVAL = 30  # seconds
    MAX_CONSECUTIVE_ERRORS = 5
    WATCHDOG_TIMEOUT = 120  # seconds
    HARDWARE_RESET_PIN = None  # Set to GPIO pin if you have hardware reset capability


class WatchdogTimer:
    """Implements a software watchdog timer."""
    
    def __init__(self, timeout, callback=None):
        self.timeout = timeout
        self.callback = callback
        self.reset()
        
    def reset(self):
        """Reset the watchdog timer."""
        self.last_reset = time.time()
        
    def check(self):
        """Check if watchdog has timed out."""
        if time.time() - self.last_reset > self.timeout:
            if self.callback:
                self.callback()
            return True
        return False


class StorageManager:
    """Handles file paths and data persistence."""
    
    def __init__(self, config):
        self.config = config
        self.setup_paths()
        self.setup_logging()
        self.setup_csv()
    
    def setup_paths(self):
        """Set up file paths for data and logs."""
        if os.path.exists(self.config.EXTERNAL_PATH):
            self.base_path = self.config.EXTERNAL_PATH
        else:
            self.base_path = self.config.LOCAL_PATH
            # Ensure local directory exists
            os.makedirs(self.config.LOCAL_PATH, exist_ok=True)
            
        self.log_path = os.path.join(self.base_path, 'koji_log.log')
        self.data_path = os.path.join(self.base_path, 'koji_data.csv')
    
    def setup_logging(self):
        """Configure logging system."""
        logging.basicConfig(
            filename=self.log_path,
            level=self.config.LOG_LEVEL,
            format=self.config.LOG_FORMAT
        )
    
    def setup_csv(self):
        """Set up CSV file with headers if it doesn't exist."""
        if not os.path.exists(self.data_path) or os.path.getsize(self.data_path) == 0:
            self.write_csv_row([
                'Time', 'Temperature', 'Humidity', 'Rice Temperature', 
                'Fan', 'Heater', 'Humidifier'
            ])
    
    def write_csv_row(self, row_data):
        """Write a row to the CSV file with error handling."""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                with open(self.data_path, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(row_data)
                return True
            except Exception as e:
                logging.warning(f"CSV write failed (attempt {attempt+1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(1)  # Wait before retry
        
        logging.error(f"Failed to write to CSV after {max_retries} attempts")
        return False


class MQTTClient:
    """Handles MQTT communication with reconnection capability."""
    
    def __init__(self, config):
        self.config = config
        self.enabled = config.ENABLE_MQTT
        self.client = None
        self.connected = False
        self.last_connection_attempt = 0
        
        if self.enabled:
            self.setup_mqtt()
    
    def setup_mqtt(self):
        """Set up MQTT client with reconnection capability."""
        try:
            import paho.mqtt.client as mqtt
            
            # MQTT callbacks
            def on_connect(client, userdata, flags, rc):
                if rc == 0:
                    self.connected = True
                    logging.info("Connected to MQTT broker")
                else:
                    self.connected = False
                    logging.warning(f"Failed to connect to MQTT broker with code {rc}")
            
            def on_disconnect(client, userdata, rc):
                self.connected = False
                if rc != 0:
                    logging.warning(f"Unexpected MQTT disconnection with code {rc}")
            
            # Create client
            self.client = mqtt.Client()
            self.client.on_connect = on_connect
            self.client.on_disconnect = on_disconnect
            
            # Try initial connection
            self.try_connect()
            
        except ImportError:
            logging.warning("Paho MQTT library not found. MQTT publishing disabled.")
            self.enabled = False
    
    def try_connect(self):
        """Try to connect to MQTT broker."""
        if not self.enabled or self.connected:
            return
            
        current_time = time.time()
        if current_time - self.last_connection_attempt < self.config.MQTT_RECONNECT_INTERVAL:
            return
            
        self.last_connection_attempt = current_time
        
        try:
            logging.info(f"Attempting to connect to MQTT broker at {self.config.MQTT_BROKER}")
            self.client.connect(self.config.MQTT_BROKER, self.config.MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            logging.error(f"MQTT connection error: {e}")
            self.connected = False
    
    def publish_data(self, topic, payload):
        """Publish data to MQTT broker with reconnection logic."""
        if not self.enabled:
            return
            
        # Try to reconnect if not connected
        if not self.connected:
            self.try_connect()
            if not self.connected:
                return
        
        # Publish if connected
        try:
            result = self.client.publish(topic, json.dumps(payload))
            if result.rc != 0:
                logging.error(f"MQTT publish error: {result}")
                self.connected = False
            else:
                logging.debug(f"Published to {topic}: {payload}")
        except Exception as e:
            logging.error(f"MQTT publish error: {e}")
            self.connected = False
    
    def cleanup(self):
        """Clean up MQTT resources."""
        if self.enabled and self.client:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except:
                pass


class SensorManager:
    """Manages temperature and humidity sensors with reconnection capability."""
    
    def __init__(self, config):
        self.config = config
        self.env_sensor_available = False
        self.rice_sensor_available = False
        self.consecutive_errors = 0
        self.last_retry_time = 0
        
        # Initial setup
        self.setup_environment_sensor()
        self.rice_temp_reader = self.setup_rice_temp_sensor()
    
    def setup_environment_sensor(self):
        """Set up the SHT31D temperature and humidity sensor."""
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sht31d = adafruit_sht31d.SHT31D(self.i2c)
            self.env_sensor_available = True
            self.consecutive_errors = 0
            logging.info("SHT31D sensor initialized successfully")
            return True
        except Exception as e:
            self.env_sensor_available = False
            logging.error(f"Failed to initialize SHT31D sensor: {e}")
            # Only raise exception during initial setup, not during reconnection attempts
            if self.consecutive_errors == 0:
                raise RuntimeError("SHT31D sensor required but unavailable") from e
            return False
    
    def setup_rice_temp_sensor(self):
        """Set up the DS18B20 temperature sensor for rice temperature."""
        base_dir = '/sys/bus/w1/devices/'
        try:
            device_folder = glob.glob(base_dir + '28*')[0]
            device_file = os.path.join(device_folder, 'w1_slave')
            self.rice_sensor_available = True

            def read_temp_raw():
                try:
                    with open(device_file, 'r') as f:
                        valid, temp_line = f.readlines()
                    return valid, temp_line
                except Exception as e:
                    logging.warning(f"Error reading DS18B20 sensor: {e}")
                    self.rice_sensor_available = False
                    return None, None

            def read_temp():
                valid, temp_line = read_temp_raw()
                if valid is None:
                    return None
                    
                retry_count = 0
                # Try up to 5 times to get a valid reading
                while valid and 'YES' not in valid and retry_count < 5:
                    time.sleep(0.2)
                    valid, temp_line = read_temp_raw()
                    if valid is None:
                        return None
                    retry_count += 1
                    
                if temp_line and 'YES' in valid:
                    equals_pos = temp_line.find('t=')
                    if equals_pos != -1:
                        temp_string = temp_line[equals_pos + 2:]
                        temp_c = float(temp_string) / 1000.0
                        return round(temp_c, 3)
                return None

            logging.info("DS18B20 sensor found and initialized")
            return read_temp

        except (IndexError, FileNotFoundError) as e:
            self.rice_sensor_available = False
            logging.warning(f"DS18B20 not found: {e}. Rice temperature monitoring disabled.")
            return lambda: None
    
    def try_reconnect_sensors(self):
        """Try to reconnect to sensors if they're unavailable."""
        current_time = time.time()
        if current_time - self.last_retry_time < self.config.SENSOR_RETRY_INTERVAL:
            return
            
        self.last_retry_time = current_time
        
        # Try to reconnect to the environment sensor
        if not self.env_sensor_available:
            logging.info("Attempting to reconnect to SHT31D sensor...")
            if self.setup_environment_sensor():
                logging.info("Successfully reconnected to SHT31D sensor")
        
        # Try to reconnect to the rice temperature sensor
        if not self.rice_sensor_available:
            logging.info("Attempting to reconnect to DS18B20 sensor...")
            self.rice_temp_reader = self.setup_rice_temp_sensor()
    
    def read_sensors(self):
        """Read all sensor values with error handling and reconnection logic."""
        # Try to reconnect sensors if needed
        if not self.env_sensor_available or not self.rice_sensor_available:
            self.try_reconnect_sensors()
        
        # Default values
        result = {
            "temperature": None,
            "humidity": None,
            "rice_temp": None
        }
        
        # Read environment sensor
        if self.env_sensor_available:
            try:
                result["temperature"] = round(self.sht31d.temperature, 3)
                result["humidity"] = round(self.sht31d.relative_humidity, 3)
                self.consecutive_errors = 0
            except Exception as e:
                self.consecutive_errors += 1
                self.env_sensor_available = False
                logging.error(f"Error reading SHT31D sensor: {e}")
                
                # If too many consecutive errors, might need a hardware reset
                if self.consecutive_errors >= self.config.MAX_CONSECUTIVE_ERRORS:
                    logging.critical(f"Too many consecutive sensor errors ({self.consecutive_errors})")
                    if self.config.HARDWARE_RESET_PIN is not None:
                        self.trigger_hardware_reset()
        
        # Read rice temperature sensor
        if self.rice_sensor_available:
            try:
                result["rice_temp"] = self.rice_temp_reader()
            except Exception as e:
                self.rice_sensor_available = False
                logging.warning(f"Error reading rice temperature: {e}")
        
        # Verify we have essential data
        if result["temperature"] is None:
            raise RuntimeError("Failed to read temperature from SHT31D sensor")
            
        return result
    
    def trigger_hardware_reset(self):
        """Trigger a hardware reset of sensors if configured."""
        if self.config.HARDWARE_RESET_PIN is None:
            return
            
        try:
            logging.warning("Triggering hardware reset of sensors")
            GPIO.setup(self.config.HARDWARE_RESET_PIN, GPIO.OUT)
            GPIO.output(self.config.HARDWARE_RESET_PIN, GPIO.LOW)
            time.sleep(0.5)
            GPIO.output(self.config.HARDWARE_RESET_PIN, GPIO.HIGH)
            time.sleep(2)  # Give sensors time to restart
            self.setup_environment_sensor()
            self.rice_temp_reader = self.setup_rice_temp_sensor()
        except Exception as e:
            logging.error(f"Error during hardware reset: {e}")


class RelayController:
    """Controls relay outputs for fan, heater, and humidifier."""
    
    def __init__(self, config):
        self.config = config
        self.setup_gpio()
        
        # Fan state tracking
        self.fan_state = 0
        self.last_fan_activation = time.time() - config.FAN_INTERVAL
        self.next_fan_deactivation = 0
        
        # Current states of devices
        self.heater_state = 0
        self.humidifier_state = 0
    
    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # Suppress warnings for reconnection scenarios
        
        # Heater is critical — raise if it fails
        try:
            GPIO.setup(self.config.HEATER_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.heater_available = True
            logging.info(f"Heater relay initialized on pin {self.config.HEATER_PIN}")
        except Exception as e:
            logging.critical(f"Failed to initialize heater relay: {e}")
            self.heater_available = False
            raise RuntimeError("Critical relay initialization failed") from e

        # Try humidifier (non-critical — allow failure)
        try:
            GPIO.setup(self.config.HUMIDIFIER_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.humidifier_available = True
            logging.info(f"Humidifier relay initialized on pin {self.config.HUMIDIFIER_PIN}")
        except Exception as e:
            self.humidifier_available = False
            logging.warning(f"Humidifier not available on GPIO {self.config.HUMIDIFIER_PIN}: {e}")
        if not self.humidifier_available:
            logging.info("Humidifier will be skipped due to missing relay or wiring.")

        # Try fan (non-critical — allow failure)
        try:
            GPIO.setup(self.config.FAN_PIN, GPIO.OUT, initial=GPIO.HIGH)
            self.fan_available = True
            logging.info(f"Fan relay initialized on pin {self.config.FAN_PIN}")
        except Exception as e:
            self.fan_available = False
            logging.warning(f"Fan not available on GPIO {self.config.FAN_PIN}: {e}")
    
    def update_fan(self, current_time):
        """Update fan state based on timing."""
        if not hasattr(self, 'fan_available') or not self.fan_available:
            self.fan_state = 0
            return
            
        if current_time - self.last_fan_activation >= self.config.FAN_INTERVAL and current_time > self.next_fan_deactivation:
            try:
                GPIO.output(self.config.FAN_PIN, GPIO.LOW)  # Turn on (active low)
                self.fan_state = 1
                self.last_fan_activation = current_time
                self.next_fan_deactivation = current_time + self.config.FAN_DURATION
                logging.info(f"Fan activated for {self.config.FAN_DURATION} seconds")
            except Exception as e:
                logging.error(f"Error activating fan: {e}")
                self.fan_available = False
                self.fan_state = 0
        elif current_time > self.next_fan_deactivation and self.fan_state == 1:
            try:
                GPIO.output(self.config.FAN_PIN, GPIO.HIGH)  # Turn off
                self.fan_state = 0
                logging.info("Fan deactivated")
            except Exception as e:
                logging.error(f"Error deactivating fan: {e}")
                self.fan_available = False
    
    def update_heater(self, temperature):
        """Update heater state based on temperature."""
        if not self.heater_available:
            # Try to reinitialize heater if it's not available
            try:
                GPIO.setup(self.config.HEATER_PIN, GPIO.OUT, initial=GPIO.HIGH)
                self.heater_available = True
                logging.info("Heater relay reinitialized")
            except Exception as e:
                logging.error(f"Failed to reinitialize heater relay: {e}")
                return
        
        try:
            if temperature < self.config.TEMP_LOWER_BOUND and self.heater_state == 0:
                GPIO.output(self.config.HEATER_PIN, GPIO.LOW)  # Turn on (active low)
                self.heater_state = 1
                logging.info(f"Heater activated (temp={temperature}°C)")
            elif temperature > self.config.TEMP_UPPER_BOUND and self.heater_state == 1:
                GPIO.output(self.config.HEATER_PIN, GPIO.HIGH)  # Turn off
                self.heater_state = 0
                logging.info(f"Heater deactivated (temp={temperature}°C)")
        except Exception as e:
            logging.error(f"Error controlling heater: {e}")
            self.heater_available = False
    
    def update_humidifier(self, humidity):
        """Update humidifier state based on humidity."""
        if not self.humidifier_available:
            self.humidifier_state = 0
            # Try to reinitialize humidifier periodically
            try:
                GPIO.setup(self.config.HUMIDIFIER_PIN, GPIO.OUT, initial=GPIO.HIGH)
                self.humidifier_available = True
                logging.info("Humidifier relay reinitialized")
            except:
                return
            
        try:
            if humidity < self.config.HUMIDITY_LOWER_BOUND and self.humidifier_state == 0:
                GPIO.output(self.config.HUMIDIFIER_PIN, GPIO.LOW)
                self.humidifier_state = 1
                logging.info(f"Humidifier activated (humidity={humidity}%)")
            elif humidity > self.config.HUMIDITY_UPPER_BOUND and self.humidifier_state == 1:
                GPIO.output(self.config.HUMIDIFIER_PIN, GPIO.HIGH)
                self.humidifier_state = 0
                logging.info(f"Humidifier deactivated (humidity={humidity}%)")
        except Exception as e:
            logging.error(f"Error controlling humidifier: {e}")
            self.humidifier_available = False
            self.humidifier_state = 0
    
    def get_states(self):
        """Get current states of all relays."""
        return {
            "fan": self.fan_state,
            "heater": self.heater_state,
            "humidifier": self.humidifier_state
        }
    
    def cleanup(self):
        """Clean up GPIO resources."""
        try:
            GPIO.cleanup()
            logging.info("GPIO resources cleaned up")
        except:
            pass

class MockSensorManager:
    def __init__(self, config):
        self.config = config
        self.start_time = time.time()

    def read_sensors(self):
        t = time.time() - self.start_time

        # Simulate sinusoidal sensor readings
        temperature = 30 + 5 * math.sin(t / 5)  # now swings between 25–35°C
        humidity = 82.5 + 7.5 * math.sin((t + 5) / 4)  # swings ~75–90%
        rice_temp = 30 + 3 * math.sin((t + 10) / 6)  # swings ~27–33°C

        return {
            "temperature": round(temperature, 2),
            "humidity": round(humidity, 2),
            "rice_temp": round(rice_temp, 2)
        }


class KojiChamber:
    """Main class for the Koji chamber control system with restart capability."""
    
    def __init__(self):
        self.config = Config()
        self.storage = StorageManager(self.config)
        self.running = False
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Initialize watchdog
        self.watchdog = WatchdogTimer(
            self.config.WATCHDOG_TIMEOUT, 
            callback=self.watchdog_callback
        )
        
        # Initialize all subsystems
        try:
            self.sensors = MockSensorManager(self.config)
            self.relays = RelayController(self.config)
            self.mqtt = MQTTClient(self.config)
            logging.info("Koji chamber control system initialized successfully")
        except Exception as e:
            logging.critical(f"Failed to initialize koji chamber: {e}")
            self.cleanup()
            raise
    
    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        logging.info(f"Received signal {sig}, shutting down")
        self.running = False
    
    def watchdog_callback(self):
        """Callback when watchdog times out."""
        logging.critical("Watchdog timeout - system not responding")
        # Could implement automatic restart logic here if needed
    
    def run(self):
        """Main loop for the Koji chamber control system with restart capability."""
        self.running = True
        error_count = 0
        max_errors = 5
        
        logging.info("Koji chamber control system started")
        
        while self.running:
            try:
                # Reset watchdog
                self.watchdog.reset()
                
                # Read sensor data
                sensor_data = self.sensors.read_sensors()
                temperature = sensor_data["temperature"]
                humidity = sensor_data["humidity"]
                rice_temp = sensor_data["rice_temp"]
                
                # Update relay states
                current_time = time.time()
                self.relays.update_fan(current_time)
                self.relays.update_heater(temperature)
                self.relays.update_humidifier(humidity)
                
                # Get current states
                states = self.relays.get_states()
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                
                # Prepare data for display, logging and storage
                rice_display = rice_temp if rice_temp is not None else "N/A"
                
                # Display current status
                print(
                    f"Time: {timestamp}, "
                    f"Temp: {temperature}°C, "
                    f"Humidity: {humidity}%, "
                    f"Rice: {rice_display}°C, "
                    f"Fan: {'ON' if states['fan'] else 'OFF'}, "
                    f"Heater: {'ON' if states['heater'] else 'OFF'}, "
                    f"Humidifier: {'ON' if states['humidifier'] else 'OFF'}"
                )
                
                # Write to CSV
                self.storage.write_csv_row([
                    timestamp,
                    temperature,
                    humidity,
                    rice_temp if rice_temp is not None else '',
                    states['fan'],
                    states['heater'],
                    states['humidifier']
                ])
                
                # Log data
                logging.info(
                    f"Readings: Temp={temperature}, Humidity={humidity}, "
                    f"Rice={rice_display}, Fan={states['fan']}, "
                    f"Heater={states['heater']}, Humidifier={states['humidifier']}"
                )
                
                # Publish to MQTT if enabled
                self.mqtt.publish_data(self.config.MQTT_TOPIC, {
                    "time": timestamp,
                    "temperature": temperature,
                    "humidity": humidity,
                    "rice_temp": rice_temp,
                    "fan": states['fan'],
                    "heater": states['heater'],
                    "humidifier": states['humidifier']
                })
                
                # Reset error counter on successful iteration
                error_count = 0
                
            except Exception as e:
                error_count += 1
                logging.error(f"Error in main loop ({error_count}/{max_errors}): {e}")
                
                # If too many consecutive errors, attempt recovery
                if error_count >= max_errors:
                    logging.critical(f"Too many consecutive errors, attempting system recovery")
                    try:
                        # Try to reinitialize critical components
                        self.sensors = SensorManager(self.config)
                        self.relays = RelayController(self.config)
                        error_count = 0
                    except Exception as recovery_error:
                        logging.critical(f"Recovery failed: {recovery_error}")
                        self.running = False
            
            # Wait for next sample
            time.sleep(self.config.SAMPLE_INTERVAL)
                
        # Clean up when loop exits
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources before exiting."""
        logging.info("Cleaning up resources...")
        try:
            if hasattr(self, 'mqtt'):
                self.mqtt.cleanup()
        except:
            pass
            
        try:
            if hasattr(self, 'relays'):
                self.relays.cleanup()
        except:
            pass
            
        logging.info("System shut down and resources cleaned up")


if __name__ == "__main__":
    try:
        # Implement a restart loop in case of critical failures
        max_restarts = 5
        restart_count = 0
        restart_delay = 60  # seconds
        
        while restart_count < max_restarts:
            try:
                chamber = KojiChamber()
                chamber.run()
                # If we get here, it was a clean exit
                break
            except Exception as e:
                restart_count += 1
                print(f"Critical error: {e}")
                logging.critical(f"Failed to start or run Koji chamber: {e}")
                if restart_count < max_restarts:
                    logging.info(f"Attempting restart {restart_count}/{max_restarts} in {restart_delay} seconds")
                    time.sleep(restart_delay)
                else:
                    logging.critical(f"Maximum restarts ({max_restarts}) reached. Giving up.")
            finally:
                # Make sure GPIO is cleaned up
                try:
                    GPIO.cleanup()
                except:
                    pass
    except Exception as e:
        print(f"Fatal error: {e}")
        logging.critical(f"Fatal error: {e}")
        # Make sure GPIO is cleaned up
        try:
            GPIO.cleanup()
        except:
            pass

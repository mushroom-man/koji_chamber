# 1. Add to import section (already included in your code)
import random  # For the random fan activation

# 2. Add to GPIO setup section
FAN_PIN = 17  # Choose an available GPIO pin
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = fan OFF (assuming active LOW relay)
logger.info(f"Fan pin configured: Fan pin {FAN_PIN}")

# 3. Initialize fan state and timing variables (add near other state variables)
fan_state = 0
fan_next_activation = time.time() + random.uniform(30, 180)  # 30 seconds to 3 minutes
fan_duration = 0  # Will be set randomly when activated

# 4. Add this function for fan control
def control_fan():
    """Control the fan with random intervals and durations."""
    global fan_state, fan_next_activation, fan_duration
    
    current_time = time.time()
    
    # If fan is OFF and it's time to turn it ON
    if fan_state == 0 and current_time >= fan_next_activation:
        # Turn fan ON
        GPIO.output(FAN_PIN, GPIO.LOW)  # Active low: LOW turns ON the relay
        fan_state = 1
        
        # Set random duration between 10-20 seconds
        fan_duration = current_time + random.uniform(10, 20)
        logger.info(f"Fan activated for {fan_duration - current_time:.1f} seconds")
        
    # If fan is ON and duration has elapsed
    elif fan_state == 1 and current_time >= fan_duration:
        # Turn fan OFF
        GPIO.output(FAN_PIN, GPIO.HIGH)  # Active low: HIGH turns OFF the relay
        fan_state = 0
        
        # Set next activation time between 30 seconds and 3 minutes from now
        fan_next_activation = current_time + random.uniform(30, 180)
        logger.info(f"Fan deactivated, next activation in {fan_next_activation - current_time:.1f} seconds")
    
    return fan_state

# 5. Update the LCD display function (modify existing function)
def update_lcd(temperature, humidity, heater, humidifier, fan):
    """Update the LCD display with current readings and status."""
    if lcd is None:
        return  # Skip if LCD is not initialized
        
    try:
        # Clear the display
        lcd.clear()
        
        # Format the temperature and humidity with proper units
        temp_str = f"T:{temperature:.1f}C"
        hum_str = f"H:{humidity:.1f}%"
        
        # Add status indicators for devices
        # "•" character for ON, "o" character for OFF
        heater_status = "•" if heater else "o"
        humid_status = "•" if humidifier else "o"
        fan_status = "•" if fan else "o"
        
        # First row: Temperature and Humidity
        lcd.cursor_pos = (0, 0)
        lcd.write_string(f"{temp_str} {hum_str}")
        
        # Second row: Device status
        lcd.cursor_pos = (1, 0)
        lcd.write_string(f"H:{heater_status} H:{humid_status} F:{fan_status}")
        
    except Exception as e:
        logger.error(f"Error updating LCD: {e}")

# 6. Add to the main loop (find this section in your code and add the fan control)
# Inside the while True loop:
        
        # Control devices with error handling
        heater_state = control_heater(temperature)
        humidifier_state = control_humidifier(humidity)
        fan_state = control_fan()  # Add this line
        
        # Generate timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Log status (update to include fan)
        status_message = f"Temperature: {temperature}°C, Humidity: {humidity}%, " \
                         f"Heater: {'ON' if heater_state else 'OFF'}, " \
                         f"Humidifier: {'ON' if humidifier_state else 'OFF'}, " \
                         f"Fan: {'ON' if fan_state else 'OFF'}"
        logger.info(status_message)
        
        # Update LCD display at specified interval (update to include fan_state)
        current_time = time.time()
        if lcd is not None and (current_time - last_lcd_update) >= lcd_update_interval:
            update_lcd(temperature, humidity, heater_state, humidifier_state, fan_state)
            last_lcd_update = current_time

# 7. Update the CSV logging function call and function definition
# Change the log_data call to include fan_state:
        write_success = log_data(timestamp, temperature, humidity, heater_state, humidifier_state, fan_state)

# 8. Update the log_data function definition:
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

# 9. Update the CSV header creation (in the setup section)
# Find this code and update it:
    if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
        try:
            with open(file_path, 'a', newline='') as csvfile:
                data_writer = csv.writer(csvfile)
                data_writer.writerow(['Time', 'Temperature', 'Humidity', 'Heater', 'Humidifier', 'Fan'])
            logger.info(f"Created new data file at {file_path}")
        except Exception as e:
            logger.error(f"Failed to create CSV file: {e}")
            raise IOError(f"Could not create or access data file: {e}")

# 10. Finally, update the finally block to turn off the fan:
finally:
    # Ensure all devices are turned off
    try:
        GPIO.output(HEATER_PIN, GPIO.HIGH)
        GPIO.output(HUMIDIFIER_PIN, GPIO.HIGH)
        GPIO.output(FAN_PIN, GPIO.HIGH)  # Turn off fan
        
        # Display shutdown message
        # ... rest of your cleanup code

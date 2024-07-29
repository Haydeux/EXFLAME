import Jetson.GPIO as GPIO
import time

# Disable warnings
GPIO.setwarnings(False)

# Pin Definitions
led_pin = 12  # BCM pin 18, BOARD pin 12

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)  # Use BOARD pin numbering
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)  # Set pin as output and initialize to low

try:
    print("high")
    GPIO.output(led_pin, GPIO.HIGH)  # Turn LED on
    while True:
        pass
        # time.sleep(5)  # Wait for 5 seconds
        # print("low")
        # GPIO.output(led_pin, GPIO.LOW)  # Turn LED off
        # time.sleep(5)  # Wait for 5 seconds
except:
    print("stopped")
finally:
    GPIO.output(led_pin, GPIO.LOW)
    GPIO.cleanup()  # Clean up GPIO on exit
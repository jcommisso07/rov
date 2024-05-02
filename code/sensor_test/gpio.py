import RPi.GPIO as GPIO
import time

# Broadcom Pin numbers
GPIO.setmode(GPIO.BCM)

# Set pins as output
#pins = [12, 13, 14, 15, 16, 17]
pins = [17]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# Set PWM and initialize
pwms = [GPIO.PWM(pin, 100) for pin in pins]
for pwm in pwms:
    pwm.start(0)

try:
    while True:
        for dc in range(11, 19, 1):
            for pwm in pwms:
                # Change duty cycle
                pwm.ChangeDutyCycle(dc)
                # Sleep for 1 second
                time.sleep(1)
                # Reset duty cycle to 0
                pwm.ChangeDutyCycle(0)
except KeyboardInterrupt:
    pass
finally:
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
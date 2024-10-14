import RPi.GPIO as GPIO
import serial
from time import sleep

# GPIO pin setup
ENA = 18
IN1 = 23
IN2 = 24
IN3 = 27
IN4 = 22
ENB = 19

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Setup PWM for motor speed control
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000)  # 1kHz frequency
pwmB = GPIO.PWM(ENB, 1000)  # 1kHz frequency

# Start the PWM with a duty cycle of 100% (maximum speed)
speed = 100  # speed as percentage (0-100)
pwmA.start(speed)
pwmB.start(speed)

# Setup UART (serial) for Bluetooth communication
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def left():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Main loop
try:
    while True:
        if uart.in_waiting > 0:
            value = uart.read().decode('utf-8').strip()
            print(f"Received: {value}")

            if value == 'U':
                forward()
            elif value == 'D':
                backward()
            elif value == 'L':
                left()
            elif value == 'R':
                right()
            elif value == 'S':
                stop()
except KeyboardInterrupt:
    pass
finally:
    # Cleanup the GPIO pins when the program is interrupted
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

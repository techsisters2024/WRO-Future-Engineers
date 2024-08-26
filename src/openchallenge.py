from machine import Pin, time_pulse_us, PWM
import time

i = 0
# Initialize motor pins
motor1_in1 = Pin(4, Pin.OUT)
motor1_in2 = Pin(5, Pin.OUT)
motor2_in1 = Pin(7, Pin.OUT)
motor2_in2 = Pin(6, Pin.OUT)

# Initialize PWM for motors
pwm_motor1 = PWM(Pin(4))
pwm_motor1.freq(1000)  # Set frequency to 1kHz
pwm_motor2 = PWM(Pin(7))
pwm_motor2.freq(1000)  # Set frequency to 1kHz

# Initialize steering servo
servo = PWM(Pin(19))
servo.freq(50)  # Typical frequency for servos

trig = Pin(12, Pin.OUT)
echo = Pin(13, Pin.IN)
flag=0
# Function to move motors forward
def move_forward(speed):
    motor1_in1.value(1)
    motor1_in2.value(0)
    pwm_motor1.duty_u16(speed)
    
    motor2_in1.value(1)
    motor2_in2.value(0)
    pwm_motor2.duty_u16(speed)

# Function to move motors backward
def move_backward(speed):
    motor1_in1.value(0)
    motor1_in2.value(1)
    pwm_motor1.duty_u16(speed)
    
    motor2_in1.value(0)
    motor2_in2.value(1)
    pwm_motor2.duty_u16(speed)

# Function to stop motors
def stop_motors():
    motor1_in1.value(0)
    motor1_in2.value(0)
    pwm_motor1.duty_u16(0)
    
    motor2_in1.value(0)
    motor2_in2.value(0)
    pwm_motor2.duty_u16(0)

def set_servo_angle(angle):
    # Convert the angle to a duty cycle
    # Adjust the min and max values as per your servo's specification
    min_duty = 1000  # Min duty cycle (usually 1000 for 0 degrees)
    max_duty = 9000  # Max duty cycle (usually 9000 for 180 degrees)
    
    duty = int(min_duty + (max_duty - min_duty) * (angle / 180))
    servo.duty_u16(duty)
    
    
def turn_center():
    set_servo_angle(90)
def turn_right():
    set_servo_angle(0)
    time.sleep(1)
    move_forward (65536)
   
def turn_left():
    set_servo_angle(160)
    time.sleep(0.5)
    set_servo_angle(90)

def measure_distance():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    
    try:
        pulse_duration = time_pulse_us(echo, 1)
        time.sleep(0.5)
    except OSError:
        pulse_duration = -1
    if pulse_duration > 0:
        distance = (pulse_duration * 0.0343) / 2
        return distance
    else:
        return None
    

try:
    while True:
        distance = measure_distance()
        if distance < 70:
            turn_right()
            time.sleep(1)
            move forward(65536)
            time.sleep(1)
            set_servo_angle(58)
            move_forward(65536)
            time.sleep(2)
            print(distance)
            print('distance less 10')
            i+=1
            if i == 12:
                move_forward (65536)
                time.sleep(1)
                stop_motors()
            
        else: 
            set_servo_angle(58)
            move_forward(32768)
            print('greater than 10',distance)
            

except KeyboardInterrupt:
    print("Program stopped")
    stop_motors()
    


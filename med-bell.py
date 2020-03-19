#!/usr/bin/env python3
######################################################
# run meditation bell timer
######################################################
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

from time import sleep, strftime
from datetime import datetime, timedelta
import time
import RPi.GPIO as GPIO
import numpy as np
from pulsesensor_mb import Pulsesensor
import smbus
######################################################
# function definitions

# starts up LCD
def startup_lcd(mode):
    mcp.output(3, 1)
    lcd.begin(16, 2)

    # startup message
    if mode == 'LW':
        lcd_message(lcd, 'Hello, Luke W...\n')
        # print('Hello, Luke W...')
    else:
        lcd_message(lcd, 'Hello, Luke D...\n')
        # print('Hello, Luke D...')
    sleep(3)
    lcd.clear()

# displays message on LCD
def lcd_message(lcd, message):
    lcd.setCursor(0, 0)
    lcd.message(message)
    #print(message)

# displays time remaining on LCD
def show_time_left(lcd, c):
    mins = int(c/60)
    secs = int(c%60)
    lcd_message(lcd, 'Time: {:0>2}m {:0>2}sec'.format(mins, secs))
    # print('Time: {:0>2}m {:0>2}sec'.format(mins, secs))

# turns off LCD
def destroy_lcd():
    lcd.clear()
    print('destroying lcd...')


# pauses until keyboard exit
def pause_indefinitely():
    print('Pausing indefinitely...')
    while True:
        time.sleep(99)

# helper function for LED pulsing
def map(value, fromLow, fromHigh, toLow, toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

# set up GPIO
def setup_gpio():
    global p
    global p_R, p_G, p_B
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location

    # set up servo pin
    GPIO.setup(servo_pin, GPIO.OUT)   # Set servo_pin's mode is output
    GPIO.output(servo_pin, GPIO.LOW)  # Set servo_pin to low
    p = GPIO.PWM(servo_pin, 50)     # set Frequece to 50Hz
    p.start(0)                     # Duty Cycle = 0

    # Set button_pin's mode is input, and pull up to high
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # set switch pins as input, pulled up to high
    GPIO.setup(offlw_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(offld_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # set up breathing led pins. setting to low turns them off to start.
    for i in breathing_led_pins:
        GPIO.setup(breathing_led_pins[i], GPIO.OUT)
        GPIO.output(breathing_led_pins[i], GPIO.LOW)
    # set frequency to 2000
    p_R = GPIO.PWM(breathing_led_pins['pin_R'], 2000)  
    p_G = GPIO.PWM(breathing_led_pins['pin_G'], 2000)
    p_B = GPIO.PWM(breathing_led_pins['pin_B'], 2000)

# make the servo rotate to specific angle (0-180 degrees) 
def servo_write(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    # map the angle to duty cycle and output it
    p.ChangeDutyCycle(map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY))
    
# swings counter-clockwise once, and retracts 
def hit_ccw(start_angle, hit_angle, in_sleeptime, out_sleeptime):
    print('Swinging CCW...')
    for dc in range (start_angle, hit_angle, 1):
        servo_write(dc)
        time.sleep(in_sleeptime)
    for dc in range(hit_angle, start_angle, -1):
        servo_write(dc)
        time.sleep(out_sleeptime)

# swings clockwise once, and retracts 
def hit_cw(start_angle, hit_angle, end_angle, in_sleeptime, out_sleeptime):
    print('Swinging CCW...')
    for dc in range (start_angle, hit_angle, -1):
        servo_write(dc)
        time.sleep(in_sleeptime)
    for dc in range(hit_angle, end_angle, 1):
        servo_write(dc)
        time.sleep(out_sleeptime)

# helper function for detecting a button press (if button exists)
def button_event(channel):
    global button_pressed
    print('button event GPIO{}'.format(channel))
    button_pressed = True
    # print('button_pressed value: {}'.format(button_pressed))

# sets color on breathing LED
def set_color(r_val, g_val, b_val):
    p_R.ChangeDutyCycle(r_val)
    p_G.ChangeDutyCycle(g_val)
    p_B.ChangeDutyCycle(b_val)

# shut down connection to servo pin
def destroy_servo():
    p.stop()

# shut down the connection to GPIO
def destroy_pwm_and_gpio():
    p_R.stop()
    p_G.stop()
    p_B.stop()
    GPIO.cleanup()

# set current prediction to empty
def reset_prediction():
    file = open('current-prediction.txt', 'w')
    file.write('empty')
    file.close()

# set recording state to on or off
def set_recording_state(text):
    file = open('current-recording-state.txt', 'w')
    file.write(text)
    file.close()


# DO THIS FOR LW
def lw(GPIO):
    minutes=60
    #global button_pressed
    start_time = datetime.now()
    
    # start with some time on the clock
    starting_buffer_time = 0.1*minutes + 30*minutes
    print('starting buffer time: {}'.format(starting_buffer_time))
    done_time = start_time + timedelta(seconds=starting_buffer_time)
    print('start time: {}'.format(start_time))
    print('done time:  {}'.format(done_time))
    reset_prediction()
    set_recording_state('on')

    # repeat until time is up or switch is turned to middle
    while done_time > datetime.now() and GPIO.input(offlw_pin) == GPIO.LOW:

        # get current prediction
        file = open('current-prediction.txt')
        prediction = file.read().replace('\n', '')
        file.close()

        if prediction == 'enough':
            lcd_message(lcd, 'Hitting gong...')
            reset_prediction()
            time.sleep(3)
            break
        if prediction == 'not_enough':
            done_time = done_time + timedelta(minutes=15)
            lcd_message(lcd, 'Adding time...')
            reset_prediction()
            time.sleep(3)
        
        show_time_left(lcd, (done_time - datetime.now()).seconds)
        time.sleep(1)

    set_recording_state('off')

    # hit the gong
    hit_cw(70, 55, 70, 0.01, 0.01)
    destroy_servo()
    lcd.clear()

# helper function for compile_ramp
def f_factory(i, interval):
    start_point, end_point = interval
    slope = (end_point[1] - start_point[1])/(end_point[0] - start_point[0])
    def f(x):
        return slope*(x  -  start_point[0]) + start_point[1]
    return f

# takes a list of vertices and makes a piecewise linear function
def compile_ramp(list_of_vertices):
    num_vertices = len(list_of_vertices)
    num_lines = num_vertices - 1
    intervals, functions = {}, {}
    for line_idx in range(num_lines):
        intervals[line_idx] = (list_of_vertices[line_idx], list_of_vertices[line_idx+1])
    for i, interval in intervals.items():
        f = f_factory(i, interval)
        functions[i] = f
    list_of_xpts = [v[0] for v in list_of_vertices]
    fn_assignments = np.repeat([-1], 101)
    for i, x in enumerate(list_of_xpts):
        fn_assignments += [0]*(x+1) + [1]*(101-x-1)
    fn_assignments[0] = 0
    ramp = {}
    for i in range(101):
        ramp[i] = functions[fn_assignments[i]](i)
    return ramp

# use photoresistor to check if it is light or dark
def detect_darkness(bus):
    address = 0x48	#default address of PCF8591
    cmd=0x40
    channel = 2
    darkness_threshold = 200

    # for some reason first reading is bad
    value = bus.read_byte_data(address,cmd+channel)
    value = bus.read_byte_data(address,cmd+channel)
        
    print('darkness value: {}'.format(value))
    dark = True if value > darkness_threshold else False
    print('dark: {}'.format(dark))
    return dark

# defines the pulsing colors function
def set_multicolors_and_ramp(bus):
    dark = detect_darkness(bus)
    if dark:  # it's dark
        r0, g0, b0 = 119*1.5, 0, 135*1.5
        r1, g1, b1 = 0, 0, 0
        list_of_vertices = [[0,0], [40,10],  [70,50], [100,100]]
    else:  # it's bright
        r0, g0, b0 = 0, 0, 255
        r1, g1, b1 = 255, 0, 10
        list_of_vertices = [[0,0], [25,25], [50,50], [100,100]]
    ramp = compile_ramp(list_of_vertices)
    return r0, g0, b0, r1, g1, b1, ramp, dark


# DO THIS FOR LD
def ld(GPIO,  bus):
    global button_pressed
    p_R.start(0)
    p_G.start(0)
    p_B.start(0)
    r0, g0, b0, r1, g1, b1, ramp, dark = set_multicolors_and_ramp(bus)
    bus.close()
    in_breath_time = 5
    out_breath_time = 7

    pulse = Pulsesensor(f=None, channel = 0)
    pulse.startAsyncBPM()

    breath_count = 0

    while GPIO.input(offld_pin) == GPIO.LOW:
        breath_count += 1
        #lcd_message(lcd, 'Breath: {:<4} in '.format(breath_count))
        #set_color(0,0,0)
        #time.sleep(0.01*in_breath_time)

        breath_start_time = time.time()
        print('breath in')
        elapsed = (time.time()-breath_start_time)
        while elapsed < in_breath_time:
            i = int((elapsed)*100/in_breath_time)
            r_val = map(ramp[i], 0, 100, r1/255*100, r0/255*100)
            g_val = map(ramp[i], 0, 100, g1/255*100, g0/255*100)
            b_val = map(ramp[i], 0, 100, b1/255*100, b0/255*100)
            set_color(r_val, g_val, b_val)
            bpm = int(round(pulse.BPM,0)) if pulse.BPM > 0 else 'none'
            lcd_message(lcd, 'Breath: {:<3}in {:>2}\nBPM: {:<4}' \
                        .format(breath_count, int(elapsed)+1, bpm))
            time.sleep(0.005*in_breath_time)
            elapsed = (time.time()-breath_start_time)
            
        # little flicker at the top of the in breath
        set_color(0,0,0)
        time.sleep(0.005*in_breath_time)
            
        print('breath out')
        #lcd_message(lcd, 'Breath: {:<4} out'.format(breath_count))
        while elapsed < in_breath_time + out_breath_time:
            i = 100-int((elapsed-in_breath_time)*100/out_breath_time)
            r_val = map(ramp[i], 0, 100, r1/255*100, r0/255*100)
            g_val = map(ramp[i], 0, 100, g1/255*100, g0/255*100)
            b_val = map(ramp[i], 0, 100, b1/255*100, b0/255*100)
            set_color(r_val, g_val, b_val)
            bpm = int(round(pulse.BPM,0)) if pulse.BPM > 0 else 'none'
            lcd_message(lcd, 'Breath: {:<3}out{:>2}\nBPM: {:<4}' \
                        .format(breath_count, int(elapsed-in_breath_time)+1,
                                bpm))
            time.sleep(0.005*out_breath_time)
            elapsed = (time.time()-breath_start_time)

        if not dark:
            set_color(0,0,0)
            time.sleep(0.005*out_breath_time)

    # turn off pulse sensor
    pulse.stopAsyncBPM()


######################################################
# initialize devices

# LCD
PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
# Create PCF8574 GPIO adapter.
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print('I2C Address Error !')
        exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)

# SERVO
# define pulse offset of servo
OFFSE_DUTY = 0.5
# define pulse duty cycle for minimum angle of servo
SERVO_MIN_DUTY = 2.5+OFFSE_DUTY
# define pulse duty cycle for maximum angle of servo
SERVO_MAX_DUTY = 12.5+OFFSE_DUTY
servo_pin = 12

# LED
breathing_led_pins = {'pin_R':33, 'pin_G':35, 'pin_B':37}

# other initialization
minutes = 60
button_pin = 22

#onoff_pin = 31
#lwld_pin = 40

offlw_pin = 40 #38
offld_pin = 38 #40

####################################################
if __name__ == '__main__':
    print('Program is starting ... ')
    #time.sleep(3)
    setup_gpio()
    mcp.output(3, 1)
    lcd.begin(16, 2)
    button_pressed = False

    while GPIO.input(offlw_pin) == GPIO.HIGH and GPIO.input(offld_pin) == GPIO.HIGH:
        # means switch is in MIDDLE position and current is not flowing
        print('Breadboard is off!')
        lcd_message(lcd, '* MedBell 2020 *')
        time.sleep(1)

    lcd.clear()
    print('Breadboard is on!')

    # mode is LW if switch is connecting offlw_pin and LD otherwise
    mode = 'LW' if GPIO.input(offlw_pin) == GPIO.LOW else 'LD'
    startup_lcd(mode)

    if mode == 'LW':
        lw(GPIO)
    elif mode == 'LD':
        bus = smbus.SMBus(1)
        ld(GPIO, bus)

    # wait for breadboard to be shut off to end program
    while GPIO.input(offlw_pin) == GPIO.LOW or GPIO.input(offld_pin) == GPIO.LOW:
        time.sleep(1)

    # shut things down
    destroy_lcd()
    destroy_pwm_and_gpio()

    from subprocess import call
    print('shutting down...')
    call("sudo shutdown -h now", shell=True)
    exit(0)



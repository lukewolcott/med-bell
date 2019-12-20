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

######################################################
# function definitions

# get CPU temperature and store it into file "/sys/class/thermal/thermal_zone0/temp"
# TODO: remove
def get_cpu_temp():
    tmp = open('/sys/class/thermal/thermal_zone0/temp')
    cpu = tmp.read()
    tmp.close()
    return '{:.2f}'.format( float(cpu)/1000 ) + ' C'

# get system time
# TODO: remove
def get_time_now():     
    return datetime.now().strftime('    %H:%M:%S')
    
# TODO: remove
def freenove_loop():
    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)     # set number of LCD lines and columns
    while(True):         
        #lcd.clear()
        lcd.setCursor(0,0)  # set cursor position
        lcd.message( 'CPU: ' + get_cpu_temp()+'\n' )# display CPU temperature
        lcd.message( get_time_now() )   # display the time
        sleep(1)

# TODO: remove
def count_down(count_from):
    c = count_from
    while c > 0:
        mins = int(c/60)
        secs = c%60
        lcd_message(lcd, 'Time: {:0>2}m {:0>2}sec'.format(mins, secs))
        c -= 1
        sleep(1)
    lcd.clear()

# TODO: remove
def Freenove_buttonEvent(channel):
    global ledState
    print ('buttonEvent GPIO%d' %channel)
    ledState = not ledState
    if ledState :
        print ('Turn on LED ... ')
    else :
        print ('Turn off LED ... ')
    GPIO.output(ledPin,ledState)
############################################################

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


def lcd_message(lcd, message):
    lcd.setCursor(0, 0)
    lcd.message(message)
    print(message)


def show_time_left(lcd, c):
    mins = int(c/60)
    secs = int(c%60)
    lcd_message(lcd, 'Time: {:0>2}m {:0>2}sec'.format(mins, secs))
    # print('Time: {:0>2}m {:0>2}sec'.format(mins, secs))


def destroy_lcd():
    lcd.clear()
    print('destroying lcd...')


# pauses until keyboard exit
def pause_indefinitely():
    print('Pausing indefinitely...')
    while True:
        time.sleep(99)


def map(value, fromLow, fromHigh, toLow, toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow


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

    # set up breathing led pins
    for i in breathing_led_pins:
        GPIO.setup(breathing_led_pins[i], GPIO.OUT)
        GPIO.output(breathing_led_pins[i], GPIO.LOW)  # Set pins to high(+3.3V) to off led
    p_R = GPIO.PWM(breathing_led_pins['pin_R'], 2000)  # set Frequece to 2KHz
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


def reset_prediction():
    file = open('current-prediction.txt', 'w')
    file.write('empty')
    file.close()


def set_recording_state(text):
    file = open('current-recording-state.txt', 'w')
    file.write(text)
    file.close()


# DO THIS FOR LW
def lw(GPIO):#, button_pressed):
    global button_pressed
    start_time = datetime.now()
    
    # start with some time on the clock
    starting_buffer_time = 0.1*minutes
    done_time = start_time + timedelta(seconds=starting_buffer_time)
    reset_prediction()
    set_recording_state('on')

    GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_event, bouncetime=300)
    while done_time > datetime.now():
        if button_pressed:
            done_time = done_time + timedelta(minutes=0.1)
            button_pressed = False

        file = open('current-prediction.txt')
        prediction = file.read().replace('\n', '')
        file.close()

        print(prediction)
        if prediction == 'enough':
            lcd_message(lcd, 'Hitting gong...')
            reset_prediction()
            time.sleep(3)
            break
        if prediction == 'not_enough':
            done_time = done_time + timedelta(minutes=0.2)
            lcd_message(lcd, 'Adding time...')
            reset_prediction()
            time.sleep(3)
        
        show_time_left(lcd, (done_time - datetime.now()).seconds)
        time.sleep(1)

    set_recording_state('off')

    # hit the gong
    hit_ccw(90, 160, 0.002, 0.002)
    destroy_servo()

def compile_ramp():
    ramp = {}
    for i in range(101):
        ramp[i] = i
    return ramp
    
# DO THIS FOR LD
def ld(GPIO):
    global button_pressed
    p_R.start(0)
    p_G.start(0)
    p_B.start(0)
    r0, g0, b0 = 119, 0, 135 #119, 0, 135
    r1, g1, b1 = 0,0,0 #255
    in_breath_time = 2 #5
    out_breath_time = 2 #7

    ramp = compile_ramp()

    GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_event, bouncetime=300)
    breath_count = 0

    while GPIO.input(offld_pin) == GPIO.LOW:
        breath_count += 1
        # print('starting while loop. button_pressed: {}'.format(button_pressed))
        lcd_message(lcd, 'Breath: {}'.format(breath_count))
        print('breath in')
        for i in range(0, 100):  # only goes for 99% of the in breath
            r_val = map(ramp[i], 0, 100, r1/255*100, r0/255*100)
            g_val = map(ramp[i], 0, 100, g1/255*100, g0/255*100)
            b_val = map(ramp[i], 0, 100, b1/255*100, b0/255*100)
            set_color(r_val, g_val, b_val)
            time.sleep(0.01*in_breath_time)

        # little flicker at the top for 1% of the in breath
        set_color(0,0,0)
        time.sleep(0.01*in_breath_time)
            
        print('breath out')
        for i in range(100, 0, -1):
            r_val = map(ramp[i], 0, 100, r1/255*100, r0/255*100)
            g_val = map(ramp[i], 0, 100, g1/255*100, g0/255*100)
            b_val = map(ramp[i], 0, 100, b1/255*100, b0/255*100)
            set_color(r_val, g_val, b_val)
            time.sleep(0.01*out_breath_time)

#        for i in range(0, 101):
#            r_val=100 - map(r*i/100, 0, 255, 0, 100)
#            g_val=100 - map(g*i/100, 0, 255, 0, 100)
#            b_val=100 - map(b*i/100, 0, 255, 0, 100)
#            set_color(r_val, g_val, b_val)
#            time.sleep(0.01*in_breath_time)
#
#        print('breath out')
#        for i in range(100, 0, -1):
#            r_val=100 - map(r*i/100, 0, 255, 0, 100)
#            g_val=100 - map(g*i/100, 0, 255, 0, 100)
#            b_val=100 - map(b*i/100, 0, 255, 0, 100)
#            set_color(r_val, g_val, b_val)
#            time.sleep(0.01*out_breath_time)

    # TODO: add code that  will show heart rate


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

offlw_pin = 31
offld_pin = 40

####################################################
if __name__ == '__main__':
    print('Program is starting ... ')
    time.sleep(3)
    setup_gpio()
    mcp.output(3, 1)
    lcd.begin(16, 2)
    button_pressed = False

    while GPIO.input(offlw_pin) == GPIO.HIGH and GPIO.input(offld_pin) == GPIO.HIGH:
        # means switch is in MIDDLE position and current is not flowing
        print('Breadboard is off!')
        lcd_message(lcd, 'Breadboard off.')
        time.sleep(1)

    print('Breadboard is on!')

    # mode is LW if switch is connecting offlw_pin and LD otherwise
    mode = 'LW' if GPIO.input(offlw_pin) == GPIO.LOW else 'LD'
    startup_lcd(mode)

    if mode == 'LW':
        lw(GPIO)
    elif mode == 'LD':
        ld(GPIO)

    # wait for breadboard to be shut off to end program
    while GPIO.input(offlw_pin) == GPIO.LOW or GPIO.input(offld_pin) == GPIO.LOW:
        time.sleep(1)

    # shut things down
    destroy_lcd()
    destroy_pwm_and_gpio()
#    from subprocess import call
#    print('shutting down...')
#    call("sudo shutdown -h now", shell=True)
    #exit(0)

#    # just wait for keyboard interrupt then shut things down
#    try:
#        pause_indefinitely()
#    except KeyboardInterrupt:
#        destroy_lcd()
#        destroy_pwm_and_gpio()


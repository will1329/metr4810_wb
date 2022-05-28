#Complitation1
#Connections List:
'''
Stepper Driver: A-12 B-16 C-20 D-21
Servo Data Pin: 23
Ir Pins: En-24, Out-25
LCD Pins: rs-22 en-5 d4-6 d5-13 d6-19 d7-26
'''

#---------------------libraries-----------------------------------

#stepper
from time import sleep
import RPi.GPIO as GPIO

#lcd
import board
from digitalio import DigitalInOut
from adafruit_character_lcd.character_lcd import Character_LCD_Mono

#servo
from gpiozero import Servo


#---------------------constants------------------------------------
#stepper pins
IN1=12 # IN1
IN2=16 # IN2
IN3=20 # IN3
IN4=21 # IN4

# Time betwwen stepper steps
time = 0.001

#stepper motor rotation calibration 
#(=desired/actual) most recent: 10 r code: 7.05 actual
mrc = 1.4184

#LCD constants
lcd_columns = 16
lcd_rows = 2

#lcd pins declared in init

#button Pin
Button_pin = 17

#servo pin
servo = 23

#servo duty cycle range
lower_dc = 2 #-90 deg
upper_dc = 7 #90 deg

#ir pins
ir_en = 24 #enable
ir_out = 25 #output

#---------------------initialising---------------------------------
# set board reference mode
GPIO.setmode(GPIO.BCM)

#stepper pins
# set pins as outputs
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)

#Set output pins to low.
GPIO.output(IN1, False)
GPIO.output(IN2, False)
GPIO.output(IN3, False)
GPIO.output(IN4, False)

#set up lcd pins as outputs
lcd_rs = DigitalInOut(board.D22)
lcd_en = DigitalInOut(board.D5)
lcd_d4 = DigitalInOut(board.D6)
lcd_d5 = DigitalInOut(board.D13)
lcd_d6 = DigitalInOut(board.D19)
lcd_d7 = DigitalInOut(board.D26)

# Initialise the LCD class
lcd = Character_LCD_Mono(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows)

# Clear LCD
lcd.message = " clear \n clear"
sleep(3)

#set button pin to low input
GPIO.setup(Button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Set servo control pi as output
GPIO.setup(servo, GPIO.OUT)

#initialize pwm for servo pin
p = GPIO.PWM(servo, 50) # second argument is hertz, sg90 servo runs on 50Hz logic

#Set servo initial position to 0 deg = 7ms duty cycle
p.start(upper_dc+1)

#ir setup
GPIO.setup(ir_en,GPIO.OUT)
GPIO.setup(ir_out,GPIO.IN)

#cheeky delay for safety
sleep(1)

#----------------------Stepper Method-----------------------------

def Step1():
    #0001
    stepCode = [0, 0, 0, 1]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step2():
    #0011
    stepCode = [0, 0, 1, 1]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step3():
    #0010
    stepCode = [0, 0, 1, 0]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step4():
    #0110
    stepCode = [0, 1, 1, 0]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step5():
    #0100
    stepCode = [0, 1, 0, 0]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)
 

def Step6():
    #1100
    stepCode = [1, 1, 0, 0]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step7():
    #1000
    stepCode = [1, 0, 0, 0]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)


def Step8():
    #1001
    stepCode = [1, 0, 0, 1]
    GPIO.output(IN1, stepCode[0])
    GPIO.output(IN2, stepCode[1])
    GPIO.output(IN3, stepCode[2])
    GPIO.output(IN4, stepCode[3])
    sleep(time)
  
def ccwfine(step):	
	for i in range (int(round(step*mrc))):   
		Step1()
		Step2()
		Step3()
		Step4()
		Step5()
		Step6()
		Step7()
		Step8()  
		print( "Step Counter Clockwise: ",i)

def cwfine(step):
	for i in range (int(round(step*mrc))):
		Step8()
		Step7()
		Step6()
		Step5()
		Step4()
		Step3()
		Step2()
		Step1()  
		print( "Step Clockwise",i)
        
#----------------------LCD Methods --------------------------------
def set_lcd(message):
	lcd.message = str(message)
	
def clear_lcd():
	lcd.message = "                \n                "
	
#-----------------------Button Methods----------------------------

def button_pressed():
	if(GPIO.input(Button_pin)):   
		#print('y')
		return True
	else:     
		#print('n')
		return False
	time.sleep(0.1)
#-------------------------Servo Methods ---------------------------		
def servo_open():
	j = 8
	
	#sweep from upper to lower duty cycles
	for i in reversed(range(((lower_dc+1)*j),(j*upper_dc))):
		p.ChangeDutyCycle((i/j)) #argument is the %duty cycle = duty/frequency, 1/20ms = 5% == -90
		sleep(0.1/j) 

def servo_close():
	j = 8
	
	#sweep from lower back to upper duty cycles
	for i in range((lower_dc*j),((upper_dc+1)*j)):
		p.ChangeDutyCycle((i/j)) #argument is the %duty cycle = duty/frequency, 1/20ms = 5% == -90
		sleep(0.1/j)	

#-----------------ir methods-------------------------------------------
def ir_check():
	
	GPIO.output(ir_en, True) # turn blaster on
	sleep(0.001)
	x = GPIO.input(ir_out)   #read state of receiver False:detection
	GPIO.output(ir_en, False) # turn blaster off 
	return x
    
def hand_detect():
	x = False # Defalt hand not detected 
	i = 0   #build counter
	number = 3 # number represents number of consectutive detections represent a hand
	
	#if hand detected %number% times in a row return hand detected
	for j in range(number):
		if not ir_check():
			i=i+1
	
	if (i == number):
		x = True
	    							
	return x

#-----------------Main-------------------------------------------

def main():
	
	total_masks = 10 #change to fit number of masks loaded0
	while (total_masks>0):
		if hand_detect():
			
			set_lcd("Dispensing      \n                ")
			
			servo_open() #open door
				
			ccwfine(360) # Attempt to dispense one mask
			
			sleep(2) #allow mask to be removed
				
			servo_close() #close door
			
			#delay  dipensing of next mask to prevent unwanted dispence and IR saturation
			for i in reversed(range(3)):
				set_lcd("Please Wait: " + str(i) + "\n seconds        " )
				print("program exiting in: ", i)
				sleep(0.3)
			
			total_masks -= 1
			
			
		else:
			set_lcd("wave hand under\n masks left: " +  str(total_masks))
	
	
	# if masks empty display reload notification
	set_lcd("out of masks    \nplease reload :)")
	sleep(3)
		
		

try:	
	main()
	set_lcd("     cleanup    \n                ")
	sleep(3)
	GPIO.cleanup()

#exit code with (ctrl) + (c)
except KeyboardInterrupt:
	p.stop()
	clear_lcd()
	GPIO.cleanup()
	print("program stopped")

GPIO.cleanup()

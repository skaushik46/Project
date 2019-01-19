"""
* Team Id : eYRC #2415
* Author List : Vivek Nehra
* Filename : Final_Line_Follower.py

* Functions : speed_control(),sharp_left(),shart_right(),forward(),backward(),left(),right(),direction_control(),create_roi(),check_if_black(),make_sharp_turn(),check_plane(),modulus()
		blend_transparent(),plantation_zone_detect(),create_overlay(),contours(),led_glow(),led_stop(),final_glow()

* Global Variables: zone_number, end_run_glow
"""
# Import required libraries
import cv2
import numpy as np
from picamera import PiCamera
import time
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from time import sleep




################## Line Follower Functions #################

# Motor Pins

# Refers to the pins connected to motor in Raspberry pi
left_motorA = 38	# Referred as forward_left
left_motorB = 36	# Referred as reverse_left
left_motorE = 40
right_motorA = 32	# Referred as forward_right
right_motorB = 35	# Referred as reverse_right
right_motorE = 37

GPIO.setwarnings(False)		# To disable any warning provided by the board
GPIO.setmode(GPIO.BOARD)	# Set mode of GPIO board to access gpio pins by their pin number on board
GPIO.setup(left_motorA,GPIO.OUT) 	# Setup GPIO pins to output mode
GPIO.setup(left_motorB,GPIO.OUT)
GPIO.setup(left_motorE,GPIO.OUT)
GPIO.setup(right_motorA,GPIO.OUT)
GPIO.setup(right_motorB,GPIO.OUT)
GPIO.setup(right_motorE,GPIO.OUT)


left_DutyA = GPIO.PWM(left_motorA, 100)		# PWM working on ACW and CW pins of the motors rather than the enable pin
right_DutyA = GPIO.PWM(right_motorA, 100)
left_DutyB = GPIO.PWM(left_motorB,100)
right_DutyB = GPIO.PWM(right_motorB,100)

# LED Pins
LED_R = 11	# Refers to the pins connected to LED in Raspberry Pi
LED_G = 13
LED_B = 16
 
GPIO.setup(LED_R,GPIO.OUT)	# Setup GPIO pins to output mode
GPIO.setup(LED_G,GPIO.OUT)
GPIO.setup(LED_B,GPIO.OUT)

"""
* Function Name : speed_control
* Input : Duty - Used to change the speed of motor turning using PWM
* Output : Void
* Logic : PWM used to change the speed of motor when needed by changing the Duty Cycle
* Example Call : speed_control(40)- turns the forward motors at 40% of the maximum possible speed. 
"""
def speed_control(Duty):
	left_DutyA.ChangeDutyCycle(Duty)		# Since speed control is need only when robot is moving forward, only duty cycle for forward motors needs to be changed
	right_DutyA.ChangeDutyCycle(Duty)
	#left_DutyB.ChangeDutyCycle(Duty)
	#right_DutyB.ChangeDutyCycle(Duty)


"""
* Function Name : make_sharp_turn
* Input : roi_img -- ROI masked binary image		
* Output : Void
* Logic : To check whether the robot has to take a sharp left or sharp right turn , masked ROI image is check to find the contours of maximum area ( the black line ) and its bounding rectangle is found .
	  If the bounding rectangle has very small x and y co-ordinates , this means it is a sharp left turn , else it's sharp right turn.

* Example Call : make_sharp_turn(roi_mask)
"""
def make_sharp_turn(roi_img):
	x,y = roi_img.shape
	_,contours,heirarchy = cv2.findContours(roi_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)		# Find contours of the masked image

	if len(contours)>0:	# Boundary Check
		# To find contour with maximum area i.e. Black track
		ci=0
		max_area=-1
		for i in range(len(contours)):
			area=cv2.contourArea(contours[i])
			if area>max_area and area>200:	# Neglect noisy parts of image
				ci=i		

		bx,by,w,h=cv2.boundingRect(contours[ci])	# Calculate bounding rectangle of the black line
		if bx<=10 and by<=10 :		# If co-ordinates of the bounding rectangle are small , it's a left sharp turn
			print "Left sharp turn"	
			sharp_left()	# Call function to take the sharp turn using differential motor control
	  		# On sharp turn , speed of motor is very high due to differential motor control , so manually stop the motor after small interval of time, else it might lose track the black track
			sleep(0.5)	# Call function to take turn for 0.5 seconds
			stop()		# Call function to stop the motor

		else:		# Else it is a sharp right turn
			print "Right sharp turn"
			sharp_right()	# Call function to take sharp right turn
			sleep(0.5)	# Stop motor after small time . Motor will turn enough to think of the turn as normal curved turn instead of sharp turn.
			stop()		# Call function to stop the motor


"""
* Function Name : sharp_left
* Input : None
* Output : Void
* Logic : Uses differential motor control action for sharp turns i.e one motor (right) moves forward and other (left) moves backward
* Example Call : sharp_left() - Left motor moves backward and right motor moves forward
"""
def sharp_left():
	left_DutyA.stop()	# Stop forward_left
	left_DutyB.start(35)	# Start reverse_left with 35% of total speed  
	right_DutyB.stop()	# Stop reverse_right
	right_DutyA.start(35)	# Start forward_right with 35% of total speed 

"""
* Function Name : sharp_right
* Input : None
* Output : Void
* Logic : Uses differential motor control action for sharp turns i.e one motor (right) moves backward and other (left) moves forward 
* Example Call : sharp_right() - Right motor moves forward and left motor moves backward
"""
def sharp_right():
	left_DutyB.stop()	# Stop reverse_left
	left_DutyA.start(35)	# Start forward_left with 35% of total speed
	right_DutyA.stop()	# Stop forward_left
	right_DutyB.start(35)	# Start reverse_left with 35% of total speed

"""
* Function Name : forward
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in forward direction
* Example Call : forward() - Left and right motors move in forward direction
"""
def forward():
	left_DutyB.start(0)		# Start reverse_left motor with 0 speed 
	right_DutyB.start(0)		# Start forward_right with 0 speed 	( Used so that any previous state of the forward_right motor does no affect the current state)
	left_DutyA.start(30)		# Start forward_left motor with 30 speed
	right_DutyA.start(30)		# Bot goes forward
	GPIO.output(left_motorA,GPIO.HIGH)	# Set respective pins to high 
        GPIO.output(right_motorA,GPIO.HIGH)
	GPIO.output(left_motorB,GPIO.LOW)
	GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : backward
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in backward direction
* Example Call : backward() - Left and right motors move in forward direction
"""
def backward():
	# Used in case bot goes off course a little bit and does not detect any black line contours , it can go backward to the track again
	left_DutyA.stop()	# Stop forward_left 
	right_DutyA.stop()	# Stop reverse_left
	left_DutyB.start(20)	# Start reverse_left motor with 20 speed 
	right_DutyB.start(20)	# Bot moves backward
	GPIO.output(left_motorB,GPIO.HIGH)	# Set respective pins to high 
        GPIO.output(right_motorB,GPIO.HIGH)
	GPIO.output(left_motorA,GPIO.LOW)
	GPIO.output(right_motorA,GPIO.LOW)
"""
* Function Name : right
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to motor so that robot moves in right direction
* Example Call : right() - Left motor moves forward and right motor moves reverse with small speed to allow smaller turning radius
"""
def right():
	left_DutyB.stop()	# Stop reverse_left
	left_DutyA.start(30)	# To move right, forward_left is set to high
	right_DutyA.stop()	# Stop forward_left
	right_DutyB.start(10)	# Reverse_right set to quite lower speed, allows small turning radius for the robot
	#sleep(0.01)
	GPIO.output(left_motorA,GPIO.HIGH)	# Set respective pins to high 
        GPIO.output(right_motorA,GPIO.LOW)
	GPIO.output(left_motorB,GPIO.LOW)
	GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : left
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in left direction
* Example Call : left() - Left motor moves reverse with small speed and right motor moves forward to allow smaller turning radius
"""
def left():
	left_DutyA.stop()	# Stop forward_left
	left_DutyB.start(10)	# Reverse_left is set to lower speed, to allow small turning radius
	right_DutyB.stop()	# Stop reverse_left
	right_DutyA.start(30)	# To move left, forward_right is set to high
	#sleep(0.01)
	GPIO.output(left_motorA,GPIO.LOW)	# Set respective pins to high 
        GPIO.output(right_motorA,GPIO.HIGH)
	GPIO.output(left_motorB,GPIO.LOW)
	GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : stop
* Input : None
* Output : Void
* Logic : Used to stop the robot by setting all motor pins to active low
* Example Call : stop() - Stop both left and right motors
"""
def stop():
	left_DutyA.stop()	# Stop the forward_left motor
	left_DutyB.stop()	# Stop the reverse_left motor
	right_DutyA.stop()	# Stop the forward_right motor
	right_DutyB.stop()	# Stop the reverse_right motor	(Completely stops both the motors)
	GPIO.output(left_motorA,GPIO.LOW)	# Set the respective gpio motor pins to low
	GPIO.output(left_motorB,GPIO.LOW)
	GPIO.output(right_motorA,GPIO.LOW)
	GPIO.output(right_motorB,GPIO.LOW)
	GPIO.output(left_motorE,GPIO.LOW)
	GPIO.output(right_motorE,GPIO.LOW)


"""
* Function Name : modulus
* Input : Two numbers num1 and num2
* Output : Returns the modulus of two numbers
* Logic : Returns the positive of the difference of the two numbers . Function used during direction control
* Example Call : modulus(10,20) returns 10
"""
def modulus(num1,num2):
	if num1>=num2:	# Number 1 is greater than number 2
		return (num1-num2)		#Return the modulus (postive of difference) of two numbers.
	else :	# Number 2 is greater than number 1
		return (num2-num1)


"""
* Function Name : direction_control

* Input : 1. current_roi_front - The centroid of the black line detected in the front part of roi
	  2. corner_roi_front - The leftmost point of the black line detected in the front part of roi
	  3. current_roi_bottom - The centroid of the black line detected in the bottom part of roi ( 2 ROIs are used to allow better control of the speed of the robot)
	  4. corner_roi_bottom - The leftmost point of the black line detected in the front part of roi

	Note : All the above values are only the x-coordinates of the points as y-coordinates are not needed for direction control

* Output : Void

* Logic : The function is used to check the direction in which the bot has to move currently . The distance between leftmost point of black line to the centroid of the line (of bottom ROI) is used to 	  follow the required direction, and the displacement between the leftmost points of the top and bottom ROI is used to control speed of forward motors... So, Front ROI is used for speed control 
	  and bottom ROI is used for direction control.

* Example Call : direction_control(200,120,205,125)

"""
def direction_control(current_roi_front,corner_roi_front,current_roi_bottom,corner_roi_bottom):

	try:	# Executes if no expection occurs
		width = modulus(current_roi_bottom,corner_roi_bottom)	# Compute distance (modulus) between leftmost point and centroid of black line (Basically half of width of the black line)
		displace_corner = modulus(corner_roi_front,corner_roi_bottom)	# Compute distance (modulus) between leftmost points (x-coordinate only) of front and bottom ROI
		#print width,displace_corner

		if (width>=64 and width<=69) or displace_corner<=5:	# Experimental Values for forward Direction -- Depend on the size of PiCam window
			print "Forward"
			forward()	# Call function to move motors in forward direction

			"""if displace_corner <= 5:	# If it is perfectly straight line , the motor moves faster using speed control

			print "Fast Forward"		# Not used parctically as both the motors do not move with equal speeds , so it's not possible to get perfectly straight line for long intervals
							# And might cause robot to lost sense of track if motor is too fast.

			speed_control(40)	# Call function to increase the speed of the motor"""
	
			#else:
			speed_control(20)	# Call function to control speed of motor

			#sleep(0.5)
			#stop()

		# Left or right direction depend on position of leftmost point from the center of the PiCam window ( Currently used window -- (480,320)
		elif corner_roi_bottom>=240:		# If leftmost point is to the right of the center of the picam window, robot moves to right
			print "Right"
			right()		# Call function to move the robot to right
			#sleep(0.5)
			#stop()

		elif corner_roi_bottom<240:
			print "Left"		# If leftmost point is to the left of the center of the picam window, robot moves to left
			left()		# Call function to move the robot to left
			#sleep(0.5)
			#stop()


	finally:	# Pass if exception occurs
		pass

"""
* Function Name : create_roi
* Input : 1. img - Input image captured by the picam
	  2. x1_roi , x2_roi - x-coordinates needed for ROI on the image 
	  3. y1_roi , y2_roi - y-coordinates needed for ROI on the image
	  4. thresh_type - Type of thresholding needed on the image ( BINARY_INV for black line detection and BINARY for inverted plane or white line detection ).

* Output : Returns a tuple so that the function returns multiple values. These include:
	   1. roi_img - Masked binary image needed to detect inverted planes.
	   2. bx - Leftmost point of the black line
	   3. cx - Centroid of the black line
	   4. max_area - Maximum area of black line needed to detect sharp turns and zone indicator

* Logic - The function creates ROI at the specified  co-ordinates values on the image by using contours approximation techniques. Blurring methods along with erosion techniques are used to reduce noise.
	  From the said contours , maximum area contour is obtained so that black line can be detected with precision.

* Example Call - create_roi(img,x-50,x-25,y/20,y-y/20,1) - where x and y are the dimensions of the img captured by picam and 0 represents thresholding using BINARY_INV technique.
"""
def create_roi(img,x1_roi,x2_roi,y1_roi,y2_roi,thresh_type):
	roi = img[x1_roi:x2_roi,y1_roi:y2_roi]		# Crop the image to get only ROI
	roi_gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)	# Convert to grayscale for accurate detection
	roi_gray=cv2.GaussianBlur(roi_gray,(5,5),0)	# Blur
	ret,roi_mask=cv2.threshold(roi_gray,50,255,thresh_type)	# Thresholding using required thresholding type BINARY or BINARY_INV as per black line or white line (inverted plane) traversal
	roi_mask=cv2.GaussianBlur(roi_mask,(5,5),0)	# Blur the threshed image
	roi_mask=cv2.erode(roi_mask,np.array([15,15],np.uint8),iterations=1)	# Erosion to remove noise
	roi_mask=cv2.medianBlur(roi_mask,5)	# Blur
	#cv2.imshow("ROI",roi_mask)
	#cv2.rectangle(img,(y1_roi,x1_roi),(y2_roi,x2_roi),(255,0,0),2)
	roi_img=roi_mask.copy()				# Make a copy of roi (so that it can be returned)as contour detection makes changes to the original image
	_,contours,heirarchy = cv2.findContours(roi_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)		# Contour detection

	if len(contours)>0:		#Boundary check
		# To find contour with maximum area
		ci=0
		max_area=-1
		for i in range(len(contours)):
			area=cv2.contourArea(contours[i])
			if area>max_area and area>1000:		# Boundary check
				ci=i

		max_area=cv2.contourArea(contours[ci])
		bx,by,w,h=cv2.boundingRect(contours[ci])		# Bounding rectangle to obtain the leftmost part of the black line
		#cv2.rectangle(roi,(bx,by),(bx+w,by+h),(0,255,0),2)		#Changes made to ROI are automatically reflected back to the original image (and vice versa). No need to set some offset

		# To find centroid of the black line using Moments
		M=cv2.moments(contours[ci])
		if M['m00'] != 0:		# Boundary check
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
		else :			# To avoid the "Division by zero" error if no contour is detected
			cx=0
			cy=0
		#cv2.circle(roi,(cx,cy),3,(0,0,255),-1)
		return (roi_img,bx,cx,max_area)		# Returns a tuple 
	else :
		return (roi_img,0,0,0)		# If no contour is present, return threshed image, rest all values are zero.


"""
* Function Name : check_if_black
* Input : mask - Threshed image obtained from create_roi function
* Output : 1 (Image is black) or 0 (Image is False)
* Logic : The function checks if the whole threshed ROI image is black or not . Helps in the detection of the Zone Indicators
* Example Call : check_if_black(roi_mask) - where roi_mask is the threshed binary image
"""
def check_if_black(mask):
	# Take a bitwise NOT of the binary image . As for black line detection, we thresh black color, hence in binary image, black color turns to white and rest colors are black.
	mask = cv2.bitwise_not(mask)		
	if cv2.countNonZero(mask) == 0:		# Check if the now black image is Completely black or not
		print "Black Image"
		return 1	# If image is completely black
	else :
		return 0	# If image is not completely black

"""
* Function Name : check_plane
* Input : roi_img - Threshed binary image obtained from create_roi function
* Output : 1 (Inverted Plane Present) or 0 (Inverted Plane Absent)
* Logic : Since for inverted Plane traversal, we need to follow a white line instead of a black one, it's presence is detected if threshed image contains more than 1 appreciable black contours of large 
	  area.
* Example Call : check_plane(roi_mask)
"""
def check_plane(roi_img):
	_,contours,heirarchy = cv2.findContours(roi_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)		# Find contours on threshed image
	inverted_plane = 0
	for i in contours:
		if cv2.contourArea(i)>3000:		# If more than 1 contour is present with such a large area , it means that invereted plane is present
			inverted_plane += 1

	if inverted_plane > 1:
		#print "Inverted Plane Present"
		return 1		# Inverted Plane present

	else:
		return 0		# Inverted Plane absent





################## Overlay Functions #################

"""
* Function Name : blend_transparent
* Input : 1. face_img - The image on which overlay has to be done
	  2. overlay_t_img - The image to be overlayed
* Output : Overlayed image
* Logic : Input image is overlayed with the overlay_t_img by converting adding both the image with desired weights ( for transparency ). For overlay, both the images should have the same dimenstions.
* Example Call : blend_transparent(plantation[x1:x2,y1:y2],overlay)
"""

def blend_transparent(face_img, overlay_t_img):
    # Split out the transparency mask from the colour info
    overlay_img = overlay_t_img[:,:,:3] # Grab the BRG planes
    overlay_mask = overlay_t_img[:,:,3:]  # And the alpha plane
    # Again calculate the inverse mask
    background_mask = 255 - overlay_mask
    # Turn the masks into three channel, so we can use them as weights
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # Create a masked out face image, and masked out overlay
    # We convert the images to floating point in range 0.0 - 1.0
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    # And finally just add them together, and rescale it back to an 8bit integer image    
    return np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))

### Hard Coded Co-ordinates for plantation Zones ###

"""  # Plantation Zone 1:
	max_count = 6		# Maximum possible count on the plantation zone for given size
	overlay_shift_x = 30	# Shifts overlay around the center of PZ depending on count ... Smart overlaying to allow better covering of PZ area
	overlay_shift_y = 0
	start_x = 420		# Basically represents the centre of PZ. All the overlays formed are symmetrical about this point
	start_y = 285
	displace_x = 65		# Displacement between two overlays
	displace_y = +20
	overlay_x = 60		# Size of the overlay
	overlay_y = 60	
	overlay = "Set yourself"
   
   # Plantation Zone 2:
	max_count = 4
	overlay_shift_x = 18
	overlay_shift_y = -20
	start_pos_x = 130
	start_pos_y = 245
	displace_x = 28
	displace_y = -40
	overlay_size_x = 35
	overlay_size_y = 35
	overlay = "Set yourself"

   # Plantation Zone 3:
	max_count = 4
	overlay_shift_x = 25		
	overlay_shift_y = 0
	start_pos_x = 310
	start_pos_y = 200
	displace_x = 43			
	displace_y = -10
	overlay_x = 35
	overlay_y = 35 
	overlay = "Set yourself"

  # Plantation Zone 4:
	max_count = 5
	overlay_shift_x = 20 
	overlay_shift_y = 0
	start_pos_x = 575
	start_pos_y = 215
	displace_x = 45
	displace_y = -10
	overlay_x = 30
	overlay_y = 30
	
	overlay = "Set yourself"
	"""

""" 
* Function Name : plantation_zone_detect
* Input : zone_number - The current zone on which overlay has to be done
* Output : None
* Logic : Function checks the Zone on which overlay has to be done and calls the create_overlay function with respective co-ordinates of each Zone as seen above
* Example Call : plantation_zone_detect(2) : Overlays image on the Zone 2
"""
def plantation_zone_detect(zone_number):
	
	if zone_number == 1:	# PZ- 1
		create_overlay(color_marker[0],color_marker[1],color_marker[2],6,30,0,420,285,65,20,60,60)	# color_marker list contains the shape, size and number of contours detected

	elif zone_number == 2:	# PZ - 2
		create_overlay(color_marker[0],color_marker[1],color_marker[2],4,18,-20,130,245,28,-40,35,35)	# Hard-coded values

	elif zone_number == 3:  # PZ - 3
		create_overlay(color_marker[0],color_marker[1],color_marker[2],4,25,0,310,200,43,-10,35,35)
	
	elif zone_number == 4: # PZ-4
		create_overlay(color_marker[0],color_marker[1],color_marker[2],4,20,0,575,215,35,-5,30,30)

	else :		# Boundary check
		pass

"""
* Function Name : create_overlay
* Input : 1. shape - The shape to be overlayed
	  2. color - The color to be overlayed
	  3. count - The number of overlays to be done
	  4. max_count - Maximum possible overlays on the particular Plantation Zone
	  5. overlay_shift_x,overlay_shift_y - Shifts overlay around the center of PZ depending on count . This method is used to allow better covering of PZ area
	  6. start_pos_x, start_pos_y - Represents the centre of PZ. All the overlays formed are symmetrical about this point
	  7. displace_x, displace_y - The x and y displacements between any two overlays
	  8. overlay_size_x,overlay_size_y - The size of the overlay

* Output : None
* Logic : A single function is used to overlay on any of the plantation zone using these input arguments. For a single overlay, it occurs at the center of PZ , as count increases , all overlays are 
	  symmetric to the center rather than starting from any side. This provides better covering of plantation zone
* Example Call : create_overlay(Triangle,Red,2,4,25,0,310,200,43,-10,35,35)
"""
def create_overlay(shape,color,count,max_count,overlay_shift_x,overlay_shift_y,start_pos_x,start_pos_y,displace_x,displace_y,overlay_size_x,overlay_size_y):
	# Overlay Images for respective Color and shape provided by the E-Yantra team
	if color == "Red":

		if shape == "Circle":
			overlay = carnation.copy()
		
		elif shape == "Triangle":
			overlay =tulipred.copy()

		elif shape == "Square":
			overlay = gerber.copy()

	elif color == "Green":
		
		if shape == "Circle":
			overlay = lily_double.copy()

		elif shape == "Triangle":
			overlay = hydrangeayellow.copy()

		elif shape == "Square":
			overlay = sunflower.copy()

	elif color == "Blue":

		if shape == "Circle":
			overlay = orchid.copy()

		elif shape == "Triangle":
			overlay = tulipred.copy()

		elif shape == "Square":
			overlay = hydrangeablue.copy()

	if count<1 :	# Boundary check
		pass
	
	else:	
		# If count is greater than max_count , the size of all the overlays is halved so that no error might be generated if overlay crosses image dimensions			
		size_decay = (int)((count-1)/(max_count))+1		
		start_x = start_pos_x - overlay_shift_x*(count-1)/size_decay	# Causes overlays to be symmetric about the center of plantation zone
		if count == 1:		
			initial_shift_y = 0
		else:
			initial_shift_y = 1
		temp_start_y = start_pos_y - (overlay_shift_y)*initial_shift_y/size_decay 	# Causes overlay to be symmetric about the center of plantation zone
		overlay_x=(int)(overlay_size_x)/(size_decay)		# If size_decay increases, i.e. Overlay count is high , the size of each overlay is decreased by size_decay times
		overlay_y=(int)(overlay_size_y)/(size_decay)
		overlay = cv2.resize(overlay,(overlay_x,overlay_y))	# To obtain overlay of needed size
		for i in range(count):
			# So that not all the overlays are in a straight line . Causes continuous overlays to be displaced from each other in terms of y - coordinates
			start_y = temp_start_y + displace_y*(i%2)/size_decay	
			# Overlay the image
			plantation[start_y-overlay_y:start_y,start_x:start_x+overlay_x] = blend_transparent(plantation[start_y-overlay_y:start_y,start_x:start_x+overlay_x],overlay)
			# Shift x-coordinate so that overlays do not overlap
			start_x = start_x + (displace_x/size_decay)





################## Color Marker Detection and LED glowing Functions #################

"""
* Function Name : color_marker_detect
* Input : 1. image : Image captured by the picam for color marker detection
	  2. color : Mask for that respective color - Red, Blue or Green
* Output : None
* Logic : Masks the image with particular color to allow detection of shape and count of the color marker
* Example Call : color_marker_detect(img,Red)
"""
def color_marker_detect(image,color):		# Detect contours for masked images
	image = cv2.GaussianBlur(image,(3,3),0)	# Blur the image
	image = cv2.medianBlur(image,3)
	dilate = cv2.dilate(image,np.array([3,3],np.uint8),iteration = 1)	# Using closing to remove noise
	closing = cv2.erode(dilate,np.array([3,3],np.uint8),iteration = 1)
	canny = cv2.Canny(closing,100,200)
	
	_,contours, heirarchy = cv2.findContours(canny,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	color_marker=[]		# Contains the shape, size and number of color marker detected
	global end_run_glow
	global zone_number
	if len(contours)>0:		# Boundary check
		overlay_count = 0	# Initially no overlay is present
		color_marker.insert(0,color)	# color_marker list contains the shape, size and number of color markers detected
		end_run_glow.append(color)
	       	for i in contours:
			if cv2.contourArea(i)>200:		# Boundary check
				"""M = cv2.moments(i)		# Find centroid of the contour
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])"""
	
				vertices = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)		# To find the total number of vertices in the image
	
				if len(vertices) == 3: # It is a triangle
					#cv2.drawContours(img,[i],0,(255,0,0),2)
					if len(color_marker)==1:
						color_marker.extend("Triangle",1)	# Considering only single type of shape is present at a single zone , and only the count needs to be changed
					
					if color_marker[1]=="Triangle":		# So, if a 2 square and 3 triangle are detected, the list will contain only Triangle with count 3
						overlay_count += 1
						color_marker[2]=overlay_count
	
				elif len(vertices) == 4: # It is a square
					#cv2.drawContours(img,[i],0,(255,0,0),2)
					if len(color_marker)==1:
						color_marker.extend("Square",1)

					if color_marker[1]=="Square":
						overlay_count += 1
						color_marker[2]=overlay_count
				
				else : # It is a circle
					#cv2.drawContours(img,[i],0,(255,0,0),2)
					if len(color_marker)==1:
						color_marker.extend("Circle",1)
					
					if color_marker[1]=="Circle":
						overlay_count += 1
						color_marker[2]= overlay_count

		print color_marker
		if len(color_marker) == 3:	# Boundary Check
			zone_number += 1	# So that next time , different Zone is selected
			plantation_zone_detect(zone_number)	# Calls function for overlaying on the zone
			led_glow(color_marker)		# Call function to glow LED of respective color and count
			color_marker=[]		# Resets the list

"""
* Function Name: led_stop
* Input : None
* Output : None
* Logic : Sets all the GPIO pins referring to LED to low. Function used for glowing of LED
* Example Call : led_stop()
"""
def led_stop():
	GPIO.output(LED_R,GPIO.LOW)
	GPIO.output(LED_G,GPIO.LOW)
	GPIO.output(LED_B,GPIO.LOW)

"""
* Function Name: led_glow
* Input : color_marker
* Output : None
* Logic : Used to blink the LED depending on the color and count of color markers detected. Blinking is done by setting pin to high for some time and then setting the pins to low for 1 second to blink.
* Example Call : led_glow()
"""
def led_glow(color_marker):
	count = color_marker[2]
	while count>0:
		sleep(1)	# Blinks the LED for 1 second
		if color_marker[0]=="Red":	# Select the color of blinking
			GPIO.output(LED_R,GPIO.HIGH)
		elif color_marker[0]=="Green":
			GPIO.output(LED_G,GPIO.HIGH)
		elif color_marker[0]=="Blue":
			GPIO.output(LED_B,GPIO.HIGH)
		sleep(0.5)
		led_stop()	# Stop the LED glow to allow blinking
		count -= 1

"""
* Function Name: final_glow
* Input : None
* Output : None
* Logic : Used to glow the LED with respective colors when the end of the run is detected
* Example Call : final_glow()
"""
def final_glow():
	count = len(end_run_glow)
	while count>0:
		sleep(1)	# Blinks the LED for 1 second
		if color_marker[0]=="Red":	# Select the color of blinking
			GPIO.output(LED_R,GPIO.HIGH)
		elif color_marker[0]=="Green":
			GPIO.output(LED_G,GPIO.HIGH)
		elif color_marker[0]=="Blue":
			GPIO.output(LED_B,GPIO.HIGH)
		sleep(0.5)
		led_stop()	# Stop the LED glow to allow blinking
		count -= 1




################## Interfacing	 #################

# Read all the images of seedlings

plantation = cv2.imread("Plantation.png")
assorted = cv2.imread("assorted.png",cv2.IMREAD_UNCHANGED)	
carnation = cv2.imread("carnation.png",-1)
gerber = cv2.imread("gerber.png",-1)
hibiscusred = cv2.imread("hibiscusred.png",-1)
hibiscusyellow = cv2.imread("hibiscusyellow.png",-1)
hydrangeablue = cv2.imread("hydrangeablue.png",-1)
hydrangeayellow = cv2.imread("hydrangeayellow.png",-1)
lilac = cv2.imread("lilac.png",-1)
lily = cv2.imread("lily.png",-1)
lily_double = cv2.imread("lily-double.png",-1)
marigold = cv2.imread("marigold.png",-1)
morningglory = cv2.imread("morningglory.png",-1)
orchid = cv2.imread("orchid.png",-1)
poinsettia = cv2.imread("poinsettia.png",-1)
rosesred = cv2.imread("rosesred.png",-1)
rosesyellow = cv2.imread("rosesyellow.png",-1)
sunflower = cv2.imread("sunflower.png",-1)
tulipblue = cv2.imread("tulipblue.png",-1)
tulipred = cv2.imread("tulipred.png",-1)	

# Blue Mask
lower_blue = np.array([80,10,20])
upper_blue = np.array([130,255,255])

# Green Mask
lower_green = np.array([40,20,20])
upper_green = np.array([90,255,255])

# Red Mask
lower_red = np.array([0,130,90])
upper_red = np.array([20,255,255])

zone_number=0	# Initially zone number is 0
end_run_glow = []		# Contains the order of colors of color markers appearing in PZ. Used at the end of the run
inverted_plane_passed=0 	# As inverted plane has not passed

# Initialises the PiCamera
camera = PiCamera()
camera.resolution = (480,320)		# Set the resolution of PiCam
camera.framerate = 5		# Set the frame rate of PiCam
rawCapture = PiRGBArray(camera, size=(480,320))		# Capture raw Image
time.sleep(2)		# Time for PiCam to warm up
gpio_set()		# Call function to setup GPIO pins
#skip_frames=0
check_zone=0		# Initially no zone is present
backward_turn_times=0

for frame in camera.capture_continuous(rawCapture, format = "bgr" , use_video_port=True):
	img=frame.array		# Capture the image
	"""if skip_frames<2:		# If robot needs to be pushed in forward direction 
		print "Forward"
		forward()
		sleep(0.5)
		stop()
		skip_frames +=1
		rawCapture.truncate(0)
		continue"""

	#else :
	x,y,c=img.shape		# Get the dimensions of the image captured

	cv2.imshow("Plantation",plantation)     # Show the plantation image ... Slows the processing speed if imshow is used for every frame

	img=cv2.GaussianBlur(img,(5,5),0)	# Blur the image
	if inverted_plane_passed!=1:            # Black line detected -- No inverted Plain
	
		(roi_img,left_roi_1x,roi_1x,contour_area_1x)=create_roi(img,x-30,x-5,y/20,y-y/20,thresh_type=1)		# Call function to obtain the ROI at the bottom of the image
		(_,left_roi_2x,roi_2x,contour_area_2x)=create_roi(img,25,50,y/20,y-y/20,thresh_type=1)	# ROI at top of image

	else :          # Inverted Plane detected
		(roi_img,left_roi_1x,roi_1x,contour_area_1x)=create_roi(img,x-30,x-5,y/20,y-y/20,thresh_type=0)	# Inverted Plane means white line following
		print left_roi_1x,roi_1x,contour_area_1x
		#cv2.imshow("Image",roi_img)
		#cv2.waitKey(1000)
		(_,left_roi_2x,roi_2x,contour_area_2x)=create_roi(img,25,50,y/20,y-y/20,thresh_type=0)  # Change ROI values to match white line following
		
		if contour_area_1x>6000:        # After inverted plane is detected, next zone represents the end of run
			forward()		# Call function to reach the zone marking the end of run
			sleep(0.5)
			stop()		#End of Run
                        final_glow      # Call function to glow LED in order of color markers detected for each zone
			break

			
	inverted_plane_check=check_plane(roi_img)	# Call function to check if inverted plane is present

	
	if inverted_plane_check == 1 and inverted_plane_passed!=1:      # Inverted Plain comes the first time
		print "Inverted Plane Present"		# Inverted Plane Present
		sleep(1)	# Wait for 1 second
		# Obtain ROI again with thresh type as BINARY instead of BINARY_INV as inverted plane traversal needs white line traversal
		inverted_plane_passed=1
		#forward()
		#sleep(0.5)
		#stop()
		rawCapture.truncate(0)
		continue

	else:
		if contour_area_1x > 5000 or check_zone == 1 or check_if_black(roi_img)==1:  # If large area is detected or complete image is black, either a zone is present or it is a sharp turn
		# To check if zone is present or sharp turn, robot moves forward by 0.25 seconds , if still a large area is detected, it is zone, else it is a sharp turn
			if check_zone == 0:		 
				print "Zone or Sharp Turn"
				print contour_area_1x
				# Store the initial frame . This frame is used if PiCam detects it is a sharp turn , but it has moved forward and has detected no contour, so it will work on this 						# current frame
				take_turn_image = roi_img.copy()
				stop()
				sleep(1)
				forward()
				sleep(0.25)
				stop()
				check_zone = 1                # This statement will not be called the next time the bot is searching whether it is a zone or a sharp turn
				rawCapture.truncate(0)
				continue		# Move to next frame so that PiCam can detect the new image captured by PiCam to check if it is a Zone or not
		
			if contour_area_1x > 4000 or check_if_black(roi_img)==1 :       # After going forward, still a large black contour is detected , means it is a zone
				print "Zone"
				stop()
				sleep(1)
				hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)		# Convert to HSV for better color detection
				maskr = cv2.inRange(hsv,lower_red,upper_red)	# Red Mask

				maskg = cv2.inRange(hsv,lower_blue,upper_blue)	# Blue Mask

				maskb = cv2.inRange(hsv,lower_green,upper_green)	# Green Mask
						
				color_marker_detect(maskr,'Red')	# Call function to detect red colored color markers
				color_marker_detect(maskb,'Blue')	# Call function to detect blue colored color markers
				color_marker_detect(maskg,'Green')	# Call function to detect green colored color markers

				sleep(1)	# Wait 1 second after LED glow before moving forward
				forward()
				sleep(1)		# So that robot can successfully get past the Zone without going off track
				check_zone=0

				# If a Zone is detected and 
				if zone_number == 5 or inverted_plane_passed==1:
					final_glow()
					break
				rawCapture.truncate(0)
				continue
					
			elif contour_area_1x <4000 : 
				print "Sharp Turn"		# It is a sharp turn
				#backward()
				#sleep(0.5)
				make_sharp_turn(take_turn_image)	# Call function to detect if it is a left sharp turn or a right sharp turn
				rawCapture.truncate(0)
				check_zone=0
				continue		# Move to the next frame after taking the sharp turn


	if left_roi_2x ==0 and roi_2x == 0 and contour_area_2x==0 and left_roi_1x==0 and roi_1x==0 and contour_area_1x==0:		# if no contour is detected, move a little backward
		print "Backward"
		backward()
		sleep(0.5)
		stop()
		backward_turn_times +=1
		if backward_turn_times == 6:		# If motor moves backward more than six times, it means that it has gone a little too much off course. So, the program stops
			break
		rawCapture.truncate(0)
		continue
				
	else:
		direction_control(roi_2x,left_roi_2x,roi_1x,left_roi_1x)	# If contour is detected, call function to detect the direction of motion of the robot

	backward_turn_times = 0
	rawCapture.truncate(0)		# Flush out the current frame

cv2.imshow("Plantation",plantation)
cv2.waitKey(1000)
cv2.destroyAllWindows()		#End of Program



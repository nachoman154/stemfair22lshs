 # import the necessary packages

 #General
import time
import math
 
 #Object Tracking
from picamera.array import PiRGBArray
from picamera import PiCamera
from collections import deque

#Data Processing
import cv2
import argparse
import imutils
import numpy as np
import multiprocessing

#Motor Control
import smbus

#Motor Control Methods
class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  #Reset the board with each new run
  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)
	
  #Writing values to each of the registries when running 
  def write(self, reg, value):
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  


  #Reads an unsigned byte from the computer
  def read(self, reg):
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result

  #Set the PWM Frequency for the Motors
  def setPWMFreq(self, freq):
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)


  #Creats an instance of a Channel and lets the computer know that there is a servo present
  def setPWM(self, channel, on, off):
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))


  #Creates the command to pulse the Servo
  def setServoPulse(self, channel, pulse):
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))


  #Define a method to shorten the large part of the code that is repeated consistently
  def start(self,Channel,low,high,step,sleep):
    for i in range(low,high,step):  
      pwm.setServoPulse(Channel,i)   
      time.sleep(sleep)

      #pwm.start(W,X,Y,Z,A)


  def stop(self,channel):
    pwm.setServoPulse(channel,0)
      #pwm.stop(W)

#Create an instance of the motor control
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

#QR CODE PLACEHOLDER VALUES

FollowingQRCodeisFound = False
BlockPickupQRCodeisFound = False

#QR CODE Booleans, to simply track if the process is running.

followingRunning = False
pickupRunning = False

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

def toggleQRCodeBoolean(qrBoolean):
  if qrBoolean == True:
    qrBoolean = False
  elif qrBoolean == False:
    qrBoolean = True
  else:
    print("WTF? (Line 36)")



def QRCodeDetection():
  detector = cv2.QRCodeDetector()

  while True:
      joe = image.copy()
      # get bounding box coords and data
      data, bbox, joe = detector.detectAndDecode(image)
      
      # if there is a bounding box, draw one, along with the data
      if(bbox is not None):
          for i in range(len(bbox)):
              cv2.line(joe, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color=(255,
                      0, 255), thickness=2)
          cv2.putText(joe, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                      0.5, (0, 255, 0), 2)
          if data:
              print("data found: ", data)
              if data == "blueBlockPickup":
                toggleQRCodeBoolean(BlockPickupQRCodeisFound)
              elif data == "redColorFollow":
                toggleQRCodeBoolean(FollowingQRCodeisFound)
              time.sleep(2)
      # display the image preview
      ##cv2.imshow("code detector", img)
      ## if(cv2.waitKey(1) == ord("q")):


#the following code defines the procedure for following a red laser pointer/circle/ball
def redColorFollow():
  # set the bounds for the red color
    laserColorLower = np.array([136, 87, 111], np.uint8)
    laserColorUpper = np.array([180, 255, 255], np.uint8)

  #create a queue
    pts = deque(maxlen=args["buffer"])

    # resize the frame, blur it, and convert it to the HSV
    # color space   
    imageBlur = imutils.resize(image, width=600)
    blurred = cv2.GaussianBlur(imageBlur, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, laserColorLower, laserColorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
      # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
      if radius > 10:
          # draw the circle and centroid on the frame,
          # then update the list of tracked points
          cv2.circle(image, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
          cv2.circle(image, center, 5, (0, 0, 255), -1)
      # update the points queue
      pts.appendleft(center)

#The following code defines the pickup proccess of a blue block
def blueBlockPickup():
  time.sleep(2)
  pwm.start(1,50,2500,10,1)
  pwm.stop()
  

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.2)
  # capture frames from the camera. Everything is done while the camera is running.
if __name__ == "__main__":
  for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


        #define each variable as its own proccess, so we can run these concurrently

        redFollow = multiprocessing.Process(target=redColorFollow)
        bluePickup = multiprocessing.Process(target=blueBlockPickup)
        qrDetect = multiprocessing.Process(target=QRCodeDetection)

        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied 
        
        image = frame.array
        # A copy of the frame if the blurring and such creates issues.
        holdOnToOutput = image.copy()

        #If QR Codes are tracked, run code. A series of placeholder if statements.
        qrDetect.start()

        if FollowingQRCodeisFound == True:
          if followingRunning == False:
          #run laser pointer following
            followingRunning = True
            redFollow.start()
            time.Sleep(2)
          elif followingRunning == True:
            followingRunning = False
            redFollow.terminate()
            time.Sleep(2)
        else:
          print("Following QR not Found!")

        if BlockPickupQRCodeisFound:
          if pickupRunning == False:
          #run laser pointer following
            pickupRunning = True
            bluePickup.start()
            time.Sleep(2)
          elif pickupRunning == True:
            pickupRunning = False
            bluePickup.terminate()
            time.Sleep(2)
        else:
          print("Block Pickup QR not Found!")

        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

cv2.destroyAllWindows()
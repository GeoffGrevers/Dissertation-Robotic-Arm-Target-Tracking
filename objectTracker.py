#######################################################
# University of Leeds, Mechatronics and Robotics BEng
# Written by Geoff Grevers     
# ELEC3875
# Project 18
# Class of 2016-17
#
#######################################################

import threading
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import numpy as np
import cv2
import argparse
from io import BytesIO
from threading import Timer
import paho.mqtt.publish as publish
import logging
import imutils

# Set to SWITCH off the logging change logging.DEBUG to logging.CRITICAL
# NOTE: logging lines have been commented out to improve performance
logging.basicConfig(level=logging.CRITICAL,
                    format='(%(threadName)-9s) %(message)s',)

# initialise camera
my_stream = BytesIO()
camera = PiCamera()
camera.resolution = (480, 320)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(480, 320))

# allow camera to warm up
time.sleep(0.1)

font = cv2.FONT_HERSHEY_SIMPLEX

def get_arguement():
    ap = argparse.ArgumentParser()
    ap.add_argument('track',
                    help='hand, face or object', type=str)
    args = vars(ap.parse_args())

    return args

# pass arguement
args = get_arguement()
track = args['track']

if track == "object":
    
    # define minimum matches
    MIN_MATCH_COUNT = 10
    # declare orb object
    orb = cv2.ORB_create()
    
    global capture
    capture = False

    
######## MQTT section ##########

# when publishing
def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))


Broker = "192.168.0.17"

pub_topic = "dev/test"   # send messages to this topic



###############################################
# Defined class for the timer variables
###############################################
class timer_var(object):
    def __init__(self, val1,val2):
        self._count = val1
        self._expired = val2
    @property
    def count(self):
        return self._count
    @property
    def expired(self):
        return self._expired
    @count.setter
    def count(self,val1):
        self._count = val1
    @expired.setter
    def expired(self,val2):
        self._expired = val2
    
#############################################
# Define count down timer
#############################################
def timer_thread(timer,):
    
    #logging.debug('timer thread starting at %s', time.strftime("%a, %d %b %Y %H:%M:%S +0000") )
    #
    timer.count -= 1
    if timer.count != 0:
        Timer(1, timer_thread, (timer,)).start() #Set this timer to 1 second
    else:
        timer.expired = True                    #Time expired
        timer.count = 5                         # reinitialise
    #logging.debug('timer thread Ending status= %s  Count=%s',timer.expired, timer.count)

###############################################
# Defined class for the transmitting variables
###############################################
class transmit_data(object):
    _corr = (0,0)
    _finger = (0)
    @property
    def corr(self):
        return self._corr
    @property
    def finger(self):
        return self._finger
    @corr.setter
    def corr(self,val):
        self._corr = val
    @finger.setter
    def finger(self,val):
        self._finger = val


#############################################
# Define transmit thread
#############################################        
def transmit_thread(event,coor):
    #logging.debug('transmit_thread starting')
    exit_thread = False
    while not exit_thread:
        event_is_set = event.wait() #Thread will will await until event set()
        x,y = coor.corr
        fin = int(coor.finger)
        x = int(x)
        y = int(y)
        finger = int(coor.finger)
        if (finger == 99):
            exit_thread = True
        x_byte = x.to_bytes(2, byteorder='big')
        y_byte = y.to_bytes(2, byteorder='big')
        fingers_byte = finger.to_bytes(1, byteorder='big')
        msgs = [{'topic':"dev/test",'payload': bytearray([x_byte[0],x_byte[1],y_byte[0],y_byte[1],fingers_byte[0]])}]
        publish.multiple(msgs, hostname=Broker)
        #logging.info('X=%s Y=%s Finger %s',x,y,fin)
    #logging.debug('transmit_thread exit')


###########################################
# Define calibration function
###########################################
def calibration(image,phase_num):
    #logging.debug('calibration starting')
    if not screenshots_complete:
        # show where areas of interest are
        if phase_num == 1:
                cv2.rectangle(image,(x1,y1),(x2,y2),(255,255,0),1)
                cv2.rectangle(image,(x3,y3),(x4,y4),(255,255,0),1)
                cv2.rectangle(image,(x5,y5),(x6,y6),(255,255,0),1)
            
        elif phase_num == 2:
                cv2.rectangle(image,(x7,y7),(x8,y8),(255,255,0),1)
                cv2.rectangle(image,(x9,y9),(x10,y10),(0,0,255),1)      # centre square
                cv2.rectangle(image,(x11,y11),(x12,y12),(255,255,0),1)

        elif phase_num == 3:
                cv2.rectangle(image,(x13,y13),(x14,y14),(255,255,0),1)
                
        elif phase_num == 4:
                cv2.rectangle(image,(x15,y15),(x16,y16),(255,255,0),1)

    cv2.putText(image, 'place hand on squares', (10,25), font, 1, (255,255,0), 2)
    cv2.putText(image, str(timer.count), (100,295), font, 1, (255,255,0), 2)
    #logging.debug('calibration End')    

###############################################
# Defined class for upper and lower variables
###############################################
class boundary(object):
    _lower_roi1 = np.array([0,0,0])
    _upper_roi1 = np.array([0,0,0])
    _lower_roi2 = np.array([0,0,0])
    _upper_roi2 = np.array([0,0,0])
    _lower_roi3 = np.array([0,0,0])
    _upper_roi3 = np.array([0,0,0])
    _lower_roi4 = np.array([0,0,0])
    _upper_roi4 = np.array([0,0,0])
    _lower_roi5 = np.array([0,0,0])
    _upper_roi5 = np.array([0,0,0])
    _lower_roi6 = np.array([0,0,0])
    _upper_roi6 = np.array([0,0,0])
    _lower_roi7 = np.array([0,0,0])
    _upper_roi7 = np.array([0,0,0])
    _lower_roi8 = np.array([0,0,0])
    _upper_roi8 = np.array([0,0,0])

    @property
    def lower_roi1(self):
        return self._lower_roi1
    @property
    def upper_roi1(self):
        return self._upper_roi1
    @property
    def lower_roi2(self):
        return self._lower_roi2
    @property
    def upper_roi2(self):
        return self._upper_roi2
    @property
    def lower_roi3(self):
        return self._lower_roi3
    @property
    def upper_roi3(self):
        return self._upper_roi3
    @property
    def lower_roi4(self):
        return self._lower_roi4
    @property
    def upper_roi4(self):
        return self._upper_roi4
    @property
    def lower_roi5(self):
        return self._lower_roi5
    @property
    def upper_roi5(self):
        return self._upper_roi5
    @property
    def lower_roi6(self):
        return self._lower_roi6
    @property
    def upper_roi6(self):
        return self._upper_roi6
    @property
    def lower_roi7(self):
        return self._lower_roi7
    @property
    def upper_roi7(self):
        return self._upper_roi7
    @property
    def lower_roi8(self):
        return self._lower_roi8
    @property
    def upper_roi8(self):
        return self._upper_roi8
    @lower_roi1.setter
    def lower_roi1(self,val):
        self._lower_roi1 = val
    @upper_roi1.setter
    def upper_roi1(self,val):
        self._upper_roi1 = val
    @lower_roi2.setter
    def lower_roi2(self,val):
        self._lower_roi2 = val
    @upper_roi2.setter
    def upper_roi2(self,val):
        self._upper_roi2 = val
    @lower_roi3.setter
    def lower_roi3(self,val):
        self._lower_roi3 = val
    @upper_roi3.setter
    def upper_roi3(self,val):
        self._upper_roi3 = val
    @lower_roi4.setter
    def lower_roi4(self,val):
        self._lower_roi4 = val
    @upper_roi4.setter
    def upper_roi4(self,val):
        self._upper_roi4 = val
    @lower_roi5.setter
    def lower_roi5(self,val):
        self._lower_roi5 = val
    @upper_roi5.setter
    def upper_roi5(self,val):
        self._upper_roi5 = val
    @lower_roi6.setter
    def lower_roi6(self,val):
        self._lower_roi6 = val
    @upper_roi6.setter
    def upper_roi6(self,val):
        self._upper_roi6 = val
    @lower_roi7.setter
    def lower_roi7(self,val):
        self._lower_roi7 = val
    @upper_roi7.setter
    def upper_roi7(self,val):
        self._upper_roi7 = val
    @lower_roi8.setter
    def lower_roi8(self,val):
        self._lower_roi8 = val
    @upper_roi8.setter
    def upper_roi8(self,val):
        self._upper_roi8 = val


###########################################
# Define first expiry function
###########################################
def first_expiry(hsv,bound):
    #logging.debug('first expiry starting')
    cv2.imwrite('hsv_capture.png',hsv)
        
    hsv_cap = cv2.imread('hsv_capture.png')
    #cv2.imshow('hsv', hsv_cap)
    bound.lower_roi1,bound.upper_roi1=boundary_routine(hsv_cap[y1:y2, x1:x2])
    bound.lower_roi2,bound.upper_roi2=boundary_routine(hsv_cap[y3:y4, x3:x4])
    bound.lower_roi3,bound.upper_roi3=boundary_routine(hsv_cap[y5:y6, x5:x6])

    
    #logging.debug('1 lower_roi1 %s', bound.lower_roi1)
    #logging.debug('1 upper_roi1 %s', bound.upper_roi1)
    #logging.debug('1 lower_roi2 %s', bound.lower_roi2)
    #logging.debug('1 upper_roi2 %s', bound.upper_roi2)
    #logging.debug('1 lower_roi3 %s', bound.lower_roi3)
    #logging.debug('1 upper_roi3 %s', bound.upper_roi3)
    #logging.debug('first expiry end')

###########################################
# Define common function
###########################################
def boundary_routine (roi):
    
    average_colour_row_roi = np.average(roi, axis=0)
    centre_colour_roi = np.average(average_colour_row_roi, axis=0)

    # declare boundary values for masks
    if all(i > 40 for i in centre_colour_roi):
        # lower hsv boundaries
        l1,l2,l3 = centre_colour_roi - 40
        # upper hsv boundaries                              
        u1,u2,u3 = centre_colour_roi + 40
    else:

        u1,u2,u3 = centre_colour_roi + 40
        centre_colour_roi[centre_colour_roi <= 40] = 0
        l1,l2,l3 = centre_colour_roi
        
        
    #logging.debug('boundary_rountine lower l1=%s, l2=%s, l3=%s', l1,l2,l3)
    #logging.debug('boundary_rountine upper u1=%s, u2=%s, u3=%s', u1,u2,u3)


    return (np.array([l1,l2,l3]),np.array([u1,u2,u3]))

    

###########################################
# Define second expiry function
###########################################
def second_expiry(hsv,bound):
    #logging.debug('second expiry starting')

    cv2.imwrite('hsv_capture_2.png',hsv)
    hsv_cap = cv2.imread('hsv_capture_2.png')

    bound.lower_roi4,bound.upper_roi4=boundary_routine(hsv_cap[y7:y8, x7:x8])
    bound.lower_roi5,bound.upper_roi5=boundary_routine(hsv_cap[y9:y10, x9:x10])
    bound.lower_roi6,bound.upper_roi6=boundary_routine(hsv_cap[y11:y12, x11:x12])
    
    
    #logging.debug('second expiry end')

###########################################
# Define third expiry function
###########################################
def third_expiry(hsv,bound):
    #logging.debug('third expiry starting')

    cv2.imwrite('hsv_capture_3.png',hsv)
    hsv_cap = cv2.imread('hsv_capture_3.png')
    
    bound.lower_roi7,bound.upper_roi7=boundary_routine(hsv_cap[y13:y14, x13:x14])


    #logging.debug('third expiry end')

###########################################
# Define fourth expiry function
###########################################
def fourth_expiry(hsv,bound):
    #logging.debug('fourth expiry starting')
    
    cv2.imwrite('hsv_capture_4.png',hsv)
    hsv_cap = cv2.imread('hsv_capture_4.png')

    bound.lower_roi8,bound.upper_roi8=boundary_routine(hsv_cap[y15:y16, x15:x16])
    

    #logging.debug('fourth expiry end')

# load Haar cascade for face detection
############## xml source: Intel Corporation, written by R. Lienhart available Online. www.github.com   #########################
face_cascade = cv2.CascadeClassifier('/home/pi/Documents/faces.xml')

# coordinates of regions of interest
# right
x1,x2, y1,y2 = 310,330, 170,190
x3,x4, y3,y4 = 250,270, 230,250
x5,x6, y5,y6 = 370,390, 230,250

# left
x7,x8, y7,y8 = 150,170, 170,190
x9,x10, y9,y10 = 210,230, 230,250
x11,x12, y11,y12 = 90,110, 230,250

# far and right
x13,x14, y13,y14 = 350,370, 150,170

#far and left
x15,x16, y15,y16 = 110,130, 150,170

if track == "hand":
    # Start the timer
    seconds = 5

    #logging.debug('starting phase 1')
    phase_num = 1
    timer = timer_var(5,False)                     # timer class
    Timer(seconds, timer_thread, (timer,)).start() # Start the timer for phase 1

#Declare the transmit data
coordinates = transmit_data

#Declare the boundary object
bound = boundary

#Declare Event
evt=threading.Event()

# Declare the thread variable
t1 = threading.Thread(target=transmit_thread,args=(evt,coordinates()))
t1.start()

# screenshot flag
screenshots_complete = False


#Main process loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    

    if track == "hand":

        # convert to HSV colour space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if not timer.expired:
            calibration (image, phase_num)

        else:
            if not screenshots_complete:

                if phase_num == 1:              #Phase 1
                    first_expiry(hsv,bound)
                    #logging.debug('starting phase 2')

                elif phase_num == 2:            #Phase 2
                    second_expiry(hsv,bound)
                    #logging.debug('starting phase 3')
                    seconds = 0                 #Reset the initial timer to 0

                elif phase_num == 3:            #Phase 3
                    third_expiry(hsv,bound)
                    #logging.debug('starting phase 4')            

                elif phase_num == 4:            #Phase 4
                    fourth_expiry(hsv,bound)
                    #logging.debug('Finished Screen Shots')
                    screenshots_complete = True

                if not screenshots_complete:    #still processing 
                    timer.expired = False       #Reset the expiry flag
                    phase_num += 1              #Increment to the next phase
                    Timer(seconds, timer_thread, (timer,)).start()
                
        # once ROIs have got their values:
        if screenshots_complete:
        
            # threshold using ROI values
            mask_roi1 = cv2.inRange(hsv, bound.lower_roi1, bound.upper_roi1)
            mask_roi2 = cv2.inRange(hsv, bound.lower_roi2, bound.upper_roi2)
            mask_roi3 = cv2.inRange(hsv, bound.lower_roi3, bound.upper_roi3)
            mask_roi4 = cv2.inRange(hsv, bound.lower_roi4, bound.upper_roi4)
            
            mask_roi5 = cv2.inRange(hsv, bound.lower_roi5, bound.upper_roi5)
            mask_roi6 = cv2.inRange(hsv, bound.lower_roi6, bound.upper_roi6)
            mask_roi7 = cv2.inRange(hsv, bound.lower_roi7, bound.upper_roi7)
            mask_roi8 = cv2.inRange(hsv, bound.lower_roi8, bound.upper_roi8)

            # sum together binary ROIs
            res_roi1_roi2 = cv2.bitwise_or(mask_roi1, mask_roi2)
            res_roi3_roi4 = cv2.bitwise_or(mask_roi3, mask_roi4)
            res_roi5_roi6 = cv2.bitwise_or(mask_roi5, mask_roi6)
            res_roi7_roi8 = cv2.bitwise_or(mask_roi7, mask_roi8)

            res1_4 = cv2.bitwise_or(res_roi1_roi2, res_roi3_roi4)
            res5_8 = cv2.bitwise_or(res_roi5_roi6, res_roi7_roi8)
            
            res = cv2.bitwise_or(res1_4, res5_8)

            # remove noise and form structures using erosion and dilation
            res = cv2.erode(res, None, iterations=3)
            res = cv2.dilate(res, None, iterations=3)

            # random trees to find contours
            im, contours, heirarchy = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

            max_area = 100
            ci = 0

            # if any contours are found, find the largest
            if len(contours) > 0:
                
                ######### the following contour code is sourced from Sasha Gazman, available online, www.github.com ######### 
                # only slight changes have been made to the source code.
                for i in range(len(contours)):
                        cnt=contours[i]
                        area = cv2.contourArea(cnt)
                        print(area)
                        if(area>max_area):
                            max_area=area
                            ci=i

                cnts = contours[ci]

                #print(cnts)
                cv2.drawContours(im, cnts, -1, (255,255,0), 3)

                #Find convex hull
                hull = cv2.convexHull(cnts)
                
                #Find convex defects
                hull2 = cv2.convexHull(cnts,returnPoints = False)
                defects = cv2.convexityDefects(cnts,hull2)
                
                #Get defect points and draw them in the original image
                FarDefect = []

                if defects != None:
                    for i in range(defects.shape[0]):
                        s,e,f,d = defects[i,0]
                        start = tuple(cnts[s][0])
                        end = tuple(cnts[e][0])
                        far = tuple(cnts[f][0])
                        FarDefect.append(far)
                        cv2.line(image,start,end,[0,255,0],1)
                        cv2.circle(image,far,10,[100,255,255],3)
                
                #Find moments of the largest contour
                moments = cv2.moments(cnts)
                
                #Central mass of first order moments
                if moments['m00']!=0:
                    cx = int(moments['m10']/moments['m00']) # cx = M10/M00
                    cy = int(moments['m01']/moments['m00']) # cy = M01/M00
                centerMass=(cx,cy)    
                
                #Draw center mass
                cv2.circle(image,centerMass,7,[100,0,255],2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image,'Centre',tuple(centerMass),font,2,(255,255,255),2)     
                
                #Distance from each finger defect(finger webbing) to the center mass
                distanceBetweenDefectsToCenter = []
                for i in range(0,len(FarDefect)):
                    x =  np.array(FarDefect[i])
                    centerMass = np.array(centerMass)
                    distance = np.sqrt(np.power(x[0]-centerMass[0],2)+np.power(x[1]-centerMass[1],2))
                    distanceBetweenDefectsToCenter.append(distance)
                
                #Get an average of three shortest distances from finger webbing to center mass
                sortedDefectsDistances = sorted(distanceBetweenDefectsToCenter)
                AverageDefectDistance = np.mean(sortedDefectsDistances[0:2])
             
                #Get fingertip points from contour hull
                #If points are in proximity of 80 pixels, consider as a single point in the group
                finger = []
                for i in range(0,len(hull)-1):
                    if (np.absolute(hull[i][0][0] - hull[i+1][0][0]) > 80) or ( np.absolute(hull[i][0][1] - hull[i+1][0][1]) > 50):
                        if hull[i][0][1] <500 :
                            finger.append(hull[i][0])
                
                #The fingertip points are 5 hull points with largest y coordinates  
                finger =  sorted(finger,key=lambda x: x[1])   
                fingers = finger[0:5]
                
                #Calculate distance of each finger tip to the center mass
                fingerDistance = []
                for i in range(0,len(fingers)):
                    distance = np.sqrt(np.power(fingers[i][0]-centerMass[0],2)+np.power(fingers[i][1]-centerMass[0],2))
                    fingerDistance.append(distance)
                
                #Finger is pointed/raised if the distance of between fingertip to the center mass is larger
                #than the distance of average finger webbing to center mass by 30 pixels
                result = 0
                for i in range(0,len(fingers)):
                    if fingerDistance[i] > AverageDefectDistance+30:
                        result = result +1
                
                #Print number of pointed fingers
                cv2.putText(image,str(result),(100,100),font,2,(255,255,255),2)

                ################# end of contour code from Sasha Gazman #################

                coordinates.corr = centerMass      # populate the class coord array 
                coordinates.finger = result        # populate the finger
                evt.set()                   # Set the Event for transmit_thread
                evt.clear()                 # Clear the event ready for the next
                
            else:
                print("No contours found")

    if track == 'object':
        # convert to grey scale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # capture training image if c is pressed
        if cv2.waitKey(1) & 0xFF == ord("c"):
            capture_flag = True
            
            # loads training image in greyscale
            train_img = cv2.imwrite("train.png",image)
            train_img = cv2.imread("train.png",0)
            # resize to match frames
            train_img = imutils.resize(train_img, 480, 320)
            # finding keypoints using ORB
            kp1, des1 = orb.detectAndCompute(train_img, None)

        # show on screen instructions
        if not capture:
            cv2.putText(image, 'press c to capture train img', (10,25), font, 1, (255,255,0), 2)

        # once training image has been captured
        if capture:

            kp2, des2 = orb.detectAndCompute(image, None)

            # FLANN parameters
            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks=40)   # or pass empty dictionary

            # convert to float
            des1_f = np.float32(des1)
            des2_f = np.float32(des2)

            # flann feature matching
            flann = cv2.FlannBasedMatcher(index_params,search_params)

            # compute matches using FLANN
            matches = flann.knnMatch(des1_f,des2_f,k=2)
            
            good = []
            for m in np.array(matches).flatten().tolist():
                good.append(m)

            # if enough matches are found
            if len(good)>MIN_MATCH_COUNT:
                src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                # use RANSAC to find outliers
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()

                # hight and width
                h,w = train_img.shape
                
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
                
                image = cv2.polylines(image,[np.int32(dst)],True,255,3, cv2.LINE_AA)

                moments = cv2.moments(dst)

                if moments['m00']!=0:
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])

                centerMass=(cx,cy)
                
                # draw on centre of mass of the object
                cv2.circle(image,centerMass,7,[100,0,0],2)
                
            else:
                print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
                matchesMask = None

            draw_params = dict(matchColor = (0,255,0), # draw matches
                           singlePointColor = None,
                           matchesMask = matchesMask, # draw only inliers
                           flags = 2)

            coordinates.corr = centerMass
            
            evt.set()                   # Set the Event for transmit_thread
            evt.clear()                 # Clear the event ready for the next

            # to illustrate feature matching
            img3 = cv2.drawMatches(train_img,kp1,image,kp2,good,None,**draw_params)
          
            cv2.imshow("im3",img3)
            cv2.moveWindow("im3",20,20)

    
    elif track == 'face':
        # convert to greyscale
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        
        faces = face_cascade.detectMultiScale(gray, 1.1, 5)

        # only one track one face at a time
        if len(faces) == 1:  

            for (x,y,w,h) in faces:
                # drawing rectangle around face
                cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                # find centre of face
                centre = int(x+w/2),int(y+h/2)
                # draw centre
                cv2.circle(image,centre,7,[255,0,255],2)
                # write word centre
                cv2.putText(image,'Centre',tuple(centre),font,2,(255,255,0),2)
                
                coordinates.corr = centre   # assign centre coordinates for transmission
                evt.set()                   # Set the Event for transmit_thread
                evt.clear()                 # Clear the event ready for the next

    if track == 'face' or 'hand':
        
        cv2.imshow('image',image)
        cv2.moveWindow('image',20,20)
    
    rawCapture.truncate(0)

    # stop if q key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        time.sleep(0.5)
        coordinates.corr = (0,0)
        coordinates.finger = 99
        evt.set()
        break

cv2.destroyAllWindows()

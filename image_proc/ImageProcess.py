#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy

import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import message_filters

class ImageProcessingClass():
    def __init__(self):
        self.yCloseLeft = self.xCloseLeft = 0
        self.yCloseRight = self.xCloseRight = 0

        self.bridge = CvBridge()

        rospy.Subscriber('/wide_stereo/left/image_rect_color', Image, self.leftCallback)
        rospy.Subscriber('/wide_stereo/right/image_rect_color', Image, self.rightCallback)

        """
        # syncing version is slow and inconsistent
        leftImageSub = message_filters.Subscriber('/wide_stereo/left/image_rect_color', Image)
        rightImageSub = message_filters.Subscriber('/wide_stereo/right/image_rect_color', Image)

        syncImage = message_filters.TimeSynchronizer([leftImageSub, rightImageSub], 10)
        syncImage.registerCallback(self.stereoCallback)
        """

    def leftCallback(self, image):
        colorImg, self.yCloseLeft, self.xCloseLeft = self.process(image)
        cv.ShowImage('Left Viewer', colorImg)
        cv.WaitKey(3)

    def rightCallback(self, image):
        colorImg, self.yCloseRight, self.xCloseRight = self.process(image)
        cv.ShowImage('Right Viewer', colorImg)
        cv.WaitKey(3)

    def process(self, image):
        try:
            colorImg = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError, e:
            return

        numBits = 8
        numChannels = 3
        hsvImg = cv.CreateImage(cv.GetSize(colorImg), numBits, numChannels)

        # smooth out
        cv.Smooth(colorImg, colorImg, cv.CV_GAUSSIAN, 3, 0)

        # convert to hsv
        hsvImg = cv.CreateImage(cv.GetSize(colorImg), numBits, numChannels)
        cv.CvtColor(colorImg, hsvImg, cv.CV_BGR2HSV)
        
        # threshold
        threshImg = cv.CreateImage(cv.GetSize(hsvImg), numBits, 1)
        
        lowerHSV = cv.Scalar(0, 120, 120)
        upperHSV = cv.Scalar(15, 255, 255)

        cv.InRangeS(hsvImg, lowerHSV, upperHSV, threshImg)

        cv.Erode(threshImg, threshImg, None, 1)
        cv.Dilate(threshImg, threshImg, None, 1)

        # find unfiltered pixels
        mat = cv.GetMat(threshImg)

        yxCoords = []
        for x in range(mat.width):
            for y in range(mat.height):
                if mat[y,x] > 0.0:
                    yxCoords.append((y,x))
                    
        yCentroid = sum([y for y,x in yxCoords])/len(yxCoords)
        xCentroid = sum([x for y,x in yxCoords])/len(yxCoords)

        # print and show centroid
        #print('('+str(xCentroid)+','+str(yCentroid)+')')
        cv.Set2D(colorImg, yCentroid, xCentroid, (255,0,0))
        
        # find nearest white pixel
        distFromCentroid = [((y-yCentroid)**2 + (x-xCentroid)**2)**.5 for y,x in yxCoords]
        yClose, xClose = yxCoords[distFromCentroid.index(min(distFromCentroid))]
        
        # print and show closest unfiltered pixel to centroid
        #print('('+str(xClose)+','+str(yClose)+')')
        cv.Set2D(colorImg, yClose, xClose, (255,0,0))

        return (colorImg, yClose, xClose)
        """
        rospy.loginfo('Showing in viewer')
        cv.ShowImage(cam + ' Viewer', colorImg)

        cv.ShowImage(cam + 'Threshold', threshImg)
        cv.WaitKey(3)
        """
    
        

def test():
    rospy.init_node('image_processing')
    ip = ImageProcessingClass()
    rospy.spin()


def webcam():
    numBits = 8
    numChannels = 3

    cv.NamedWindow("Original", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("Threshold", cv.CV_WINDOW_AUTOSIZE)
    capture = cv.CaptureFromCAM(0)

    while True:
        frame = cv.QueryFrame(capture)
        
        #cv.Flip(frame,frame,1)
        cv.Smooth(frame, frame, cv.CV_GAUSSIAN, 3, 0)

        hsvImg = cv.CreateImage(cv.GetSize(frame), numBits, numChannels)
        cv.CvtColor(frame, hsvImg, cv.CV_BGR2HSV)
        threshImg = cv.CreateImage(cv.GetSize(hsvImg), numBits, 1)
        threshImg1 = cv.CreateImage(cv.GetSize(hsvImg), numBits, 1)
        
        # works okay
        #lowerHSV = cv.Scalar(h/2.0, s/2.0, v/2.0)
        #upperHSV = cv.Scalar(h*2.0, s*2.0, s*2.0)

        # works well except for brightness
        #lowerHSV = cv.Scalar(159, 135, 135)
        #upperHSV = cv.Scalar(179, 255, 255)
        
        lowerHSV = cv.Scalar(159, 110, 110)
        upperHSV = cv.Scalar(179, 255, 255)

        
        cv.InRangeS(hsvImg, lowerHSV, upperHSV, threshImg)
        

        #cv.Add(cv.GetMat(threshImg),cv.GetMat(threshImg1),threshImg1)

        # yellow
        #cv.InRangeS(hsvImg, cv.Scalar(20,100,100), cv.Scalar(30,255,255), threshImg)

        cv.Erode(threshImg, threshImg, None, 3)
        cv.Dilate(threshImg, threshImg, None, 10)

        cv.ShowImage("Original", frame)
        cv.ShowImage("Threshold", threshImg)
        cv.WaitKey(3)


def online():
    posx=0
    posy=0
    def getthresholdedimg(im):
	'''this function take RGB image.Then convert it into HSV for easy colour detection and threshold it with yellow part as white and all other regions as black.Then return that image'''
	imghsv=cv.CreateImage(cv.GetSize(im),8,3)
	cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)				# Convert image from RGB to HSV
	imgthreshold=cv.CreateImage(cv.GetSize(im),8,1)
        lowerHSV = cv.Scalar(159, 135, 135)
        upperHSV = cv.Scalar(179, 255, 255)
        #lowerHSV = cv.Scalar(45, 75, 75)
        #upperHSV = cv.Scalar(75, 255, 255)
	cv.InRangeS(imghsv,lowerHSV,upperHSV,imgthreshold)	# Select a range of yellow color
	return imgthreshold


    capture=cv.CaptureFromCAM(0)
    frame = cv.QueryFrame(capture)
    frame_size = cv.GetSize(frame)
    grey_image = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
    test=cv.CreateImage(cv.GetSize(frame),8,3)
    cv.NamedWindow("Real")
    cv.NamedWindow("Threshold")
    while True:
	color_image = cv.QueryFrame(capture)
	imdraw=cv.CreateImage(cv.GetSize(frame),8,3)
	cv.Flip(color_image,color_image,1)
	cv.Smooth(color_image, color_image, cv.CV_GAUSSIAN, 3, 0)
	imgyellowthresh=getthresholdedimg(color_image)
	cv.ShowImage("Threshold",imgyellowthresh)
        cv.Erode(imgyellowthresh,imgyellowthresh,None,3)
	cv.Dilate(imgyellowthresh,imgyellowthresh,None,10)

	storage = cv.CreateMemStorage(0)
	contour = cv.FindContours(imgyellowthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
	points = []	

#	This is the new part here. ie Use of cv.BoundingRect()
	while contour:
		# Draw bounding rectangles
		bound_rect = cv.BoundingRect(list(contour))
		contour = contour.h_next()
		
		# for more details about cv.BoundingRect,see documentation
		pt1 = (bound_rect[0], bound_rect[1])
		pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])
		points.append(pt1)
		points.append(pt2)
		cv.Rectangle(color_image, pt1, pt2, cv.CV_RGB(255,0,0), 1)
		lastx=posx
		lasty=posy
		posx=cv.Round((pt1[0]+pt2[0])/2)
		posy=cv.Round((pt1[1]+pt2[1])/2)
		if lastx!=0 and lasty!=0:
			cv.Line(imdraw,(posx,posy),(lastx,lasty),(0,255,255))
			cv.Circle(imdraw,(posx,posy),5,(0,255,255),-1)
	cv.Add(test,imdraw,test)
        
        cv.ShowImage("Real",color_image)
	#cv.ShowImage("Threshold",imgyellowthresh)

        cv.WaitKey(3)




if __name__ == '__main__':
    test()
    #webcam()
    #online()

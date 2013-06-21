#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy

import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
import tf
import image_geometry

import message_filters
from threading import Lock

class ImageProcessingClass():
    def __init__(self):
        self.yCloseLeft = self.xCloseLeft = 0
        self.yCloseRight = self.xCloseRight = 0
        
        self.xyCloseLeft = (0,0)
        self.xyCloseRight = (0,0)
        
        self.xyCentroidLeft = (0,0)
        self.xyCentroidRight = (0,0)
        
        self.leftInfo = self.rightInfo = None

        self.locks = dict()
        self.locks['leftImage'] = Lock()
        self.locks['rightImage'] = Lock()
        #self.locks['stereo'] = Lock()
        self.locks['leftInfo'] = Lock()
        self.locks['rightInfo'] = Lock()

        self.foundColorLeft = False
        self.foundColorRight = False

        self.listener = tf.TransformListener()
        self.outputFrame = 'base_link'
        
        self.bridge = CvBridge()

        rospy.Subscriber('/wide_stereo/left/image_rect_color', Image, self.leftImageCallback)
        rospy.Subscriber('/wide_stereo/right/image_rect_color', Image, self.rightImageCallback)

        rospy.Subscriber('/wide_stereo/left/camera_info', CameraInfo, self.leftInfoCallback)
        rospy.Subscriber('/wide_stereo/right/camera_info', CameraInfo, self.rightInfoCallback)
        


        # syncing version
        #leftImageSub = message_filters.Subscriber('/wide_stereo/left/image_rect_color', Image)
        #rightImageSub = message_filters.Subscriber('/wide_stereo/right/image_rect_color', Image)
        #leftInfoSub = message_filters.Subscriber('/wide_stereo/left/camera_info', CameraInfo)
        #rightInfoSub = message_filters.Subscriber('/wide_stereo/right/camera_info', CameraInfo)

        #syncImage = message_filters.TimeSynchronizer([leftImageSub, rightImageSub], 10)
        #syncImage.registerCallback(self.stereoCallback)
        
        # let some camera feeds in
        rospy.sleep(1)

    def getCentroid(self):
        """
        Returns PointStamped of the centroid of the color being processed
        
        Returns None if lacks camera info or camera images
        """
        u, v = self.xyCentroidLeft
        disparity = self.xyCentroidLeft[0] - self.xyCentroidRight[0]

        return self.convertStereo(u, v, disparity)

    def getClosestToCentroid(self):
        """
        Returns PointStamped of the nearest instance of the color to the centroid

        Returns None if lacks camera info or camera images

        If a solid object, then getCentroid and getClosestToCentroid are the same
        """
        u, v = self.xyCloseLeft
        disparity = self.xyCloseLeft[0] - self.xyCloseRight[0]

        return self.convertStereo(u, v, disparity)

    def canProcess(self):
        return (self.leftInfo != None and self.rightInfo != None and self.foundColorLeft and self.foundColorRight)

    def convertStereo(self, u, v, disparity):
        """
        Converts two pixel coordinates u and v along with the disparity to give PointStamped       
        """

        if not self.canProcess():
            return None
        
        for key in self.locks.keys():
            self.locks[key].acquire()

        stereoModel = image_geometry.StereoCameraModel()
        stereoModel.fromCameraInfo(self.leftInfo, self.rightInfo)
        (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)
        
        cameraPoint = PointStamped()
        cameraPoint.header.frame_id = self.leftInfo.header.frame_id
        cameraPoint.header.stamp = rospy.Time.now()
        cameraPoint.point = Point(x,y,z)

        self.listener.waitForTransform(self.outputFrame, cameraPoint.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        outputPoint = self.listener.transformPoint(self.outputFrame, cameraPoint)

        for key in self.locks.keys():
            self.locks[key].release()

        return outputPoint
    
    def leftInfoCallback(self, info):
        self.locks['leftInfo'].acquire()
        self.leftInfo = info
        self.locks['leftInfo'].release()

    def rightInfoCallback(self, info):
        self.locks['rightInfo'].acquire()
        self.rightInfo = info
        self.locks['rightInfo'].release()
        

    """
    def stereoCallback(self, leftImage, rightImage):
        self.locks['stereo'].acquire()
        
        colorImgLeft, foundLeft, self.xyCloseLeft, self.xyCentroidLeft = self.process(leftImage)
        colorImgRight, foundRight, self.xyCloseRight, self.xyCentroidRight = self.process(rightImage)

        self.hasFoundColor = foundLeft and foundRight
        
        #cv.ShowImage('Right Viewer', colorImgRight)
        #cv.ShowImage('Left Viewer', colorImgLeft)
        #cv.WaitKey(3)

        self.locks['stereo'].release()
    """

    def leftImageCallback(self, image):
        self.locks['leftImage'].acquire()
        colorImg, self.foundColorLeft, self.xyCloseLeft, self.xyCentroidLeft = self.process(image)
        #cv.ShowImage('Left Viewer', colorImg)
        #cv.WaitKey(3)
        self.locks['leftImage'].release()

    def rightImageCallback(self, image):
        self.locks['rightImage'].acquire()
        colorImg, self.foundColorRight, self.xyCloseRight, self.xyCentroidRight = self.process(image)
        #cv.ShowImage('Right Viewer', colorImg)
        #cv.WaitKey(3)
        self.locks['rightImage'].release()
    

    def process(self, image):
        """
        Takes in a sensor_msgs/Image and outputs a tuple for the closest pixel coordinates to the centroid that are the correct color and outputs the centroid of the correct color

        Temporarily, first argument is a modified opencv image, for debugging
        """
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

        if len(yxCoords) == 0:
            return colorImg, False, (0,0), (0,0)
                    
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

        return (threshImg, True, (xClose, yClose), (xCentroid, yCentroid))

        

def test():
    rospy.init_node('image_processing')
    ip = ImageProcessingClass()
    
    while not rospy.is_shutdown():
        centroid = ip.getCentroid()
        closest = ip.getClosestToCentroid()
        if centroid == None or closest == None:
            print('Not found')
        else:
            print('Centroid')
            print(centroid.point)
            print('Closest')
            print(closest.point)
        rospy.sleep(.5)







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


if __name__ == '__main__':
    test()
    #webcam()

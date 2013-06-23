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
    """
    This class receives the left and right color rectified frames from the pr2 and processes the images.

    The processing done is filtering of a specific color (in this case red,
    but there may be future modifications to generalize the color that is segmented).

    The methods that should be called by the user are getCentroid and getClosestToCentroid
    """
    def __init__(self):
        # pixel coordinates for color closest to centroid in left and right image respectively
        self.xyCloseLeft = (0,0)
        self.xyCloseRight = (0,0)

        # pixel coordinates for centroid of the specific color in left and right image respectively
        self.xyCentroidLeft = (0,0)
        self.xyCentroidRight = (0,0)
        
        # camera_info messages for left and right images
        self.leftInfo = self.rightInfo = None

        # locks for message retrieval
        # these are necessary since when processing,
        # we don't want some of the info to change mid execution
        self.locks = dict()
        self.locks['leftImage'] = Lock()
        self.locks['rightImage'] = Lock()
        self.locks['leftInfo'] = Lock()
        self.locks['rightInfo'] = Lock()

        # booleans for if any color has been found in left and right image respectively
        self.foundColorLeft = False
        self.foundColorRight = False

        self.listener = tf.TransformListener()
        self.outputFrame = 'base_link'
        
        # converts ros images to opencv images
        self.bridge = CvBridge()

        # subscribers to camera images
        rospy.Subscriber('/wide_stereo/left/image_rect_color', Image, self.leftImageCallback)
        rospy.Subscriber('/wide_stereo/right/image_rect_color', Image, self.rightImageCallback)

        # subscribers to camera info
        rospy.Subscriber('/wide_stereo/left/camera_info', CameraInfo, self.leftInfoCallback)
        rospy.Subscriber('/wide_stereo/right/camera_info', CameraInfo, self.rightInfoCallback)
        
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
        """
        Returns true if camera info has been received and
        color has been found in both images
        """
        return (self.leftInfo != None and self.rightInfo != None and self.foundColorLeft and self.foundColorRight)

    def convertStereo(self, u, v, disparity):
        """
        Converts two pixel coordinates u and v along with the disparity to give PointStamped
        
        This code was taken from stereo_click package
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
        

    def leftImageCallback(self, image):
        """
        Left image callback method that also processes
        the image immediately. This is not a problem
        since the processing is faster than the frame rate
        """
        self.locks['leftImage'].acquire()
        img, self.foundColorLeft, self.xyCloseLeft, self.xyCentroidLeft = self.process(image)
        #cv.ShowImage('Left Viewer', img)
        #cv.WaitKey(3)
        self.locks['leftImage'].release()

    def rightImageCallback(self, image):
        """
        Right image callback method that also processes
        the image immediately. This is not a problem
        since the processing is faster than the frame rate
        """
        self.locks['rightImage'].acquire()
        img, self.foundColorRight, self.xyCloseRight, self.xyCentroidRight = self.process(image)
        #cv.ShowImage('Right Viewer', img)
        #cv.WaitKey(3)
        self.locks['rightImage'].release()
    

    def process(self, image):
        """
        Processes a sensor_msgs/Image

        The image is converted to HSV, thresholded, and then
        processed for the centroid and closest to centroid

        Outputs four values:
        - a processed image (for debugging purposes)
        - boolean if any color has been found in the image
        - tuple of pixel coordinates closest to color centroid
        - tuple of pixel coordinates for centroid of color
        """
        # convert to opencv format
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
        
        # these are the bounds for the color being filtered
        # currently, these work well for the color red
        # can be changed depending on the application
        lowerHSV = cv.Scalar(0, 120, 120)
        upperHSV = cv.Scalar(15, 255, 255)

        cv.InRangeS(hsvImg, lowerHSV, upperHSV, threshImg)

        # filter out noise, but also decreases granularity
        cv.Erode(threshImg, threshImg, None, 1)
        cv.Dilate(threshImg, threshImg, None, 1)

        # find unfiltered pixels
        mat = cv.GetMat(threshImg)
        yxCoords = []
        for x in range(mat.width):
            for y in range(mat.height):
                if mat[y,x] > 0.0:
                    yxCoords.append((y,x))

        # check if any color is present
        if len(yxCoords) == 0:
            return colorImg, False, (0,0), (0,0)
                    
        yCentroid = sum([y for y,x in yxCoords])/len(yxCoords)
        xCentroid = sum([x for y,x in yxCoords])/len(yxCoords)

        # debugging: print and show centroid
        #print('('+str(xCentroid)+','+str(yCentroid)+')')
        #cv.Set2D(colorImg, yCentroid, xCentroid, (255,0,0))
        
        # find nearest color pixel to centroid (based on euclidean distance)
        distFromCentroid = [((y-yCentroid)**2 + (x-xCentroid)**2)**.5 for y,x in yxCoords]
        yClose, xClose = yxCoords[distFromCentroid.index(min(distFromCentroid))]
        
        # debugging: print and show closest unfiltered pixel to centroid
        #print('('+str(xClose)+','+str(yClose)+')')
        #cv.Set2D(colorImg, yClose, xClose, (255,0,0))

        return (threshImg, True, (xClose, yClose), (xCentroid, yCentroid))

        

def test():
    """
    Runs the image processing class,
    prints out the centroid and
    closest to centroid
    """
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



if __name__ == '__main__':
    test()

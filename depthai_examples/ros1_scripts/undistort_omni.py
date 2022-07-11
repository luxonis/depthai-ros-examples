from cv2 import undistort
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
# from depthai_ros_msgs.msg import SpatialDetectionArray
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import Image


class OmniUndistort:
    def __init__(self):
       rospy.init_node("my_node")
       rospy.Subscriber("/stereo_inertial_publisher/left/image_raw", Image,  self.undistortCallbackLeft, queue_size=1)
    #    rospy.Subscriber("/stereo_inertial_publisher/right/image_raw", Image, self.undistortCallbackRight, queue_size=1)
 
       # On initialization, set up a Publisher for ImageMarkerArrays
       self.pubLeft = rospy.Publisher("left/original", Image, queue_size=1)
       self.pubLeftPerspective = rospy.Publisher("left/undistortPerspective", Image, queue_size=1)
       self.pubLeftCylindrical = rospy.Publisher("left/undistortCylindrical", Image, queue_size=1)
       self.pubLeftLongLati = rospy.Publisher("left/undistortLongLati", Image, queue_size=1)
       self.pubLeftStereographic = rospy.Publisher("left/undistortStereographic", Image, queue_size=1)

       self.pubRightPerspective = rospy.Publisher("right/undistortPerspective", Image, queue_size=1)
       self.pubRightCylindrical = rospy.Publisher("right/undistortCylindrical", Image, queue_size=1)
       self.pubRightLongLati = rospy.Publisher("right/undistortLongLati", Image, queue_size=1)
       self.pubRightStereographic = rospy.Publisher("right/undistortStereographic", Image, queue_size=1)

       rospy.spin()

    def undistort(self, image, k, d, xi):
        new_size = image.shape
        width =  new_size[1]
        height = new_size[0]
        copy_image = image.copy()
        # cv2.imshow("original", copy_image)
        # cv2.waitKey(1)

        width  = 4000
        height = 2000
        new_size = (2000, 4000)
        
        Knew = np.array([[width/4, 0,       width/2],
                         [0,    height/4,   height/2],
                         [0,        0,      1]])
        undistortedPerspective   = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_PERSPECTIVE, Knew, new_size = new_size)
        
        Knew = np.array([[width/3.1415, 0,         0],
                         [0,        height/3.1415, 0],
                         [0,            0,         1]])
        undistortedCylindrical   = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_CYLINDRICAL, new_size = new_size)
        undistortedLongLati      = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_LONGLATI, Knew, new_size = new_size)
        undistortedStereographic = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_STEREOGRAPHIC, Knew, new_size = new_size)

        bridge = CvBridge()
        undistortedPerspectiveMsg   = bridge.cv2_to_imgmsg(undistortedPerspective, encoding="bgr8")
        undistortedCylindricalMsg   = bridge.cv2_to_imgmsg(undistortedCylindrical, encoding="bgr8")
        undistortedLongLatiMsg      = bridge.cv2_to_imgmsg(undistortedLongLati, encoding="bgr8")
        undistortedStereographicMsg = bridge.cv2_to_imgmsg(undistortedStereographic, encoding="bgr8")
        imageMsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        # self.pubLeft.publish(imageMsg)
        cv2.imshow("distort", image)
        cv2.imshow("stereographic", undistortedStereographic)
        cv2.waitKey(1)
        return undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg

    def undistortCallbackLeft(self, imageMsg):
        k  = np.array([[1028.9773604091108, 0, 637.0711926542228], 
                       [0, 1024.915013009914, 359.1709279099879], 
                       [0, 0, 1.0]], float)
        d  = np.array([[-0.07990387890772704, 0.4140699440729293, -0.00033578870234897423, 8.044571858836192e-05]], float)
        xi =  np.array([[2.1004668620326186]], float)
        
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(imageMsg, desired_encoding='passthrough')
        undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg = self.undistort(image, k, d, xi)

        self.pubLeftPerspective.publish(undistortedPerspectiveMsg)
        self.pubLeftCylindrical.publish(undistortedCylindricalMsg)
        self.pubLeftLongLati.publish(undistortedLongLatiMsg)
        self.pubLeftStereographic.publish(undistortedStereographicMsg)


    def undistortCallbackRight(self, imageMsg):
        k  = np.array([[1016.881259866076, 0, 633.1888651587963],
                       [0, 1012.9617023214325, 360.2427396970111],
                       [0, 0, 1]], float)

        d  = np.array([[-0.09002922696408215, 0.46893422416016406, -0.0009798368219106355, 0.0005800480852674678]], float)
        xi = np.array([[2.0892921078695146]], float) 
        
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(imageMsg, desired_encoding='passthrough')
        undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg = self.undistort(image, k, d, xi)
        cv2.imshow("distort", image)
        cv2.waitKey(1)
        
        self.pubRightPerspective.publish(undistortedPerspectiveMsg)
        self.pubRightCylindrical.publish(undistortedCylindricalMsg)
        self.pubRightLongLati.publish(undistortedLongLatiMsg)
        self.pubRightStereographic.publish(undistortedStereographicMsg)

        

def main():
    OmniUndistort()


if __name__ == "__main__":
    main()
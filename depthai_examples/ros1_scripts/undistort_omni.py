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
       rospy.Subscriber("/stereo_inertial_publisher/right/image_raw", Image, self.undistortCallbackRight, queue_size=1)
 
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
        
        Knew = np.array([[width/4,  0,          width/2],
                         [0,        height/4,   height/2],
                         [0,        0,          1]])

       # 20 deg Y axis rotation

        rot = np.array([[0.9396926,   0.0000000,  0.3420202],
                        [0.0000000,   1.0000000,  0.0000000],
                        [-0.3420202,  0.0000000,  0.9396926]])
        
       # 40 deg Y axis rotation

        rot = np.array([[ 0.7660444,  0.0000000,  0.6427876],
                        [ 0.0000000,  1.0000000,  0.0000000],
                        [-0.6427876,  0.0000000,  0.7660444]])
        
       # 60 deg Y axis rotation

        rot = np.array([[ 0.5000000,  0.0000000,  0.8660254],
                        [ 0.0000000,  1.0000000,  0.0000000],
                        [-0.8660254,  0.0000000,  0.5000000]])

       # -80 deg Y axis rotation
        rot = np.array([[0.1736482,  0.0000000, -0.9848077],
                        [0.0000000,  1.0000000,  0.0000000],
                        [0.9848077,  0.0000000,  0.1736482]])

       # -60 deg Y axis rotation
        rot = np.array([[0.5000000,  0.0000000, -0.8660254],
                        [0.0000000,  1.0000000,  0.0000000],
                        [0.8660254,  0.0000000,  0.5000000]])

       # -60 deg Y axis and -20 on X axis rotation
        rot = np.array([[0.5000000,  0.0000000, -0.8660254],
                        [0.2961981,  0.9396926,  0.1710101],
                        [0.8137977, -0.3420202,  0.4698463]])

        undistortedPerspective = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_PERSPECTIVE, R = rot)

        # undistortedPerspective   = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_PERSPECTIVE, Knew, new_size = new_size)
        
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
        # imageMsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        # self.pubLeft.publish(imageMsg)
        # cv2.imshow("distort", image)
        # cv2.imshow("stereographic", undistortedStereographic)
        # cv2.waitKey(1)
        return undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg

    # 2.026116912280306, 747.5605341318326, 748.8425464125638, 503.6087904711565, 379.31152707350475
    def undistortCallbackLeft(self, imageMsg):
        k  = np.array([[747.5605341318326, 0, 503.6087904711565], 
                       [0, 748.8425464125638, 379.31152707350475], 
                       [0, 0, 1.0]], float)
        d  = np.array([[-0.10992433544726093, 0.4083965617819916, -0.0007173534337265417, -0.0006166026810620713]], float)
        xi =  np.array([[2.026116912280306]], float)
        
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(imageMsg, desired_encoding='passthrough')
        undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg = self.undistort(image, k, d, xi)

        self.pubLeftPerspective.publish(undistortedPerspectiveMsg)
        self.pubLeftCylindrical.publish(undistortedCylindricalMsg)
        self.pubLeftLongLati.publish(undistortedLongLatiMsg)
        self.pubLeftStereographic.publish(undistortedStereographicMsg)

    # 2.0119458339348713, 737.5308301015713, 738.6893351565132, 502.38453703944117, 379.0327864501004
    def undistortCallbackRight(self, imageMsg):
        k  = np.array([[737.5308301015713, 0, 502.38453703944117],
                       [0, 738.6893351565132, 379.0327864501004],
                       [0, 0, 1]], float)

        d  = np.array([[-0.12706231175543312, 0.4767368997245978, -0.0015363469862339065, 0.0005152349386779312]], float)
        xi = np.array([[2.0119458339348713]], float)
        
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(imageMsg, desired_encoding='passthrough')
        undistortedPerspectiveMsg, undistortedCylindricalMsg, undistortedLongLatiMsg, undistortedStereographicMsg = self.undistort(image, k, d, xi)
        
        self.pubRightPerspective.publish(undistortedPerspectiveMsg)
        self.pubRightCylindrical.publish(undistortedCylindricalMsg)
        self.pubRightLongLati.publish(undistortedLongLatiMsg)
        self.pubRightStereographic.publish(undistortedStereographicMsg)

        

def main():
    OmniUndistort()


if __name__ == "__main__":
    main()
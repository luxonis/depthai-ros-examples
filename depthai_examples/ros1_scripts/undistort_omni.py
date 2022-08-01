import message_filters
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

        maxDisparity = 190
        blockSize = 5
        K = 32
        LRthreshold = 2
        self.stereoProcessor = cv2.StereoSGBM_create(
            minDisparity=1,
            numDisparities=maxDisparity,
            blockSize=blockSize,
            P1=2 * (blockSize ** 2),
            P2=K * (blockSize ** 2),
            disp12MaxDiff=LRthreshold,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
 

        left_sub  = message_filters.Subscriber("/stereo_inertial_publisher/left/image_raw",  Image)
        right_sub = message_filters.Subscriber("/stereo_inertial_publisher/right/image_raw", Image)
    
        ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.9)        
        ts.registerCallback(self.stereoReconstructCallback)

        #    rospy.Subscriber("/stereo_inertial_publisher/left/image_raw", Image,  self.undistortCallbackLeft, queue_size=1)
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
        # copy_image = image.copy()
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

    def stereoReconstructCallback(self, leftImgMsg, rightImageMsg):

        kLeft  = np.array([[755.220194243529, 0, 503.7627638065912], 
                           [0, 752.5595083167417, 379.12063549278554], 
                           [0, 0, 1.0]], float)
        dLeft  = np.array([[-0.12250034916583766, 0.5409617731001941, -0.0027348679896492937, -0.0033019856319997736]], float)
        xiLeft = np.array([[2.0554874923292057]], float)

        kRight  = np.array([[757.1337928004853, 0, 508.14972031711176],
                            [0, 754.3808116028206, 377.7936644736236],
                            [0, 0, 1]], float)

        dRight  = np.array([[-0.12466843249136443, 0.4539994383192077, -0.0020298550160945754, -0.002938034843103457]], float)
        xiRight = np.array([[2.041547658038243]], float)

        # transformation = np.array([[0.9999378975984609, -0.004352271870197581, -0.010259565094392187, 0.04263875562328708],
        #                            [0.004565082250721026, 0.9997729992246492, 0.020811296100477848, 0.000654833800297008],
        #                            [0.010166659746560596, -0.02085683942752352, 0.9997307793994743, 0.0003253521964047307]], float)


        transformation = np.array([[0.999986523390199, 0.004976222165131139, 0.0014799496434515056, -0.04260840945432192],
                                   [-0.0049631869672225526, 0.9999499689519986, -0.008684835520975933, -0.00013985148542148193],
                                   [-0.0015230932710398521, 0.008677373212053903, 0.9999611909374421, -0.00016307543333419315],
                                   [0.0, 0.0, 0.0, 1.0]], float)

        R = transformation[:3, :3]

        from scipy.spatial.transform import Rotation
        print('R----->')
        print(R)
        rot = Rotation.from_matrix(R)
        print(rot.as_euler('xyz', degrees=True))


        # R = np.array([[-0.999853,    0.0153559,  -0.00762841],
        #               [-0.0153555,    -0.999882, -0.000117155],
        #               [-0.00762931,   0.0,     0.999971]], float)
        """ 
        R = R.transpose()

        print('R New---- ->')
        print(R)
        rot = Rotation.from_matrix(R)
        print(rot.as_euler('xyz', degrees=True))
        """
        
        # t = - np.dot(R, transformation[:, 3])
        t = transformation[:3, 3]
        # print("Prinyiny Org T")
        # print(transformation[:, 3])
        print("Printing T")
        print(t)

        flag = cv2.omnidir.RECTIFY_PERSPECTIVE
        bridge = CvBridge()
        imageLeft  = bridge.imgmsg_to_cv2(leftImgMsg, desired_encoding='passthrough')
        bridge = CvBridge()
        imageRight = bridge.imgmsg_to_cv2(rightImageMsg, desired_encoding='passthrough')
        numDisparities = 16
        SADWindowSize = 3

        if 1: # stereo mode with direct reconstruction. Not working. there seems to be offset. 
            disp, imgLeftRect, imgRightRect, pointCloud = cv2.omnidir.stereoReconstruct(imageLeft, imageRight, kLeft, dLeft, xiLeft, kRight, dRight, xiRight, R, t, flag, numDisparities, SADWindowSize)

            cv2.imshow("disp", disp)
            cv2.imshow("imgLeftRect", imgLeftRect)
            cv2.imshow("imgRightRect", imgRightRect)
            cv2.waitKey(1)
        # undistortedPerspective = cv2.omnidir.undistortImage(image, k, d, xi, cv2.omnidir.RECTIFY_PERSPECTIVE, R = rot)
        if 0:
            R1, R2 = cv2.omnidir.stereoRectify(R, t)
            
            print('R1----->')
            print(R1)
            rot = Rotation.from_matrix(R1)
            print(rot.as_euler('xyz', degrees=True))
            print('R2----->')
            print(R2)
            rot = Rotation.from_matrix(R2)
            print(rot.as_euler('xyz', degrees=True))

            undistortedPerspectiveLeft = cv2.omnidir.undistortImage(imageLeft, kLeft, dLeft, xiLeft, cv2.omnidir.RECTIFY_PERSPECTIVE     , R = R1)
            undistortedPerspectiveRight = cv2.omnidir.undistortImage(imageRight, kRight, dRight, xiRight, cv2.omnidir.RECTIFY_PERSPECTIVE, R = R2)

            cv2.imshow("imageRight", imageRight)
            cv2.imshow("undistortedPerspectiveLeft", undistortedPerspectiveLeft)
            cv2.imshow("undistortedPerspectiveRight", undistortedPerspectiveRight)
            cv2.waitKey(1)
        if 0:
            R1, R2 = cv2.omnidir.stereoRectify(R, t)
            print('R1----->')
            print(R1)
            rot = Rotation.from_matrix(R1)
            print(rot.as_euler('xyz', degrees=True))
            print('R2----->')
            print(R2)
            rot = Rotation.from_matrix(R2)
            print(rot.as_euler('xyz', degrees=True))

            undistortedPerspectiveLeft = cv2.omnidir.undistortImage(imageLeft, kLeft, dLeft, xiLeft, cv2.omnidir.RECTIFY_PERSPECTIVE     , R = R1)
            undistortedPerspectiveRight = cv2.omnidir.undistortImage(imageRight, kRight, dRight, xiRight, cv2.omnidir.RECTIFY_PERSPECTIVE, R = R2)


            subpixelBits = 16.
            disparity = self.stereoProcessor.compute(left, right)
            disparity = (disparity / subpixelBits).astype(np.uint8)

            cv2.imshow("undistortedPerspectiveLeft", undistortedPerspectiveLeft)
            cv2.imshow("undistortedPerspectiveRight", undistortedPerspectiveRight)
            cv2.imshow("disparity", disparity)
            key = cv2.waitKey(1)



  

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
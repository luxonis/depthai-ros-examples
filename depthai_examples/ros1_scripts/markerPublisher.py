import rospy
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker
from depthai_ros_msgs.msg import SpatialImgDetections
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA



class SpatialDetectionsVisualizationNode:
    def __init__(self):
       rospy.init_node("my_node")
       rospy.Subscriber("/stereo_inertial_publisher/color/yolov4_Spatial_detections", SpatialImgDetections, self.detectionCallback, queue_size=1)

       # On initialization, set up a Publisher for ImageMarkerArrays
       self.pub_markers = rospy.Publisher("/spatialDetectionMarkers", ImageMarkerArray, queue_size=1)
       
       rospy.spin()

    def detectionCallback(self, spatialMsgArray):
        markers = ImageMarkerArray()

        for spatialMsg in spatialMsgArray.detections:
            bbox = spatialMsg.bbox
            markers.markers.append(ImageMarker(
                    header=spatialMsgArray.header,
                    scale=1,
                    type=ImageMarker.LINE_STRIP,
                    outline_color=ColorRGBA(0, 1, 1, 1),
                    points=[
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                        Point(bbox.center.x + bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                        Point(bbox.center.x + bbox.size_x/2, bbox.center.y - bbox.size_y/2, 0),
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y - bbox.size_y/2, 0),
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                    ],
                )
                )
        self.pub_markers.publish(markers)

        
def main():
    SpatialDetectionsVisualizationNode()


if __name__ == "__main__":
    main()
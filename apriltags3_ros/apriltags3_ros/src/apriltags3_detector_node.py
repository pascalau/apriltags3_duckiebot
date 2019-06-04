#!/usr/bin/env python
# license removed for brevity

import rospy
import apriltags3_ros as ap3
from sensor_msgs.msg import Image, CameraInfo
from duckietown_msgs.msg import BoolStamped
from apriltags3_ros.msg import AprilTagDetection, AprilTagDetectionArray
import cv2
from cv_bridge import CvBridge

class detector_node():

    def __init__(self):

        #Constructing Detector for Apriltags detection
        #TODO Maybe add a config file for detector-object aswell

        self.at_detector = ap3.Detector(searchpath=['/home/software/catkin_ws/devel'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)    

        # CV Bridge

        self.bridge = CvBridge()

        #   Topics

        self.image_rect = rospy.Subscriber('camera_node/image/rect', Image, self.detect_ap)
        self.camera_info = rospy.Subscriber('camera_node/raw_camera_info', CameraInfo, self.Camera_params)
        self.switch_sub = rospy.Subscriber('apriltag_detector_node/switch', BoolStamped, self.trigger)
        self.detections = rospy.Publisher('tag_detections', AprilTagDetectionArray)

        # Variables & Constants
        #TODO maybe add a configfile if necessary

        self.switch = True
        self.camera_intrinsics = [337.3092553690036,336.85901913795016,314.1137093251905,217.29382862441514]    #Initialisation of camera parameters with defaults
        self.tag_size = 0.065   #default tagsize
        self.sequence_id = 0
        self.rotation_quaternion = [1,0,0,0]

    #Detection and publication of detections
    def detect_ap(self,msg):

        #Trigger of FSM node
        if not self.switch:
            return

        #Import of image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        #Tagdetection
        tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=self.camera_intrinsics, tag_size=self.tag_size)

        #Processing of detected tags
        detected_list = []
        time = rospy.Time.now()

        for tag in tags:
            detectedtag = AprilTagDetection()

            detectedtag.id = [tag.tag_id]
            detectedtag.size = [self.tag_size]
            
            #Header
            detectedtag.pose.header.stamp = time
            detectedtag.pose.header.frame_id = rospy.get_namespace()[1:-1]+"/camera_optical_frame"
            detectedtag.pose.header.seq = self.sequence_id


            #Position
            detectedtag.pose.pose.pose.position.x = tag.pose_t[0]
            detectedtag.pose.pose.pose.position.y = tag.pose_t[1]
            detectedtag.pose.pose.pose.position.z = tag.pose_t[2]
            
            #Orientation
            #ATTENTION: The old Apriltags detection node also changed the orientation by +90 degrees around the x-axis,
            #           and rotates the zero position of the tag.
            #           To understand this shift in coordinates, the Website https://eater.net/quaternions/video/intro
            #           illustrates this shift beautifully when the first operation is set to [1,0,0,0] (w,x,y,z) and
            #           the second is set to [0,1,0,0].
            #           Quaternions in this node are represented by a list. Convention: [x,y,z,w] (utils.py).
            #           Frame of reference before:  x right, y down, z in the tag
            #           Frame of reference after:   x right, y in the tag, z up
            #           Transformation r_new = r_old*[1,0,0,0] ;where [x,y,z,w] and * is vectorproduct

            rot = ap3.rot2quat(tag.pose_R)
            rot = ap3.quatmul(rot,self.rotation_quaternion)
            detectedtag.pose.pose.pose.orientation.x = rot[0]
            detectedtag.pose.pose.pose.orientation.y = rot[1]
            detectedtag.pose.pose.pose.orientation.z = rot[2]
            detectedtag.pose.pose.pose.orientation.w = rot[3]

            detected_list.append(detectedtag)

        #Generation of output message
        output_msg = AprilTagDetectionArray()

        output_msg.detections = detected_list
        output_msg.header.stamp = time
        output_msg.header.frame_id = rospy.get_namespace()[1:-1]+"/camera_optical_frame"
        output_msg.header.seq = self.sequence_id

        self.detections.publish(output_msg)

        self.sequence_id += 1
        
        return

    #Passing of camera intrinsic parameters
    def Camera_params(self,msg):

        self.camera_intrinsics[0] = msg.K[0]
        self.camera_intrinsics[1] = msg.K[4]
        self.camera_intrinsics[2] = msg.K[2]
        self.camera_intrinsics[3] = msg.K[5]

        return

    #Trigger of FSM node
    def trigger(self,msg):

        self.switch = msg.data
        
        return
    

if __name__ == '__main__':
    rospy.init_node('apriltag3_detection_node', anonymous=False)
    node = detector_node()
    rospy.spin()

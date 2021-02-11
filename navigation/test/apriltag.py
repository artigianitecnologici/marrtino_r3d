#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError

TOPIC_tag_detections = 'tag_detections'
tag_sub = None # tag_detection subscriber
from apriltag_ros.msg import AprilTagDetectionArray
tag_sub = rospy.Subscriber(TOPIC_tag_detections, AprilTagDetectionArray, tag_cb)

# ROS Callback functions


def tag_cb(data):
    global tag_trigger_, tag_count, tag_id_, tag_distance_, tag_angle_
    v = data.detections
    if (len(v)>0):
        tag_id_ = v[0].id
        tag_distance_ = v[0].pose.pose.position.z
        tag_angle_ = math.atan2(-v[0].pose.pose.position.x,v[0].pose.pose.position.z)*180.0/math.pi

        tag_trigger_ = True
        tag_count = 3 # about seconds
        print 'tag ',tag_id_,' distance ',tag_distance_
        print 'tag trigger = ',tag_trigger_
    else:
        if (tag_trigger):
            tag_count = tag_count - 1
            print 'tag count = ',tag_count
            if (tag_count==0):
                tag_trigger_ = False
                tag_id_ = -1
                tag_distance_ = 0
                tag_angle_ = 0


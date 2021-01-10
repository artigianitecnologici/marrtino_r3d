import math, time

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
import tf

TOPIC_odom = '/odom'
FRAME_map = 'map'
FRAME_base = 'base_frame'

odom_robot_pose = [0, 0, 0]
map_robot_pose = [0, 0, 0]
odomcount = 0 
odomframe = ''

listener = None

def odom_cb(data):
    global odom_robot_pose, odomcount, odomframe
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw
    odomcount += 1
    odomframe = data.header.frame_id


def get_robot_pose():
    global map_robot_pose, listener

    if listener is None:
        listener = tf.TransformListener()
        rospy.Rate(5).sleep()

    try:
        (trans,rot) = listener.lookupTransform(FRAME_map, FRAME_base, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    map_robot_pose[0] = trans[0]
    map_robot_pose[1] = trans[1]
    map_robot_pose[2] = yaw

    return map_robot_pose


def DEG(a):
    return a*180.0/math.pi

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))


# main
if __name__ == "__main__":

    rospy.init_node('getpose', disable_signals=True)

    listener = tf.TransformListener()
    odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)

    time.sleep(1)

    p = get_robot_pose()
    print p
    print("Robot pose: %s"  %(pose_str(p)))

    odom_sub.unregister()








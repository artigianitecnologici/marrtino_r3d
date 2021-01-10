# coding=utf-8

#questa versione non prende come parametro il t della porta
import math
import sys,os,time
import rospy
import actionlib
from threading import Thread
from nav_msgs.msg import Odometry
import tf
from apriltag_ros.msg import AprilTagDetectionArray

# sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program") #serve a fare poi l'import di robot_cmd
#
# import robot_cmd_ros
# from robot_cmd_ros import *

TOPIC_odom = '/odom'
FRAME_map = 'map'
FRAME_base = 'base_frame'

odom_robot_pose = [0, 0, 0]
map_robot_pose = [0, 0, 0]
odomcount = 0
odomframe = ''

TAG_TOPIC = "tag_detections"
TAG_MSG = AprilTagDetectionArray
last_detection = (None,None,None,None)
def get_last_detection():
    global last_detection
    return last_detection

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
    return "%.2f %.2f %.2f" %(p[0], p[1], DEG(p[2]))

def tag_callback(data):
    global last_detection
    if not data.detections: return
    tag_id = data.detections[0].id[0]
    x = data.detections[0].pose.pose.pose.position.x
    y = data.detections[0].pose.pose.pose.position.y
    z = data.detections[0].pose.pose.pose.orientation.x
    last_detection = (tag_id,x,y,z)

# main, NB: DA UTILIZZARE IN MODO CHE LE TRAIETTORIE CHE SI USANO PER MUOVERE IL ROBOT SIANO PIÙ
#RETTILINEE POSSIBILI
#PENSARE DI PASSARE UNO PORTA SEMPRE ANDANDO AVANTI (FORWARD)
if __name__ == "__main__":
    with open("DRsetup.txt",'w') as file:
        ris = "Nodi:\n"
        archi= "Archi:\n"
        list = []
        prec =""

        rospy.init_node('getpose', disable_signals=True)
        listener = tf.TransformListener()
        odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)

        tag_sub = rospy.Subscriber(TAG_TOPIC, TAG_MSG, tag_callback, queue_size=1)

        while (True):
            scan = raw_input("Inserisci 'nome_nodo' del punto attuale su simulatore, 'fine' per terminare\n")
            if (scan == 'fine'):
               break
            scan2 = raw_input("c'è un tag? si o no\n")
            (tag_id, pose_x, pose_y, pose_z) = ("no","a","a","a")
            if(scan2!="no"):
        
                (tag_id, pose_x, pose_y, pose_z) = get_last_detection()

            time.sleep(1)

            p = get_robot_pose()
            list.append(scan)
            line="{} {} {} {} {} {}  \n".format(scan,(pose_str(p)),tag_id,pose_x,pose_y,pose_z)
            sys.stdout.write(line) #print con end=''
            ris+=line
            if len(prec)>0:
                scan2=raw_input("è adiacente al nodo precedente " + prec + '?\n')
                p=scan2.split()
                if p[0] == 't':
                    archi+= "{} {}\n".format(prec,scan)
            prec=scan

        odom_sub.unregister()
        while(True):
            sys.stdout.write(archi)
            print("Questi sono i nodi:" + list.__str__())
            scan = raw_input("Inserisci altri archi nel formato: 'nodo1 nodo2', 'fine' per terminare\n")
            if (scan == 'fine'):
               break
            archi += scan+"\n"
        ris+=archi
        file.write(ris)
        print(ris)

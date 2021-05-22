#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import tf
import math

rospy.init_node("testmovimenti")


global odom_robot_pose, map_robot_pose, gt_robot_pose






odom_robot_pose = [0,0,0]
odom_robot_vel = [0,0]
# Angle functions

def DEG2RAD(a):
    return a*math.pi/180.0

def RAD2DEG(a):
    return a/math.pi*180.0

def NORM_180(a):
    if (a>180):
        return a-360
    elif (a<-180):
        return a+360
    else:
        return a


def NORM_PI(a):
    if (a>math.pi):
        return a-2*math.pi
    elif (a<-math.pi):
        return a+2*math.pi
    else:
        return a

def getRobotPose(frame=None):
    return get_robot_pose(frame)

def get_robot_pose(frame=None): # returns [x,y,theta]
                         # frame: 'odom', 'map', 'gt'
    global odom_robot_pose, map_robot_pose, gt_robot_pose

    if frame==None: # auto detect
        if map_robot_pose is not None:
            return list(map_robot_pose)
        else:
            return list(odom_robot_pose)
    elif frame=='odom':
        return list(odom_robot_pose)
    elif frame=='map':
        return list(map_robot_pose)
    else: # frame=='gt':
        return list(gt_robot_pose)


def getRobotVel():
    return get_robot_vel()

def get_robot_vel():
    global odom_robot_vel
    return list(odom_robot_vel)

def odom_cb(data):
    global odom_robot_pose, odom_robot_vel
    
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw
    #odomframe = data.header.frame_id

    
    odom_robot_vel[0] = data.twist.twist.linear.x
    odom_robot_vel[1] = data.twist.twist.angular.z

PI = 3.1415926535897


def turn(angle,vel):
    rospy.loginfo("turning {} rad at {} rad/sec".format(angle,vel))
    if angle < 0:
        vel=-vel
        angle=-angle

    vel_msg = Twist()
    vel_msg.angular.z = vel
    vel_msg.linear.x= 0
    vel_msg.linear.y= 0
    vel_msg.linear.z= 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    t0 = rospy.Time.now().to_sec()
    
    current_angle = 0
    velocity_publisher.publish(vel_msg)

    while(current_angle < angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (abs(vel)*(t1-t0))
        rate.sleep()
    velocity_publisher.publish(vel_msg)
    rate.sleep()
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("done")

def forward(d,vel):
    rospy.loginfo("forward {} m at {} m/sec".format(d,vel))
    vel_msg = Twist()
    vel_msg.linear.x = vel

    t0 = rospy.Time.now().to_sec()
    while t0<=0:
        t0 = rospy.Time.now().to_sec()
    current_pos = 0
    while(current_pos < d):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_pos = (vel*(t1-t0))
        rate.sleep()
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("done")


def rotate(angle,vel):
     
     
    rospy.loginfo("turning {} rad at {} rad/sec".format(angle,vel))
    vel_msg = Twist()

   
    clockwise = True

    #Converting from angles to radians
    angular_speed = vel 
    relative_angle = angle 

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if (relative_angle) < 0 :
        relative_angle = abs(relative_angle)
        clockwise = False

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    
    current_angle = 0
    # get pose odom 
    robot_pose = get_robot_pose('odom')
    current_th = robot_pose[2]
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (angular_speed*(t1-t0))
        rate.sleep()

    velocity_publisher.publish(vel_msg)
    rate.sleep()

    robot_pose = get_robot_pose('odom')
    new_th = RAD2DEG( current_th - robot_pose[2]  )
    rospy.loginfo("odom check angolo ruotato {} ".format(new_th))
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    

def rotate_odom(angle,vel):
     
     
    rospy.loginfo("turning {} rad at {} rad/sec".format(angle,vel))
    vel_msg = Twist()

   
    clockwise = True

    #Converting from angles to radians
    angular_speed = vel 
    relative_angle = angle 

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if (relative_angle) < 0 :
        relative_angle = abs(relative_angle)
        clockwise = False

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    # get pose odom 
    robot_pose = get_robot_pose('odom')
    current_th = robot_pose[2]
    t1=0
    while((relative_angle - current_th ) > 0 ):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        robot_pose = get_robot_pose('odom')
        current_th = robot_pose[2]
        print RAD2DEG(NORM_PI(current_th)),RAD2DEG(relative_angle-current_th)
        #rate.sleep()

    velocity_publisher.publish(vel_msg)
    rate.sleep()

    robot_pose = get_robot_pose('odom')
    new_th = RAD2DEG(robot_pose[2] - current_th)
    
    rospy.loginfo("start {} current  {}  delta {} ".format(current_th,robot_pose[2],new_th))
    #
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("")
    rospy.loginfo("done")

if __name__ == '__main__':
    try:
        # Testing our function
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
        print("Waiting for robot pose... (5 seconds)")
        delay = 15 # 0.25 # sec
        #delay = input("Type test 1=turn 2=rotate 3=forward :")
        #rate = rospy.Rate(1/delay) # Hz
        rate = rospy.Rate(delay)
        rate.sleep()
        
        timeout = 5 #seconds
        t0 = rospy.Time.now().to_sec()
        
        while (odom_robot_pose is None and timeout>0):
            rate.sleep()
            timeout -= delay

        t1 = rospy.Time.now().to_sec()
        print(t1-t0)
        print "Current delay   ", delay 

        

        test = input("Type test 1=turn 2=rotate 3=forward :")


        if (test == 1 or test == 2 or test==3 ) :
            speed = input("Input your speed (degree/sec) 24:")
            angle = input("Type your distance (degrees):")



        #Converting from angles to radians
        factore = 1
        r_speed = speed*2*PI/360
        r_angle = angle*factore*2*PI/360


        if (test == 1 ):
            turn(r_angle,r_speed)

        if (test == 2 ):
            contatore = 0
            rotate(r_angle,r_speed)
            

        if (test == 3 ):
            rotate_odom(r_angle,r_speed)

    except rospy.ROSInterruptException:
        pass

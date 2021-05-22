#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
PI = 3.1415926535897

def rotate(angle,speed):
    #Starts a new node
    
    vel_msg = Twist()

    
    clockwise = 1 #input("Clockwise? 0 1: ") #True or false

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

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
    print "rel angle ",relative_angle,"ang.speed ",angular_speed
    while(current_angle < relative_angle):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (angular_speed*(t1-t0)) #*0.50

    print "End rotate"
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)
    #rospy.spin()

def turn(angle,vel):
    rospy.loginfo("turning {} rad at {} rad/sec".format(angle,vel))
    #Converting from angles to radians
    angular_speed = vel*2*PI/360
    relative_angle = angle*2*PI/360
    
    vel_msg = Twist()
    vel_msg.angular.z = abs(angular_speed)
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    t0 = rospy.Time.now().to_sec()
    #while t0<=0:
    #    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    
    while(current_angle < relative_angle):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (vel*(t1-t0)*0.90)
        rate.sleep()
    #vel_publisher.publish(vel_msg)
    #rate.sleep()
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)
    rospy.loginfo("done")

def forward(d,vel):
    rospy.loginfo("forward {} m at {} m/sec".format(d,vel))
    vel_msg = Twist()
    vel_msg.linear.x = vel
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    t0 = rospy.Time.now().to_sec()
    #while t0<=0:
    #    t0 = rospy.Time.now().to_sec()
    current_pos = 0
    while(current_pos < d):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_pos = (vel*(t1-t0))
        #rate.sleep()
    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)
    rospy.loginfo("done")

if __name__ == '__main__':
    try:
        # Testing our function
        # Receiveing the user's input
        #Starts a new node
        rospy.init_node('robot_cleaner', anonymous=True)
        vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        rate = rospy.Rate(15.0)
        print("Test move  your robot")
        speed = input("Input your speed (m/sec) 0.2:")
        dst = input("Input distance m:")
        #sr  = input("Input your speed rotation (cm/sec) 30:")
        forward(dst,speed)
        
        #angle = input("Type your distance (degrees):")
        #clockwise = input("Clockwise? 0 1: ") #True or false



    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped

from turtlesim.msg import Pose
from math import pow, atan2, sqrt
global loc_robot_pose

class MarrtinoBot:

    def __init__(self):
        # Creates a node with name 'marrtino_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('marrtino_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',Twist, queue_size=10)
												  
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
       
        	
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = Pose()
        #print "pose x:",data
        print "X=", data.pose.pose.position.x
        print "Y=", data.pose.pose.position.y
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()
        print "Distance : ",self.euclidean_distance(goal_pose)
        print "gp",goal_pose
        
        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
           
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            #print "Linear vel :",self.linear_vel(goal_pose)
            #print "Angular vel :",self.angular_vel(goal_pose)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        print "End"

        # If we press control + C, the node will stop.
        #rospy.spin()

if __name__ == '__main__':
    try:
       
        x = MarrtinoBot()
        x.move2goal()
        print "End"

    except rospy.ROSInterruptException:
        pass


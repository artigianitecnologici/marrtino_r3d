#!/usr/bin/env python

import threading
import rospy
import actionlib
import csv
import time
import tf
import math

from sensor_msgs.msg import Range,LaserScan
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,String
from math import pow, atan2, sqrt
try:
    from apriltag_ros.msg import AprilTagDetectionArray
    AprilTagFound = True
    rospy.loginfo("apriltag_ros found ....")
except:
    rospy.loginfo("apriltag_ros not found")
    AprilTagFound = False

waypoints = []

obstacle_distance = 999


class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        #
        self.localizer_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,self.localizer_cb)
        self.apriltag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
        self.range_sub = rospy.Subscriber('/teraranger_evo_mini/range',Range,self.range_cb)
        self.laser_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.laser_cb)
        rospy.loginfo('Init follow path')
	    # Get a move_base action client
        #self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #rospy.loginfo('Connecting to move_base...')
        #self.client.wait_for_server()
        #rospy.loginfo('Connected to move_base.')
        global obstacle_distance
        obstacle_distance = 999

    def laser_cb(data):
        global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
        nc = len(data.ranges)/2
        nr = int((data.angle_max - math.pi/2)/data.angle_increment)
        nl = len(data.ranges) - nr
        laser_center_dist = min(data.ranges[nc-10:nc+10])
        try:
            laser_left_dist = min(data.ranges[nl-10:nl+10])
            laser_right_dist = min(data.ranges[nr-10:nr+10])
        except:
            pass     

    def localizer_cb(self,msg):
        global loc_robot_pose
        loc_robot_pose = msg

    def range_cb(self,msg):
        global obstacle_distance
        obstacle_distance = msg.range

    def getRangeObstacle(self):  
        global obstacle_distance
        return obstacle_distance    
        
        
    def tag_cb(self,msg):
        global tag_trigger_, tag_count, tag_id_, tag_distance_, tag_angle_
        v = msg.detections
        #print v
        if (len(v)>0):
            tag_id_ = v[0].id
            tag_distance_ = v[0].pose.pose.pose.position.z
            tag_angle_ = math.atan2(-v[0].pose.pose.pose.position.x,v[0].pose.pose.pose.position.z)*180.0/math.pi

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
	
    
    def getRobotPose(self):  
        global loc_robot_pose
        return loc_robot_pose

    def laser_center_distance():
        global laser_center_dist
        return laser_center_dist
    
    def calc_distance(self, x_source,y_source,x_target,y_target ):
        return sqrt(pow((x_target - x_source), 2) +  pow((y_target - y_source), 2))	
		
    def calc_angle(self,  x_source,y_source,x_target,y_target):
        print "yt-ys,xt-xs ",y_target - y_source, x_target - x_source
        return math.atan2(y_target - y_source, x_target - x_source)
		
    def rotate_robot(self,speed,angle,clockwise):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        # verifica angle 
        
        if (angle<0):
            
            angle = abs(angle)
            clockwise = True
            if (angle > 180 ):
                angle = 360 - angle
                clockwise = False
                print "> 180 ",angle

        else:
            clockwise = False
            if (angle > 180 ):
                angle = 360 - angle
                clockwise = True
                print "< 180 ",angle

        
        # Converting from angles to radians
        angular_speed = speed*2*math.pi/360
        relative_angle = angle*2*math.pi/360
        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        

        #print "--------",relative_angle
        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
            
        # Setting the current time for distance calculus
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        #print "rel angle ",relative_angle
        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = (angular_speed*(t1-t0))
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def sendMoveMsg(self,linear,angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular 
        self.velocity_publisher.publish(vel_msg)

    def forward_robot(self,speed,distance):
        
        vel_msg = Twist()
        pose_robot = self.getRobotPose()

        initialX = pose_robot.pose.pose.position.x
        initialY = pose_robot.pose.pose.position.y
        o = pose_robot.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        initialW = euler[2]
        #
        targetReached = False
        while ((not targetReached) and (not rospy.is_shutdown())):
            pose_robot = self.getRobotPose()
            currentX = pose_robot.pose.pose.position.x
            currentY = pose_robot.pose.pose.position.y
            currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
            if (currentDistance >= distance):
                #print("Robot stopped")
                targetReached = True
                self.sendMoveMsg(0, 0)
            else:
                # Moving robot
                # check ostacolo
                #obstacle = self.getRangeObstacle()
                obstacle_laser = self.laser_center_distance()
                print "obstacle laser :",obstacle_laser
                if (obstacle < 0.50):
                    print "ostacolo :",obstacle
                else:
                    self.sendMoveMsg(speed, 0)
					
                # Rate that the publishing is done at
                #self.rate.sleep()    

    

   





            
    
    def move_robot(self,y_target,x_target):
        pose_robot = self.getRobotPose()
        #
        #print pose_robot
        pose_x = pose_robot.pose.pose.position.x
        pose_y = pose_robot.pose.pose.position.y
        o = pose_robot.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        pose_w = euler[2]
        # calcola la differenza x y source e pose
        # calcola distanza e angolo rispetto alla posizione in cui si trova
        # ---------------------------------------------------------------
        delta_y = y_target - pose_y
        delta_x = x_target - pose_x
        distance = sqrt(pow(delta_y,2)+pow(delta_x,2)) 
        ad = math.atan2(delta_y,delta_x) 
        angle = (ad-pose_w)*180/math.pi
        #rospy.loginfo('Current Pose (x,y,w): %s, %s,%s , %s' %  (pose_x, pose_y,pose_w, pose_w*180/math.pi))
        #rospy.loginfo('Target Pose (x,y): %s, %s dy %s  dx %s' %  (x_target, y_target,delta_y,delta_x))
        #rospy.loginfo('Distance :  %s ad %s angolo %s  ' % (distance,ad,  angle  	))	
        self.rotate_robot(0.20,angle,True)
        # 
        #time.sleep(2)
		
        #forward
        speed = 0.20
        self.forward_robot(speed,distance)
        #x = input('Wait:')

               
        


					
    def execute(self, userdata):
        global waypoints
        # Execute waypoints each in sequence
        
        no_of_points = 0
        oldx = 0
        oldy = 0
        for waypoint in waypoints:
            print "N.ro point ",no_of_points  
            no_of_points = no_of_points +1 
            #self.move_robot_xy(waypoint.target_pose.pose.position.y, waypoint.target_pose.pose.position.x)
            # is ok
            self.move_robot(waypoint.target_pose.pose.position.y, waypoint.target_pose.pose.position.x)

        return 'success'



class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')

        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
 
    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False
	
# to load points and save in array (waypoints)
        def load_points(path):
            global waypoints
            
            self.initialize_path_queue()
            no_of_points = 0
            
            with open(path.data) as csvfile:
                readCSV = csv.reader(csvfile, delimiter=',')
                for row in readCSV:
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = self.frame_id
                    goal.target_pose.pose.position.x = float(row[0])
                    goal.target_pose.pose.position.y = float(row[1])
                    goal.target_pose.pose.position.z = float(row[2])
                    goal.target_pose.pose.orientation.x = float(row[3])
                    goal.target_pose.pose.orientation.y = float(row[4])
                    goal.target_pose.pose.orientation.z = float(row[5])
                    goal.target_pose.pose.orientation.w = float(row[6])
                    waypoints.append(goal)
                    no_of_points += 1
            print("{} points are readed".format(no_of_points))
            return

        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        topic = "/load_paths"
        #rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        #rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                path = rospy.wait_for_message(topic, String, timeout=1)
                load_points(path)
                self.path_ready = True
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # to check whether follow path command is given
                else:
                    raise e

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.pub = rospy.Publisher('way_cmp', String, queue_size=10)
    def execute(self, userdata):
        self.pub.publish("finished")
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        # azzerare getPath
        global waypoints
        waypoints = [] # the waypoint queue
        return 'success'

if __name__ == "__main__":
    rospy.init_node('follow_waypoints')
    rospy.loginfo('Follow Waypoint v.1.0')
    rospy.loginfo('---------------------')
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},remapping={'waypoints':'waypoints'})
                           
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'}) # Fine 


#
    outcome = sm.execute()

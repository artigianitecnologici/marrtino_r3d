#import
import os,sys
import threading,time
from sensor_msgs.msg import Range
from math import pow, atan2, sqrt
from DRmn2 import *
import tf
# navigation with cmd_vel
from geometry_msgs.msg import Twist
PI = 3.1415926535897

#sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')  #sys and os
from robot_cmd_ros import *

#class marrtino_bot
plinear = 1
global pose_prec

def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

def move2goal(p2):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    goal_x=float(p2[0])
    goal_y=float(p2[1])
    robot_pose = getRobotPose()

    distance_tolerance = 0.10
    
    vel_msg = Twist()
    distanza = sqrt(pow((goal_x - robot_pose[0]), 2) + pow((goal_y - robot_pose[1]), 2))
    while distanza >= distance_tolerance:

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control

        # Linear velocity in the x-axis.
        vel_msg.linear.x = 10
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0 #angular_vel(goal_pose)
        # Publishing our vel_msg
        velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
      #  self.rate.sleep()
        robot_pose = getRobotPose()
        distanza = sqrt(pow((goal_x - robot_pose[0]), 2) + pow((goal_y - robot_pose[1]), 2))
        print "Distanza ",distanza
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        #wt=int(input("premi un tasto"))
        # Stopping our robot after the movement is over.
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
#
# move : 
#
def move(speed,distance,isforward):
    #Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    delay = 0.1 # sec
    rate = rospy.Rate(1/delay) # Hz      
    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    # Checking if our movement is CW or CCW
    if isforward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    
    while((current_distance) < distance):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0) * 0.85
        #print "cd",current_distance
        rate.sleep()

    
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

#
# rotate : 
#
def rotate(speed,angle,clockwise):
    #Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

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
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (angular_speed*(t1-t0))*0.96

    #print "End rotate"
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

#dist e angle
def dist(p1,p2):
    x=float(p2[0])-float(p1[0])
    y=float(p2[1])-float(p1[1])
    t = math.sqrt(x*x+y*y)
    #print "distanza calcolata",t
    return t
    

def turntogoal(source_pose,target_pose):
    r = True
	print "Source ",source_pose
    print "Target ",target_pose
	
    p = source_pose #getRobotPose()
   
    ad = math.atan2(float(target_pose[1])-float(source_pose[1]),float(target_pose[0])-float(source_pose[0]))
     
    angle = (ad-float(source_pose[2]))*180/math.pi
    speed = 30
    #
    #
    if (angle > 0):
        clockwise = False
    else:
        clockwise = True
        if (angle > 180):
            angle = 360 - angle
            clockwise = False
            

    print "angolo ",angle
    #
    #rotate(speed,abs(angle),clockwise)
    #
    return r




#global variables 
distance = 0
angle = 0
turning = False
obstacle = False
navigable = True
p1 = None
p2 = None
ftoll = 1 #forward tollerance
atoll = -9998 #angle tollerance

#update distance/state
def upState(pprec,pnode):
    global path,distance,p1,p2,angle,navigable,temp

    if len(path)==0:
        navigable = False
    else:
        p1 = pprec #getRobotPose()
        #print "posizione attuale",p1
        #print "devo arrivare a ",p2
        
        p2 = pnode #path[0].point().strip().split()
        angle = turntogoal(p1,p2) #angolo dovrebbe rimanere costante
       # angle = angles(temp,p2) #angolo dovrebbe rimanere costante
        distance = dist(p1,p2)
        print "p1:",p1," p2:",p2
        pose_prec = p2
        # possiamo sapere se e' stato spostato
        rspeed = 0.3
        #forward(distance)
        #move(rspeed,distance,True)
        time.sleep(2)       
        p1 = getRobotPose()
        distance2 = dist(p1,p2)
        
        print "Nodo :",p2, " angolo " , angle , " Distanza " ,distance," Scarto 2 ",distance2
        

#Laser Distance Check
def ldc(msg):
    global distance,obstacle,turning,navigable

    if navigable and not turning:
        upState()
        #print "Distanze"
        #print(msg.range)
        #print distance
        if msg.range<distance:
            if not obstacle:
                print "Ostacolo rilevato su strada"
                if msg.range<=0.25:
                    obstacle = True
                    print "Ostacolo troppo vicino, stop in corso"
                    try:
                        robot_stop_request()
                    except:
                        "Stop eseguito"
                        
        else:   
            if obstacle:
                begin(nodename='DRnavX', use_desired_cmd_vel=True)
            obstacle = False




#main
if __name__ == "__main__":
    
    begin(nodename='DRnavX', use_desired_cmd_vel=True)
    g = Graph.read('DRsetup.txt') #possibile modifica con sys.argv[_]
    path = g.navigate(g.getNode(sys.argv[1]),g.getNode(sys.argv[2]))
    #print "elementi"
    #print len(path)
    #print(path)
    #print "temp -----------------------"
    #for i in range(len(path)):
    #    temp = path.pop(0).point().split()
    #    print temp
    #print "temp -----------------------"
    #upState()
    sub = rospy.Subscriber('/teraranger_evo_mini/range',Range,ldc)
    
    
    #while (navigable):
    # l'intercettazione  dell' ostacolo  e del tag fa effettuata dentro MOVE_REL
    print "Inizio Navigazione "
    #pose_prec = getRobotPose()
    for i in range(len(path)-1):
        
        p = path.pop(1).point().split()
        p0 = path.pop(0).point().split()
        print " p ",p
        print " p0 ",p0
        upState(p0,p)

        #time.sleep(2)
        #r = forward(distance, False)
        #time.sleep(2)
        # 
        

#    p = path.pop(1).point().split()
#    print p

#    upState(p)
#    r = forward(distance, False)
#    print "sono a", getRobotPose()
    
       
    
    print "Fine Navigazione " 
    #print "Controllo Tag"
    tagdetect=tag_trigger()
    if (tagdetect==True):
        tag_angle=tag_angle()
        tag_distance=tag_distance()
        tag_id=tag_id()
        print 'tag ',tag_id,' distance ',tag_distance ,' tag_angle',tag_angle
       


    # controllo posizione con amcl_pose

     #turn(float(temp[2]), 'Abs') #final turn
    sub.unregister()
    end()

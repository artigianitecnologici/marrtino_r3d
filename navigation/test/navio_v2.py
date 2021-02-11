import rospy
import os,sys
import threading,time
from sensor_msgs.msg import Range
from math import pow, atan2, sqrt
from DRmn2 import *
import tf
# navigation with cmd_vel
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
PI = 3.1415926535897
# Good values
tv_good = 0.2
rv_good = 0.8
tv_min = 0.1
rv_min = 0.2

move_step = 1.0
stop_request = False
# robot pose from localization
loc_robot_pose = [0,0,0]

#sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')  #sys and os
#from robot_cmd_ros import *

#class marrtino_bot



def getRobotPose(): # returns [x,y,theta]
    global loc_robot_pose
    return list(loc_robot_pose)

# tx    
def exec_move_REL(tx):
    global tv_good, odom_robot_pose
    start_pose = getRobotPose()
    tv_nom = tv_good 
    r = True
    if (tx < 0):
        tv_nom *= -1
        tx *= -1
    odom_robot_pose = getRobotPose()
    dx = abs(dist(start_pose,odom_robot_pose) - tx)
    print "Distanza dx ",dx
    while (dx>0.05):
        tv = tv_nom
        if (dx<0.5):
            tv = tv_nom*dx/0.5
        if (abs(tv)<tv_min):
            tv = tv_min*tv/abs(tv)
        rv = 0.0
        if setSpeed(tv, rv, 0.1, False):
            pose =  getRobotPose()
            dx = abs(dist(start_pose, pose) - tx)
        else:
            print("move action canceled by user")
            r = False
            dx = 0
        #print("MOVE -- POS: %.1f %.1f %.1f -- targetTX %.1f DX %.1f -- VEL: %.2f %.2f" %(pose[0], pose[1], RAD2DEG(pose[2]), tx, dx, tv, rv))
    setSpeed(0.0,0.0,0.1)
    return r
def setSpeed(lx,az,tm,stopend=False):
    return set_speed(lx,az,tm,stopend)

# lx - velocita' lineare
# az - velocita' angolare
# tm - time
#
def set_speed(lx,az,tm,stopend=False):
    global cmd_pub, stop_request

    if (stop_request):
        raise Exception("setSpeed called in stop_request mode")

    delay = 0.1 # sec
    rate = rospy.Rate(1/delay) # Hz
    cnt = 0.0
    msg = Twist()
    msg.linear.x = lx
    msg.angular.z = az
    msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown() and cnt<=tm and not stop_request:
        velocity_publisher.publish(msg)
        cnt = cnt + delay
        try:
            rate.sleep()
        except KeyboardInterrupt:
            print("User KeyboardInterrupt")
            return False
    if (stopend):
        msg.linear.x = 0
        msg.angular.z = 0
        velocity_publisher.publish(msg)
        rate.sleep()
    return True


def localizer_cb(data):
    global loc_robot_pose
    if (loc_robot_pose is None):
        loc_robot_pose = [0,0,0]
    loc_robot_pose[0] = data.pose.pose.position.x
    loc_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    loc_robot_pose[2] = euler[2] # yaw



def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

def move2(p2):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    goal_x=float(p2[0])
    goal_y=float(p2[1])
    robot_pose = getRobotPose()
    vel_msg = Twist()
    distance_rel = sqrt(pow((goal_x - robot_pose[0]), 2) + pow((goal_y - robot_pose[1]), 2))
    linear_speed = 0.1
    # Linear velocity in the x-axis.
    vel_msg.linear.x = linear_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    # Angular velocity in the z-axis.
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0 #angular_vel(goal_pose)


    distance_tolerance = 0.10
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < distance_rel):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = linear_speed*(t1-t0)
    
        
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def forward(r=1, obstacleAvoidance=False):
    global tv_good
    #print 'forward',r
    #if obstacleAvoidance:
    #    enableObstacleAvoidance(True)
    move_step = 1
    v = exec_move_REL(move_step*r)
    #if obstacleAvoidance:
    #    enableObstacleAvoidance(False)
    return v



#
# move : 
#
def move(goal_pose,speed,distance,isforward):
    #Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
      
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
    current_pose = getRobotPose()
    distance = dist(current_pose,goal_pose)

    while(distance > 0.10):

        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_pose = getRobotPose()
        distance = dist(current_pose,goal_pose)
        print "distance ",distance
    
    vel_msg.angular.x = 0
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
        current_angle = angular_speed*(t1-t0)

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
    

def turntogoal(target_pose):
    r = True
    #print target_pose
    p = getRobotPose()

    print "pose",p
    ad = math.atan2(float(target_pose[1])-p[1],float(target_pose[0])-p[0])
    
    angle = (ad-p[2])*180/math.pi
    speed = 30
    #
    #

    if (angle > 0):
        clockwise = False
    else:
        clockwise = True


    #
    rotate(speed,abs(angle),clockwise)
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
def upState(pnode):
    global path,distance,p1,p2,angle,navigable,temp

    if len(path)==0:
        navigable = False
    else:
        p1 = getRobotPose()
        #print "posizione attuale",p1
        #print "devo arrivare a ",p2
        
        p2 = pnode #path[0].point().strip().split()
        angle = turntogoal(p2) #angolo dovrebbe rimanere costante
       # angle = angles(temp,p2) #angolo dovrebbe rimanere costante
        distance = dist(p1,p2)
        # possiamo sapere se e' stato spostato
        #rspeed = 0.1
        #move(p2,rspeed,distance,True)
        time.sleep(3)
        #move2(p2)
        forward(distance)
        p1 = getRobotPose()
        scarto = dist(p1,p2)
        print "Nodo :",p2, " angolo " , angle , " Distanza " ,distance," Scarto ",scarto
        

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
    
    # begin(nodename='navio', use_desired_cmd_vel=True)
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
    rospy.init_node('navio')
    sub = rospy.Subscriber('/teraranger_evo_mini/range',Range,ldc)
    localizer_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, localizer_cb)
    
    rate = rospy.Rate(1)

    #while (navigable):
    # l'intercettazione  dell' ostacolo  e del tag fa effettuata dentro MOVE_REL
    print "Inizio Navigazione "
    for i in range(len(path)-1):
        
        p = path.pop(1).point().split()

        upState(p)
        

  
       
    
    print "Fine Navigazione " 
    print "Controllo Tag"
    #tagdetect=tag_trigger()
    #if (tagdetect==True):
    #    tag_angle=tag_angle()
    #    tag_distance=tag_distance()
    #    tag_id=tag_id()
    #    print 'tag ',tag_id,' distance ',tag_distance ,' tag_angle',tag_angle
       


    # controllo posizione con amcl_pose

     #turn(float(temp[2]), 'Abs') #final turn
    localizer_sub.unregister()
    sub.unregister()
    #end()

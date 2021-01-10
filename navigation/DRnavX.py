#import
import os,sys
import threading,time
from sensor_msgs.msg import Range
from DRmn2 import *
import tf
sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')  #sys and os
from robot_cmd_ros import *

#dist e angle
def dist(p1,p2):
    x=float(p2[0])-float(p1[0])
    y=float(p2[1])-float(p1[1])
    t = math.sqrt(x*x+y*y)
    print "distanza calcolata",t
    return t
    

def turntogoal(target_pose):
    r = True
    #print target_pose
    p = getRobotPose()

    if math.fabs(float(target_pose[1])-p[1]) + math.fabs(float(target_pose[0])-p[0]) < 0.05:
        return True

    ad = math.atan2(float(target_pose[1])-p[1],float(target_pose[0])-p[0])
    th_deg = (ad-p[2])*180/math.pi
    #se l'angolo e' maggiore di 30
    
    if math.fabs(th_deg)>20:
         r = turn(th_deg)

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
def upState():
    global path,distance,p1,p2,angle,navigable,temp

    if len(path)==0:
        navigable = False
    else:
        p1 = getRobotPose()
        #print "posizione attuale",p1
        #print "devo arrivare a ",p2
        
        p2 = path[0].point().strip().split()
        angle = turntogoal(p2) #angolo dovrebbe rimanere costante
       # angle = angles(temp,p2) #angolo dovrebbe rimanere costante
        distance = dist(p1,p2)
        print "Distanza : ",distance

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
    
    temp = path.pop(0).point().split()
    #upState()
    sub = rospy.Subscriber('/teraranger_evo_mini/range',Range,ldc)
    
    
    while (navigable):
        upState()
          
        if (obstacle):
            print "Rimuovere ostacolo"
            time.sleep(2)
            continue
        else:
        
            r = False
            try:
              while True:
                    
                    upState()
                    
                 
                    if(distance>ftoll):
                        r = forward(ftoll, False)
                    else:
                        r = forward(distance, False)
                        break
            except:
                print "stop eseguito"

            if r:
                temp = path.pop(0).point().split()
                print "dovevo arrivare a", temp
                print "sono a", getRobotPose()
        upState()
     #turn(float(temp[2]), 'Abs') #final turn
    sub.unregister()
    end()

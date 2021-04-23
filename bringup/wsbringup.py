from __future__ import print_function

import socket
import time
import os

from threading import Thread

import sys

# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H pip install tornado

try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except Exception as e:
    print(e)
    print('Install tornado: pip install --user tornado')
    sys.exit(0)

#sys.path.append('../program')
#sys.path.append('../scripts')

#import check
#from check import *

from tmuxsend import TmuxSend

# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_name = 'Bringup'     # server name
server_port = 9990          # web server port
status = "Idle"             # robot status sent to websocket


# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def checkStatus(self, what='ALL'):

        self.setStatus('Checking...')

        

        self.setStatus('Idle')
        time.sleep(1)
        self.setStatus('Idle')


    def setStatus(self, st):
        global status
        status = st
        self.write_message('STATUS %s' %status)


    def open(self):
        global websocket_server, run
        websocket_server = self
        print('>>> New connection <<<')
        self.setStatus('Executing...')
        self.winlist = ['cmd','roscore','quit','wsrobot','modim',
                        'robot','laser','rviz','imgproc','joystick','audio',
                        'map_loc','navigation','playground','netcat']

        self.wroscore = self.winlist.index('roscore')
        self.wrobot = self.winlist.index('robot')
        self.wlaser = self.winlist.index('laser')
        self.wrviz = self.winlist.index('rviz')
        self.wimgproc = self.winlist.index('imgproc')
        self.wjoystick = self.winlist.index('joystick')
        self.waudio = self.winlist.index('audio')
        self.wwsrobot = self.winlist.index('wsrobot')
        self.wquit = self.winlist.index('quit')
        self.wmodim = self.winlist.index('modim')
        self.wmaploc = self.winlist.index('map_loc')
        self.wnav = self.winlist.index('navigation')
        self.wplayground = self.winlist.index('playground')
        self.wnet = self.winlist.index('netcat')

        self.tmux = TmuxSend('bringup',self.winlist)
        self.tmux.roscore(self.wroscore)
        time.sleep(1)
        #self.tmux.cmd(self.wmodim,'cd $MODIM_HOME/src/GUI')
        #self.tmux.cmd(self.wmodim,'python ws_server.py -robot marrtino')
        time.sleep(1)
        #self.wsrobot()
        #time.sleep(3)

        self.checkStatus()

        print("----")
        sys.stdout.flush()
       

    def waitfor(self, what, timeout):
        time.sleep(2)
        r = check_it(what)
        while not r and timeout>0:
            time.sleep(1)
            timeout -= 1
            r = check_it(what)
        self.write_message('RESULT %s %s' %(what,str(r)))



    def on_message(self, message):    
        print('>>> MESSAGE RECEIVED: %s <<<' %message)
        self.setStatus(message)

        try:
            self.process_message(message)
        except:
            print("Error in message %s" %message)

        print("----")
        sys.stdout.flush()

        self.setStatus('Idle')

    def process_message(self, message):
        global code, status

        print('Code --> received:\n%s' %message)
 
        if (message=='stop'):
            print('!!! EMERGENCY STOP !!!')
            self.checkStatus()

        elif (message=='check'):
            self.checkStatus()

        elif (message=='ros_quit'):
            self.tmux.quitall(range(5,len(self.winlist)))
            self.checkStatus()

        # robot start/stop
        elif (message=='robot_start'):
            self.tmux.cmd(self.wnet,"echo '@robot' | netcat -w 1 localhost 9236")
            self.tmux.roslaunch(self.wrobot,'launch','bringup')
            #self.waitfor('robot',5)
            self.waitfor('odom',1)
            self.waitfor('sonar',1)
        elif (message=='robot_kill'):
            self.tmux.cmd(self.wnet,"echo '@robotkill' | netcat -w 1 localhost 9236")
            self.tmux.roskill('orazio')
            self.tmux.roskill('state_pub_robot')
            time.sleep(1)
            self.tmux.killall(self.wrobot)
            time.sleep(1)
            if check_robot():
                self.tmux.cmd(wquit,"kill -9 `ps ax | grep websocket_robot | awk '{print $1}'`")
                time.sleep(1)
            while check_robot():
                time.sleep(1)
            self.write_message('RESULT robot False')
            #self.checkStatus('robot')


        

        # simrobot start/stop
        elif (message[0:14]=='simrobot_start'):
            self.tmux.roslaunch(self.wrobot,'launch','simulation')
         
        elif (message=='simrobot_kill'):
            self.tmux.cmd(self.wnet,"echo '@stagekill' | netcat -w 1 localhost 9235")
            self.tmux.roskill('stageros')
            time.sleep(1)
            self.tmux.killall(self.wrobot)
            time.sleep(1)
            if check_simrobot():
                self.tmux.cmd(wquit,"kill -9 `ps ax | grep websocket_robot | awk '{print $1}'`")
                time.sleep(1)
            while check_simrobot():
                time.sleep(1)
            self.write_message('RESULT simrobot False')


        # wsrobot
        elif (message=='wsrobot_start'):
            self.wsrobot()
            self.checkStatus()
        elif (message=='wsrobot_kill'):
            self.tmux.cmd(self.wquit,"kill -9 `ps ax | grep websocket_robot | awk '{print $1}'`")
            time.sleep(3)
            self.checkStatus()

        # sonar
        elif (message=='read_sonars'):
            self.setStatus('Read sonars')
            for i in range(0,4):
                v = getSonarValue(i)
                self.write_message('VALUE sonar%d %.2f' %(i,v))
                print('  -- Sonar %d range = %.2f' %(i,v))
            self.setStatus('Idle')
            self.checkStatus('sonar')

        # marrtinox rviz
        elif (message=='marrtinox_rviz_start'):
            self.tmux.roslaunch(self.wrviz,'launch','marrtinox_rviz')
            self.waitfor('rgb_rviz',5)
            #time.sleep(5)
            #self.checkStatus('rviz')
        elif (message=='marrtinox_rviz_start_kill'):
            self.tmux.roskill('usb_cam')
            self.tmux.roskill('state_pub_usbcam')
            time.sleep(2)
            self.tmux.killall(self.wrviz)
            time.sleep(2)
            self.checkStatus('rviz')

        # astra
        elif (message=='astra_start'):
            self.tmux.roslaunch(self.wrviz,'rviz','astra')
            self.waitfor('rgb_rviz',5)
            self.waitfor('depth_rviz',1)
            #time.sleep(5)
            #self.checkStatus('rviz')
        elif (message=='astra_kill'):
            self.tmux.roskill('astra')
            self.tmux.roskill('state_pub_astra')
            time.sleep(2)
            self.tmux.killall(self.wrviz)
            time.sleep(2)
            self.checkStatus('rviz')

        # kinect
        elif (message=='kinect_start'):
            self.tmux.roslaunch(self.wlaser,'laser','kinect')
            #self.waitfor('rgb_rviz',5)
            #self.waitfor('depth_rviz',1)
            #time.sleep(5)
            self.checkStatus('laser')
        elif (message=='kinect_kill'):
            self.tmux.roskill('kinect')
            #self.tmux.roskill('state_pub_kinect')
            time.sleep(2)
            self.tmux.killall(self.wlaser)
            time.sleep(2)
            self.checkStatus('rviz')

        # xtion
        elif (message=='xtion_start'):
            self.tmux.roslaunch(self.wrviz,'rviz','xtion2')
            self.waitfor('rgb_rviz',5)
            self.waitfor('depth_rviz',1)
            #time.sleep(5)
            #self.checkStatus('rviz')
        elif (message=='xtion_kill'):
            self.tmux.roskill('xtion2')
            self.tmux.roskill('state_pub_xtion')
            time.sleep(2)
            self.tmux.killall(self.wrviz)
            time.sleep(2)
            self.checkStatus('rviz')


        # hokuyo
        elif (message=='hokuyo_start'):
            self.tmux.roslaunch(self.wlaser,'laser','hokuyo')
            self.waitfor('laser',5)
            #time.sleep(5)
            #self.checkStatus('laser')
        elif (message=='hokuyo_kill'):
            self.tmux.roskill('hokuyo')
            self.tmux.roskill('state_pub_laser')
            time.sleep(3)
            self.tmux.killall(self.wlaser)
            time.sleep(3)
            self.checkStatus('laser')

        # rplidar
        elif (message=='rplidar_start'):
            self.tmux.roslaunch(self.wlaser,'laser','rplidar')
            self.waitfor('laser',5)
            #time.sleep(5)
            #self.checkStatus('laser')
        elif (message=='rplidar_kill'):
            self.tmux.roskill('rplidar')
            self.tmux.roskill('state_pub_laser')
            time.sleep(3)
            self.tmux.killall(self.wlaser)
            time.sleep(3)
            self.checkStatus('laser')

        # astralaser
        elif (message=='astralaser_start'):
            self.tmux.roslaunch(self.wlaser,'laser','astra_laser')
            time.sleep(5)
            self.checkStatus('rvizlaser')
        elif (message=='astralaser_kill'):
            self.tmux.roskill('astralaser')
            self.tmux.roskill('depth2laser')
            self.tmux.roskill('state_pub_astra_laser')
            time.sleep(3)
            self.tmux.killall(self.wlaser)
            time.sleep(3)
            self.checkStatus('rvizlaser')

        # xtionlaser
        elif (message=='xtionlaser_start'):
            self.tmux.roslaunch(self.wlaser,'laser','xtion2_laser')
            time.sleep(5)
            self.checkStatus('rvizlaser')
        elif (message=='xtionlaser_kill'):
            self.tmux.roskill('xtion2laser')
            self.tmux.roskill('depth2laser')
            self.tmux.roskill('state_pub_xtion_laser')
            time.sleep(3)
            self.tmux.killall(self.wlaser)
            time.sleep(3)
            self.checkStatus('rvizlaser')

        # joystick
        elif (message=='joystick_start'):
            self.tmux.cmd(self.wnet,"echo '@joystick' | netcat -w 1 localhost 9239")
            self.tmux.roslaunch(self.wjoystick,'teleop','teleop')
            time.sleep(3)
            self.checkStatus('joystick')
        elif (message=='joystick_kill'):
            self.tmux.cmd(self.wnet,"echo '@joystickkill' | netcat -w 1 localhost 9239")
            self.tmux.roskill('joystick')
            self.tmux.roskill('joy')
            time.sleep(3)
            self.tmux.killall(self.wjoystick)
            time.sleep(3)
            self.checkStatus('joystick')


        # joystick 4wd
        elif (message=='joystick4wd_start'):
            self.tmux.roslaunch(self.wjoystick,'teleop','teleop','use_4wd:=true &')
            time.sleep(3)
            self.tmux.python(self.wjoystick,'teleop','joy4w.py')
            time.sleep(3)
            self.checkStatus('joystick')
        elif (message=='joystick4wd_kill'):
            self.tmux.roskill('joy')
            time.sleep(3)
            self.tmux.killall(self.wjoystick)
            time.sleep(3)
            self.checkStatus('joystick')


        # audio
        elif (message=='audio_start'):
            self.tmux.python(self.waudio,'audio','audio_server.py')
            time.sleep(3)
            self.checkStatus()
        elif (message=='audio_kill'):
            self.tmux.killall(self.waudio)
            time.sleep(3)
            self.checkStatus()


        # apriltags detector
        elif (message=='apriltags_start'):
            self.tmux.roslaunch(self.wimgproc,'marker','tags')
            time.sleep(3)
            self.checkStatus()
        elif (message=='apriltags_kill'):
            self.tmux.killall(self.wimgproc)
            time.sleep(3)
            self.checkStatus()



        # gmapping
        elif (message=='gmapping_start'):
            self.tmux.roslaunch(self.wmaploc,'launch','gmapping')
            time.sleep(5)
            self.checkStatus()
        elif (message=='gmapping_kill'):
            self.tmux.killall(self.wmaploc)
            time.sleep(5)
            self.checkStatus()

        # srrgmapper
        elif (message=='srrg_mapper2d_start'):
            self.tmux.roslaunch(self.wmaploc,'launch','srrg_mapper')
            time.sleep(5)
            self.checkStatus()
        elif (message=='srrg_mapper2d_kill'):
            self.tmux.killall(self.wmaploc)
            time.sleep(5)
            self.checkStatus()

        # save map
        elif (message=='save_map'):
            #self.tmux.cmd(self.wplayground,'mkdir -p ~/playground')
            self.tmux.cmd(self.wplayground,'cd ~/src/marrtino_r3d/maps')
            self.tmux.cmd(self.wplayground,'rosrun map_server map_saver -f mymap')
            self.checkStatus()
        # save waypoint 1
        elif (message=='save_waypoint01fwd'):
            self.tmux.roslaunch(self.wmaploc,'launch','waypoint_saver', 'direction:=fwd counter:=1')
            time.sleep(5)
            self.checkStatus()
        # save waypoint 2
        elif (message=='save_waypoint02fwd'):
            self.tmux.roslaunch(self.wmaploc,'launch','waypoint_saver', 'direction:=fwd counter:=2')
            time.sleep(5)
            self.checkStatus()

        # amcl
        elif (message=='amcl_start'):
            self.tmux.roslaunch(self.wmaploc,'launch','amcl')
            time.sleep(5)
            self.checkStatus()
        elif (message=='amcl_lastmap_start'):
            self.tmux.roslaunch(self.wmaploc,'launch','amcl', 'map_name:=mymap')
            time.sleep(5)
            self.checkStatus()
        elif (message=='amcl_kill'):
            self.tmux.killall(self.wmaploc)
            time.sleep(5)
            self.checkStatus()

        # srrg_localizer
        elif (message=='srrg_localizer_start'):
            self.tmux.roslaunch(self.wmaploc,'navigation','srrg_localizer')
            time.sleep(5)
            self.checkStatus()
        elif (message=='srrg_localizer_lastmap_start'):
            self.tmux.roslaunch(self.wmaploc,'navigation','srrg_localizer', 'mapsdir:=$HOME/playground map_name:=lastmap')
            time.sleep(5)
            self.checkStatus()
        elif (message=='srrg_localizer_kill'):
            self.tmux.killall(self.wmaploc)
            time.sleep(5)
            self.checkStatus()

        # gradient_based_navigation
        elif (message=='obst_avoid_start'):
            self.tmux.roslaunch(self.wnav,'navigation','obstacle_avoidance')
            time.sleep(3)
            self.checkStatus()
        elif (message=='obst_avoid_kill'):
            self.tmux.killall(self.wnav)
            time.sleep(3)
            self.checkStatus()


        # move_base
        elif (message=='move_base_node_start'):
            self.tmux.roslaunch(self.wnav,'navigation','move_base')
            time.sleep(5)
            self.checkStatus()
        elif (message=='move_base_node_kill'):
            self.tmux.killall(self.wnav)
            time.sleep(5)
            self.checkStatus()

        # spqrel_planner
        elif (message=='spqrel_planner_start'):
            self.tmux.roslaunch(self.wnav,'navigation','spqrel_planner')
            time.sleep(5)
            self.checkStatus()
        elif (message=='spqrel_planner_kill'):
            self.tmux.killall(self.wnav)
            time.sleep(5)
            self.checkStatus()

        # shutdown
        elif (message=='shutdown'):
            self.tmux.quitall()
            self.checkStatus()
            self.tmux.cmd(self.wquit,'touch ~/log/shutdownrequest')
            self.tmux.cmd(self.wquit,'sudo shutdown -h now')


        else:
            print('Code received:\n%s' %message)
            if (status=='Idle'):
                t = Thread(target=run_code, args=(message,))
                t.start()
            else:
                print('Program running. This code is discarded.')



    def on_close(self):
        print('Connection closed')

    def on_ping(self, data):
        print('ping received: %s' %(data))

    def on_pong(self, data):
        print('pong received: %s' %(data))

    def check_origin(self, origin):
        #print("-- Request from %s" %(origin))
        return True


    def wsrobot(self):
        self.tmux.python(self.wwsrobot,'blockly','websocket_robot.py')
        time.sleep(3)



# Main loop (asynchrounous thread)

def main_loop(data):
    global run, websocket_server, status
    while (run):
        time.sleep(2)
        if (run and not websocket_server is None):
            try:
                websocket_server.write_message("STATUS "+status)
                #print(status)
            except tornado.websocket.WebSocketClosedError:
                # print('-- WebSocketClosedError --')
                websocket_server = None
    print("Main loop quit.")


def run_code(code):
    global status
    if (code is None):
        return





# Main program

if __name__ == "__main__":

    # Run main thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Run web server
    application = tornado.web.Application([
        (r'/websocketserver', MyWebSocketServer),])  
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print("%s Websocket server listening on port %d" %(server_name,server_port))
    sys.stdout.flush()
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if (not websocket_server is None):
        websocket_server.close()
    print("%s Websocket server quit." %server_name)
    run = False    
    print("Waiting for main loop to quit...")



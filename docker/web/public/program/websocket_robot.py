# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H pip install tornado

import sys, os, socket, time, random
import thread2
#from thread2 import Thread
from numbers import Number

RED   = "\033[1;31m"  
BLUE  = "\033[1;34m"
CYAN  = "\033[1;36m"
GREEN = "\033[0;32m"
RESET = "\033[0;0m"
BOLD    = "\033[;1m"
REVERSE = "\033[;7m"


import sys
#sys.path.append('../program')

try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except Exception as e:
    print(e)
    print('Install tornado:   sudo -H pip install tornado')
    sys.exit(0)


import robot_cmd_ros
from robot_cmd_ros import *

robot_cmd_ros.use_robot = True

# Global variables 

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_port = 9030          # web server port
code = None
status = "Idle"             # robot status sent to websocket

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def open(self):
        global websocket_server, run
        websocket_server = self
        self.run_thread = None
        print('New connection')
    
    # messages from Javascript
    def on_message(self, message):
        global code, status
        if (message=='stop'):
            print('Stop code and robot')
            if status!='Idle':
                status = 'Stop'
            robot_stop_request()
            time.sleep(0.5)
        elif (message[0:5]=='event'):
            print('Received event %s' %message)
            v = message.split('_')
            if (v>1):
                set_global_param('event',v[1])
                print('Global param %s = %s' %('event',v[1]))
        else:
            print('Code received:\n%s' %message)
            if (status=='Idle'):
                self.run_thread = thread2.Thread(target=run_code, args=(message,))
                self.run_thread.start()
            else:
                print('Program running. This code is discarded.')
        self.write_message('OK')
  
    def on_close(self):
        print('Connection closed')
        self.run_thread = None
  
    def on_ping(self, data):
        print('ping received: %s' %(data))
  
    def on_pong(self, data):
        print('pong received: %s' %(data))
  
    def check_origin(self, origin):
        #print("-- Request from %s" %(origin))
        return True


# function to be called by programs to display text on web interface
def display(text):
    global websocket_server
    try:
        #print('  -- writing on websocket: %s' %text)
        websocket_server.write_message('display %s' %text)
    except tornado.websocket.WebSocketClosedError:
        print('Cannot write on websocket')

# Main loop (asynchrounous thread)

def main_loop(data):
    global run, websocket_server, status
    while (run):
        time.sleep(1)
        if (run and not websocket_server is None):
            try:
                websocket_server.write_message(status)
                #print(status)
            except tornado.websocket.WebSocketClosedError:
                #print('Connection closed.')
                websocket_server = None
    print("Main loop quit.")


global fncode_running
fncode_running = False

# empty function to have it defined
def fncode():
    pass

# returns the code inside definition of function fncode()
def deffunctioncode(code):
    r = "global fncode\n"
    r = r+"def fncode():\n"
    v = code.split("\n")
    incode = False
    for i in v:
        if (i=='begin()'):
            r = r+'  '+i+'\n'
            incode = True
        elif (i=='end()'):
            r = r+'  '+i+'\n'
            incode = False
        elif (incode):
            r = r+'  '+i+'\n'
    return r

# fncode with exception handling
def fncodeexcept():
    global fncode_running
    fncode_running = True
    try:
        fncode()
    except Exception as e:
        print("CODE EXECUTION ERROR")
        print e
    fncode_running = False


run_code_thread = None

def exec_thread(code):
    global run_code_thread
    fncodestr = deffunctioncode(code)
    #print(fncodestr)
    try:
        exec(fncodestr)
    except Exception as e:
        print("FN CODE DEFINITION ERROR")
        print e
    run_code_thread = thread2.Thread(target=fncodeexcept, args=())
    run_code_thread.start()
    #print "Run code thread: ", run_code_thread," started."
    # wait until thread finished
    time.sleep(1)
    global fncode_running,status
    while fncode_running and status!="Stop":
        time.sleep(0.5)
    if status=="Stop":
        # wait for normal termination
        maxtime = 3
        t = 0
        dt = 0.2
        while fncode_running and t<maxtime:
            time.sleep(dt)
            t += dt
        if fncode_running:
            # terminate
            try:
                print('Terminating code ...')
                run_code_thread.terminate()
                #print "Run code thread: ", run_code_thread," terminated."
            except Exception as e:
                print e
                #print "Thread already terminated"
    run_code_thread.join()
    run_code_thread = None    


def run_code(code):
    global status
    if (code is None):
        return
    print("Executing")
    print(code)
    status = "Executing program"
    print("=== Start code run ===")
    exec_thread(code)
    print("=== End code run ===")
    status = "Idle"


# Main program
  
if __name__ == "__main__":

    # Run main thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Run robot
    begin('websocket_robot')

    if ready():
        # Run web server
        application = tornado.web.Application([
            (r'/websocketserver', MyWebSocketServer),])  
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(server_port)
        print("%sWebsocket server listening on port %d%s" %(GREEN,server_port,RESET))
        try:
            tornado.ioloop.IOLoop.instance().start()
        except KeyboardInterrupt:
            print(" -- Keyboard interrupt --")

    # Quit
    end()

    if (not websocket_server is None):
        websocket_server.close()
    print("Web server quit.")
    run = False    
    print("Waiting for main loop to quit...")
    t.join()


#    if run_code_thread != None:
#        print('Program execution still alive!!!')        


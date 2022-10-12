# MARRtino Blockly program #

MARRtino Blockly programming.

## Install ##

1. Follow installation instructions in MARRtino Python program.

2. Install blockly and websocket libraries.

* Blockly

    Blockly code is installed as a git submodule. Install with

        git submodule init
        git submodule update

    

* Websocket 

    Install tornado websocket library  

        sudo -H easy_install tornado


## Blocky programming ##


1. Run a robot

    Simulator

        cd marrtino_apps/stage
        roslaunch simrobot.launch


    Real robot

        cd marrtino_apps/robot
        roslaunch robot.launch 

    

2. Run the websocket server (on the robot machine)

        cd marrtino_apps/blockly
        python websocket_robot.py

    

3. Run the Blockly Robot app (on the client machine)

    **Option 1**. If you are on a Linux machine with marrtino_apps installed, just execute these commands:

        cd marrtino_apps/blockly
        firefox blockly_robot.html

    **Opyion 2**. If you set up a web server on the machine connected to the robot, you can use any browser on any machine (including tablet, smartphone, etc.) and connect to the IP of the robot with the URL 
        http://<IP_robot>/

    _Note_: to set up a web server follow instructions 'Web server update for Raspberry' in the Software page of MARRtino web site.


4. Use the Blockly Robot app

    Set IP of the robot (i.e., IP of the machine running the websocket server) and connect to it.

    Use Blockly to build a program and run it through the ```Run``` button.

    Use the ```Stop``` button at any time to stop the program and the robot.

    Use the ```Export/Import block code``` buttons to save and load blockly code in XML format. Copy and paste xml code on files to store them permanently.



## Develop new blocks ##

https://blockly-demo.appspot.com/static/demos/blockfactory/index.html

----

## Tile command programming ##

Same as Blocky programming, but using the HTML file
```tile_robot.html``` instead of  ```blockly_robot.html```.



## Python programming ##

Same as Blocky programming, but using the HTML file
```python_robot.html``` instead of  ```blockly_robot.html```.


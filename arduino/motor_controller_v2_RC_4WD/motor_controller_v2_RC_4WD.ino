/*
 Controller per 2560 ROVER EXPLORER




*/
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor 

const int PIN_SIDE_LIGHT_LED = 46;                  //Side light blinking led pin

unsigned long lastMilli = 0;
unsigned long lastMilli2 = 0;
// Start control with rc
// ==================================
#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 2000
#define TRC_MIN 1000
#define RC_DEADBAND 50
#define ERROR_center 50
#define pERROR 100  
#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 50
#define MODE_RUN 0
// Assign your channel in pins
#define THROTTLE_IN_PIN 18
#define STEERING_IN_PIN 19

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

const byte interruptPinThrottle = 18;
const byte interruptPinSteering = 19;
// 
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
// 
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

volatile uint8_t bUpdateFlagsShared;

unsigned long pulse_time  ;
// Eof controllo with rc
// ======================================
//--- Robot-specific constants ---
const double radius = 0.04;                   //Wheel radius, in m
const double wheelbase = 0.187;               //Wheelbase, in m
const double encoder_cpr = 990;                //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// PID Parameters
const double PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
Adafruit_DCMotor *leftMotor2 = AFMS.getMotor(3);      //Create left motor object
Adafruit_DCMotor *rightMotor2 = AFMS.getMotor(4);     //Create right motor object  
ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

const int lightIncNumber = 30;                                                                                                                                       //Number of lightIncrements for side light blinking
int lightInc = 0;                                                                                                                                                    //Init increment for side light blinking
int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightT = 0; //init light period

//__________________________________________________________________________

void setup() {
  pinMode(interruptPinThrottle, INPUT_PULLUP);
  pinMode(interruptPinSteering, INPUT_PULLUP);
   
  attachInterrupt(digitalPinToInterrupt(interruptPinThrottle),calcThrottle,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinSteering),calcSteering,CHANGE);
  
  pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      //set pin for side light leds as output
  analogWrite(PIN_SIDE_LIGHT_LED, 255);     //light up side lights
  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
 
  AFMS.begin();
  
  //setting motor speeds to zero
  leftMotor->setSpeed(0);
  leftMotor->run(BRAKE);
  leftMotor2->setSpeed(0);
  leftMotor2->run(BRAKE);
  rightMotor->setSpeed(0);
  rightMotor->run(BRAKE); 
  rightMotor2->setSpeed(0);
  rightMotor2->run(BRAKE);
 
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  // RC Control
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint8_t bUpdateFlags;
  static uint16_t throttleLeft;
  static uint16_t throttleRight;
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    pulse_time =millis() ;
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    bUpdateFlagsShared = 0;

    interrupts(); 
  } 
  // 
  // Controllo  MOTORI DX e SX   
  //
  
  if ((bUpdateFlags & THROTTLE_FLAG) || (bUpdateFlags & STEERING_FLAG)) {
      throttleLeft = map(unThrottleIn,unThrottleMin,unThrottleMax,PWM_MIN,PWM_MAX);
	    throttleRight = map(unSteeringIn,unSteeringMin,unSteeringMax,PWM_MIN,PWM_MAX);
      PWM_leftMotor = throttleLeft;
      PWM_rightMotor = throttleRight;
	    gDirection = DIRECTION_FORWARD;
  
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
      leftMotor2->setSpeed(0);
      leftMotor2->run(BRAKE);
      rightMotor2->setSpeed(0);
      rightMotor2->run(BRAKE);
      // Running
	    leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(FORWARD);
      leftMotor2->setSpeed(abs(PWM_leftMotor));
      leftMotor2->run(FORWARD);
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(BACKWARD); 
	    rightMotor2->setSpeed(abs(PWM_rightMotor));
      rightMotor2->run(BACKWARD);
   
    } else { 

    leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
      leftMotor2->setSpeed(0);
      leftMotor2->run(BRAKE);
      rightMotor2->setSpeed(0);
      rightMotor2->run(BRAKE);

  }
 

  // Se non riceve nessun comando RC

  if (bUpdateFlags == 0)  {  
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    
    if (!nh.connected()){
      analogWrite(PIN_SIDE_LIGHT_LED, lightValueNoComm[lightInc]);
      lightInc=lightInc+1;
      if (lightInc >= 25){
        lightInc=0;
      }
    }
    else{
      analogWrite(PIN_SIDE_LIGHT_LED, lightValue [lightInc]);
      lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
      lightInc=lightInc+(30/(lightT/LOOPTIME));
      if (lightInc >= lightIncNumber){
        lightInc=0;
      }
    }
    
    
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
    }
    else if (speed_req_left == 0){                        //Stopping
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
    }
    else if (PWM_leftMotor > 0){                          //Going forward
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(BACKWARD);
    }
    else {                                               //Going backward
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(FORWARD);
    }
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (speed_req_right == 0){                       //Stopping
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(FORWARD);
    }
    else {                                                //Going backward
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(BACKWARD);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
   } // DIRECTION_STOP
   // Clear
    bUpdateFlags = 0;
 }

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;

  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
   }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}


#include <Arduino.h>

#include "Motor.h"

/* Include definition of serial commands */
#include "commands.h"
/* Sensor functions */

#define MAX_SPEED 3.3 // in m/s

#define BAUDRATE 115200

// Target RPM for X axis motor
#define MOTOR_X_RPM 200
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 200

#define L 13.5 //Distance between the 2 wheels
#define MINIMUM_SPEED 2 //minimum speed in cm/s 
#define MAXIMUM_SPEED 20 //minimum speed in cm/s
// #define MINIMUM_SPEED 30 //minimum speed in cm/s 
// #define MAXIMUM_SPEED 100 //minimum speed in cm/s
#define SENSOR_BATTERY A1
#define LED_DONE_BOOT 13  
// X motor
#define DIR_X 5
#define STEP_X 2

// Y motor
#define DIR_Y 7
#define STEP_Y 4

#define EN_M 8
// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 32
#define MOTOR_STEP 6400

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually

long timerBothMotors=0;
bool showTimer = 0;
static StepperDriver motorR(DIR_X,STEP_X,EN_M,MOTOR_STEP);
static StepperDriver motorL(DIR_Y,STEP_Y,EN_M,MOTOR_STEP);

float vLin;
float vAng;
float speedR =0;
float speedL =0;
/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
unsigned long timer;
/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println("OK");
    break;
  case BATTERY_STATS:
    // float sensorValue = analogRead(SENSOR_BATTERY);
  //   double Battery = map(sensorValue, 453, 514, 0, 100);  // 100% battery
  //   Serial.println(Battery);
    Serial.println(analogRead(SENSOR_BATTERY)); 
    break;
  case SERVO_WRITE:
  //   servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
  //   Serial.println(servos[arg1].getServo().read());
    break;
  case READ_ENCODERS:
  //   Serial.print(readEncoder(LEFT));
  //   Serial.print(" ");
  //   Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
  //   resetEncoders();
  //   resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    if(fabs(atof(argv1))<MINIMUM_SPEED)
    { if(atof(argv1)<0)
      speedR=-MINIMUM_SPEED;
      else
      speedR=MINIMUM_SPEED;
    }
    else if(fabs(atof(argv1))>MAXIMUM_SPEED)
    {
      if(atof(argv1)<0)
      speedR=-MAXIMUM_SPEED;
      else
      speedR=MAXIMUM_SPEED;      
    }
    else
    speedR=atof(argv1);


    if(fabs(atof(argv2))<MINIMUM_SPEED)
    {if(atof(argv2)<0)
      speedL=-MINIMUM_SPEED;
      else
      speedL=MINIMUM_SPEED;
    }
    else if(fabs(atof(argv2))>MAXIMUM_SPEED)
    {
      if(atof(argv2)<0)
      speedL=-MAXIMUM_SPEED;
      else
      speedL=MAXIMUM_SPEED;
    }
    else
    speedL=atof(argv2);
    // motorR.doneMove = 0;
    // motorL.doneMove = 0;
    // showTimer = 1;
    // timerBothMotors = micros();
    // Serial.print("speedR:  ");Serial.print(speedR);Serial.print(" ,speedL:  ");Serial.println(speedL);
    Serial.println("OK"); 
    break;
  case DIRECTION:
    analogWrite(arg1, arg2);
    if(arg1 == 1)
    motorR.setDirection(StepperDriver::Direction::CCW);
    else
    motorR.setDirection(StepperDriver::Direction::CW);
    if(arg2 == 1)
    motorL.setDirection(StepperDriver::Direction::CCW);
    else
    motorL.setDirection(StepperDriver::Direction::CW);
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    // Serial.println(micros()-timer);
    // timer = micros(); 
    Serial.println("OK");
    break;
  case UPDATE_PID:
  //   while ((str = strtok_r(p, ":", &p)) != '\0') {
  //      pid_args[i] = atoi(str);
  //      i++;
  //   }
  //   Kp = pid_args[0];
  //   Kd = pid_args[1];
  //   Ki = pid_args[2];
  //   Ko = pid_args[3];
  //   Serial.println("OK");
    break;
  case VELOCITY:
    vLin=atof(argv1);
    vAng=atof(argv2);
    speedR=vLin+(vAng * L/2);
    speedL=vLin-(vAng * L/2);
    if(fabs(speedR)<MINIMUM_SPEED)
    { if(speedR<0)
      speedR=-MINIMUM_SPEED;
      else
      speedR=MINIMUM_SPEED;
    }
    else if(fabs(speedR)>MAXIMUM_SPEED)
    {
      if(speedR<0)
      speedR=-MAXIMUM_SPEED;
      else
      speedR=MAXIMUM_SPEED;      
    }
 
    if(fabs(speedL)<MINIMUM_SPEED)
    {if(speedL<0)
      speedL=-MINIMUM_SPEED;
      else
      speedL=MINIMUM_SPEED;
    }
    else if(fabs(speedL)>MAXIMUM_SPEED)
    {
      if(speedL<0)
      speedL=-MAXIMUM_SPEED;
      else
      speedL=MAXIMUM_SPEED;
    }
    // motorR.doneMove = 0;
    // motorL.doneMove = 0;
    // showTimer = 1;
    // timerBothMotors = micros();
    // Serial.print("speedR:  ");Serial.print(speedR);Serial.print(" ,speedL:  ");Serial.println(speedL);
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

//tes communication
// int x; 
// void setup() { 
// 	Serial.begin(115200); 
// 	Serial.setTimeout(1); 
// } 
// void loop() { 
// 	while (!Serial.available()); 
// 	x = Serial.readString().toInt(); 
// 	Serial.print(x + 1); 
// } 

void setup() {
  /*
    * Set target motors RPM.
    */
  Serial.begin(BAUDRATE);
  motorR.begin();
  motorL.begin();
  motorR.setDirection(StepperDriver::Direction::CCW);
  motorL.setDirection(StepperDriver::Direction::CW);
  pinMode(LED_DONE_BOOT,OUTPUT);
  digitalWrite(LED_DONE_BOOT,LOW);
  // timer = micros();
  // for (int i =0 ; i <2*200 ; i++){
  //   digitalWrite(STEP_X,HIGH);
  //   delayMicroseconds(1);
  //   digitalWrite(STEP_X,LOW);
  //   delayMicroseconds(1000);
  // }
  // Serial.print("time Motor R in us= ");Serial.println(micros()-timer); 
  // Serial.println("done move");
  // timer = micros();
  // for (int i =0 ; i <32*200 ; i++){
  //   digitalWrite(STEP_Y,HIGH);
  //   delayMicroseconds(1);
  //   digitalWrite(STEP_Y,LOW);
  //   delayMicroseconds(750);
    
  // }
  // Serial.print("time Motor L in us= ");Serial.println(micros()-timer); 
  // Serial.println("done move");  
  //  timer = micros();
  // for (int i =0 ; i <32*200 ; i++){
  //   digitalWrite(STEP_X,HIGH);digitalWrite(STEP_Y,HIGH);
  //   delayMicroseconds(1);
  //   digitalWrite(STEP_X,LOW);digitalWrite(STEP_Y,LOW);
  //   delayMicroseconds(500);
  // }
  // Serial.print("time Motor RL in us= ");Serial.println(micros()-timer); 
  // Serial.println("done move");  
    
}

void loop() {
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
    
  }
  if(speedR >= 0 )
    {motorR.setDirection(StepperDriver::Direction::CCW);}
  else
    {motorR.setDirection(StepperDriver::Direction::CW);}
  if(speedL >= 0 )
    {motorL.setDirection(StepperDriver::Direction::CW);}
  else
    {motorL.setDirection(StepperDriver::Direction::CCW);}
  
  motorR.execute(&speedR);
  motorL.execute(&speedL);
  //  if(!motorL.doneMove && !motorR.doneMove&& showTimer){
  // Serial.print("delay motor R (ms): ");Serial.println(micros()-motorR.prev_micros);
  // }
  // if((motorL.doneMove || motorR.doneMove)&& showTimer){
  // Serial.print("Time (us): ");Serial.println(micros()-timerBothMotors);
  // timerBothMotors = micros();
  // timerBothMotors=0;
  // showTimer=0;
  // }// speedR=0;
  // speedL=0;
}

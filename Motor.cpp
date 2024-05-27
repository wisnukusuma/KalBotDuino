#include "Motor.h"



 void StepperDriver::begin(){
    prev_micros= micros();
    stepDelay = 0;
    curr_micros = 0;
    step = 0;
    wheel_circumference = 21.5;
    accStep = 0;
    
}


void StepperDriver::enable(){

    digitalWrite(en_pin,LOW);
    delayMicroseconds(2);
}

void StepperDriver::disable(){

    digitalWrite(en_pin,HIGH);
    delayMicroseconds(2);
}

void StepperDriver::setDirection(Direction dir){
if(dir==CW){
    digitalWrite(dir_pin,HIGH);
    }
if(dir==CCW)
    {
    digitalWrite(dir_pin,LOW);
    }
}

void StepperDriver::execute(float *vel){
    float velocity = fabs(*vel);
    numStep = (velocity) * float(motor_step/wheel_circumference);
    stepDelay= (1/numStep)*500000L;
    curr_micros=micros();
    if(curr_micros - prev_micros >= stepDelay && stepDelay !=0 ){ ////If the difference in time is greater then said step delay then send a step signal for Right
      if(step < numStep)
      {
        move();
        prev_micros = curr_micros;
        step++;
        if(*vel < 0)
        accStep++;
        else
        accStep--;
      }
      else if(step>=numStep){
        // Serial.println(stepDelay);
        numStep=0;
        step=0;
        // *vel = 0;
      }
              
    }
}
void StepperDriver::move(){
      digitalWrite(step_pin,HIGH);
      delayMicroseconds(1);
      digitalWrite(step_pin,LOW);
}
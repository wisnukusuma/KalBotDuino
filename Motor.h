#ifndef STEPPER_DRIVER_BASE_H
#define STEPPER_DRIVER_BASE_H

#include <Arduino.h>

class StepperDriver {
public:
    /*
     * Basic connection: DIR, STEP are connected.
     */
    short dir_pin;
    short step_pin;
    short en_pin;
    int motor_step;
    float wheel_circumference; //get by measure the circumference of the wheel 
    enum Direction {CW, CCW};
    StepperDriver(short dir, short step, short en, int mot_step)
    {
    dir_pin = dir;
    step_pin = step;
    en_pin = en;
    motor_step = mot_step;
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin,HIGH);
    digitalWrite(en_pin,LOW);
    }
    void begin(void);
    void enable(void);
    void disable(void);
    void setDirection(Direction dir);
    void move(void);
    void execute(float *vel);
protected:
    unsigned int step;
    float numStep;
    unsigned int prev_micros;
    unsigned int curr_micros;
    double stepDelay;
    
};
#endif
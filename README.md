This code is used for differential drive robots (2-wheel motor propulsion) using **STEPPER MOTOR**. The main functionality provided is to receive Robot speed from **Twist Message ROS** requests over a serial connection, The original code has provisions for other features - e.g. read/write of digital/analog pins, servo control, but I've never used them.

The main commands to know are

m <motor_right_velocity> <motor_left_velocity> - Motor speed in cm/s
v <velocity_linear> <velocity_angular> - Set the linear velocity and the angular velocity of the robot cm/s and rad/s respectively

                          -----------          ________
                          |         |                  \
                          -----------                   \  
              ----------------------------------         v Angular Velocity (rad/s)
              |                                 |               
              |                                 |    ----------------> Linear Velocity (cm/s)
              |                                 |
              -----------------------------------
                          ------------
                          |          |
                          ------------

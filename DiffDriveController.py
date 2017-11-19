#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega,kp,ka,kb):
        # TODO for
        # Student: Specify these parameters
        self.kp = kp
        self.ka = ka
        self.kb = kb
        print('diffDriveCon')

        #check stability requirements
        if self.kp <= 0 or self.kb > 0 or self.ka < self.kb:
            print('system will be unstable based on ka kb and ka')
            sys.exit()


        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        delX = state[0] - goal[0]
        delY = state[1] - goal[1]
        rho = np.sqrt(delX**2 + delY**2)
        velCalc   = rho * self.kp
        done = rho < 0.1


        alpha = -state[2] + np.arctan2(delY,delX)
        beta  = -state[2] - alpha
        
        omegaCalc = self.ka * alpha + self.kb * beta

        if not done:
            velocity = min([velCalc,self.MAX_SPEED])
            omega    = (omegaCalc / abs(omegaCalc)) * min([abs(omegaCalc),abs(self.MAX_OMEGA)])
        else:
            velocity = 0
            omega = 0
        if False:
            print('rho: ' + str(round(rho,2)) + 
                ' X: ' + str(round(state[0],2)) + 
                ' Y: ' + str(round(state[1],2)) +
                ' Theta: ' + str(round(state[2],2)) +
                ' vel: ' +  str(round(velocity,3)) + 
                ' omega: ' +  str(round(omega,3)) +
                ' done: ' + str(done))
        
        return(velocity,omega,done,rho,state[0],state[1])
        

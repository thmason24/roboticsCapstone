#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""

import yaml
import numpy as np
import DiffDriveController as ddc

import sys

onRobot = False
# TODO for student: Comment this section when running on the robot 
if not onRobot:
    from RobotSim import RobotSim
    import matplotlib.pyplot as plt

# TODO for student: uncomment when changing to the robot
if onRobot:
    #from ros_interface import ROSInterface
    import RosInterface
    import rospy

# TODO for student: User files, uncomment as completed
#from MyShortestPath import dijkstras
#from KalmanFilter import KalmanFilter
#from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """

        # TODO for student: Comment this when running on the robot 
        if onRobot:
            self.ros_interface = RosInterface.ROSInterface(t_cam_to_body)
        else:
            self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                      max_speed, max_omega, x_spacing, y_spacing)

        #initialize differential drive controller
        maxSpeed = 0.5
        maxOmega = 0.5
        self.controller = ddc.DiffDriveController(maxSpeed, maxOmega)
        self.velocity = 0
        self.omega    = 0
        self.done = False
        self.missedTagCount = 0



        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        #self.kalman_filter = KalmanFilter(world_map)
        #self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """
        # TODO for student: Comment this when running on the robot 
        if onRobot:
            meas = self.ros_interface.get_measurements()
            imu_meas = self.ros_interface.get_imu()
        else:
            meas = self.robot_sim.get_measurements()
            imu_meas = self.robot_sim.get_imu()

        #compute velocity
        #get measurements for tag 3
        goal = [0.2,-0.1]
        tagFound = False
        if not meas == None:
            for i in meas:
                if i[3] == 3:
                    (self.velocity,self.omega,self.done) = self.controller.compute_vel(i,goal)
                    tagFound = True
                    #print('velocity: ' + str(self.velocity))
                    #print('omega   : ' + str(self.omega))
        if tagFound:
            self.missedTagCount = 0
        else:

            self.missedTagCount += 1

        if self.missedTagCount > 10:
            self.velocity = 0
            self.omega = 0

        #print(self.velocity)
        if onRobot:
            self.ros_interface.command_velocity(0.3, 0.8)
        else:
            self.robot_sim.command_velocity(self.velocity,self.omega)
        return
    
def main(args):
    # Load parameters from yaml
    if onRobot:
        param_path = '/home/pi/catkin_ws/src/robot_control/src/params.yaml'
    else:
        param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # TODO for student: Comment this when running on the robot 
    # Run the simulation
    if not onRobot:
        while not robotControl.robot_sim.done and plt.get_fignums():
            robotControl.process_measurements()
            robotControl.robot_sim.update_frame()

        plt.ioff()
        plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    if onRobot:
        rospy.init_node('node', anonymous=True)
        r = rospy.Rate(60)
        time = rospy.get_time()
        while not rospy.is_shutdown() and time + 10 > rospy.get_time():
            robotControl.process_measurements()
            r.sleep()
        # Done, stop robot
        robotControl.ros_interface.command_velocity(0.3,0)

if __name__ == "__main__":
    main(sys.argv)



#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import Header, Float32, String, Bool
from tf2_ros import TransformBroadcaster

class MovimentPlanningNode(DTROS):

    def __init__(self, node_name):

    # Initialize the DTROS parent class
        super(MovimentPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

   
    # Intialize required parameters
        self.dist = 0
        self.move_dist = 0
        self.starting_distance = 0
        self.starting_angle = 0
        self.distance_reached = 0
        self.angle_reached = 0
        self.angle = 0
        self.move_num = 0

        self.set_start_dist = True
        self.set_start_angle = True
        self.run_led = 0
        self.goal_reached = [True, False, False]
        
        # vars to track movment
        self.state = 1
        self.stop = False
        self.running = True
        self.cmd_start_dist = 0
        self.cmd_start_angle = 0
        self.cmd_start = 0        # prob dont need this one, it just tracks time
        self.vel = [0, 0]

        ############################################# CHANGE THIS IF ON A DIFFERENT DUCKIEBOT
        # self.forward_vel = [0.5, 0.57]
        # self.left = [-0.4, 0.4]
        # self.right = [0.4, -0.4]
        ############################################# THIS IS SET FOR csc22904
        self.forward_vel = [0.525, 0.5]
        self.left = [-0.48, 0.45]
        self.right = [0.48, -0.45]

        led_emitter = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        rospy.wait_for_service(led_emitter)
        self.change_pattern = rospy.ServiceProxy(led_emitter, ChangePattern)

    #Subscriber
        robot_pose = "/%s" % os.environ['VEHICLE_NAME'] + "/wheel_odometry/pose"
        self.sub_location = rospy.Subscriber(robot_pose, Pose2DStamped, self.cb_location_data, queue_size=1)

 
    # Publisher
        wheels_cmd = "/%s" % os.environ['VEHICLE_NAME'] + "/wheels_driver_node/wheels_cmd"
        self.pub_wheel_command = rospy.Publisher(wheels_cmd, WheelsCmdStamped, queue_size=1)

        self.exit_pub = rospy.Publisher("/exit", Bool, queue_size=1)


        self.log("Initialized")

    #Call back to retrive data from location publisher/subscriber
    def cb_location_data(self, data):
        self.dist = data.x
        self.angle = data.theta

        self.distance_reached = self.dist - self.starting_distance
        self.angle_reached = self.angle - self.starting_angle
    
    def LED_emitor_client(self, pattern_name):
        color = String()
        color.data = pattern_name
        self.change_pattern(color)
      
    # publish the velocity to a topic
    def set_velocity(self):

        msg = WheelsCmdStamped()
        msg.vel_left = self.vel[0]
        msg.vel_right = self.vel[1]
        self.pub_wheel_command.publish(msg)
        # stop the main loop if we are done
        if self.stop:
            self.running = False
            rospy.signal_shutdown("ROBO MOVED")
    
    # lets other nodes know to shutdown
    def publish_exit(self):
        msg  = Bool()
        msg.data = self.stop
        self.exit_pub.publish(msg)
        
    # check if the bot has come close to moving a dist
    def is_near(self, dist):
        # threshold
        epsilon = 0.3
        print(f"NEAR: {self.dist}, GOAL: {dist}, DIFF: {abs(self.dist - dist)}")
        # check if the bot is near or has pasted a dist
        if abs(self.dist - dist) < epsilon or self.dist > dist:
            print("#################CHANGING DIR")
            return True
        return False

    # check if the bot has come close to an angle
    def angle_is_near(self, theta):
        # threshold
        epsilon = 0.3
        rospy.loginfo(f"theta: {self.angle}, GOAL: {theta}, DIFF: {abs(self.angle - theta)}")
        # check if the bot is near an angle
        if abs(self.angle - theta) < epsilon :#or abs(self.angle) > abs(theta):
            print("#################CHANGING SPEED")
            return True
        return False

    # checks if stopping conditions are met for a cmd
    def run_cmd(self, cmd, state):
        print(f"diff dist: {cmd['dist'] + self.cmd_start_dist}, diff angle: {cmd['angle'] + self.cmd_start_angle}")
        diff_angle = True
        diff_dist = True

        if cmd["angle"] != 0:
            diff_angle = self.angle_is_near(cmd["angle"] + self.cmd_start_angle)
        if cmd["dist"] != 0:
            diff_dist = self.is_near(cmd["dist"] + self.cmd_start_dist)
        return diff_dist and diff_angle and self.state == state

    # wait before each cmd for 1 second
    def pause(self):
        self.vel = [0, 0]
        self.set_velocity()
        rospy.sleep(1)

               
    def run(self):

        # robo will execute all these commands in order until termination           read these descriptions
        cmd = [
            # once these conditions are met |  do this  
            {"dist": 0, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},        # wait 5 secs
            {"dist": 0, "angle": 0, "vel": self.right, "col": "BLUE"},              # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "BLUE"}, # go straight 
            {"dist": 1.25, "angle": 0, "vel": self.left, "col": "BLUE"},            # turn left
            {"dist": 0, "angle": np.pi / 2, "vel": self.forward_vel, "col": "BLUE"},# go straight
            {"dist": 1.25, "angle": 0,  "vel": self.left, "col": "BLUE"},           # turn left
            {"dist": 0, "angle": np.pi/2, "vel": self.forward_vel, "col": "BLUE"},  # go straight
            {"dist": 1.25, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},     # wait 5 secs
            {"dist": 0, "angle": 0,  "vel": self.left, "col": "GREEN"},             # turn left
            {"dist": 0, "angle": np.pi, "vel": self.forward_vel, "col": "GREEN"},   # go straight
            {"dist": 1.25, "angle": 0, "vel": self.right, "col": "GREEN"},          # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "GREEN"},# go straight
            {"dist": 1.25, "angle": 0, "vel": self.right, "col": "GREEN"},          # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "GREEN"},# go straight
            {"dist": 1.25, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},     # wait 5 secs
            {"dist": 0, "angle": 0, "vel": self.left, "col": "GREEN"},              # spin 180
            {"dist": 0, "angle": np.pi, "vel": self.forward_vel, "col": "WHITE"},   # prep for circle
            {"dist": 0.45, "angle": 0, "vel": [0, 0], "col": "WHITE", "wait": 0},  # prep for circle
            "circle",                                                               # go in a crcle 
            "end",                                                                  # terminate cleanly
            {"dist": 0, "angle": 0, "vel": [0, 0], "col": "LIGHT_OFF", "wait": 0},  # alt terminate code (cant be used w/ circle)
        ]

        curr_cmd = cmd[self.state-1]
        print(self.angle_is_near(2*np.pi + self.cmd_start_angle), not self.is_near(self.cmd_start_dist))    

        # runs cicle cmd and cirlce exit (ones that are stings)
        if curr_cmd == "circle":
            print("CIRCLE_START", self.dist)
            ############################################# CHANGE THIS IF ON A DIFFERENT DUCKIEBOT
            self.vel = [0.3, 0.6]
            ############################################# THIS IS SET FOR csc22904
            self.cmd_start_dist = self.dist
            self.cmd_start_angle = self.angle
            self.state += 1
            self.LED_emitor_client("WHITE")
            print(curr_cmd)

        # runs cicle cmd and cirlce exit (ones that are stings)
        elif curr_cmd == "end" and self.angle_is_near(2*np.pi + self.cmd_start_angle) and not self.dist == self.cmd_start_dist + 0.2:
            print("CIRCEL_END", self.dist)
            print(curr_cmd)
            self.vel = [0, 0]
            self.LED_emitor_client("LIGHT_OFF")
            self.set_velocity()
            self.stop = True

        # this runs all other cmds (ones that are dicts)
        elif type(curr_cmd) != str and self.run_cmd(curr_cmd, self.state) and not self.stop:
            # pause between cmds
            self.pause()
            # set the init vals for each cmd
            self.cmd_start_dist = self.dist
            self.cmd_start_angle = self.angle
            self.cmd_start = rospy.Time.now().to_sec()
            # update which cmd we are on
            self.state += 1
            print(curr_cmd)
            # publish the velocity and LED col
            self.vel = curr_cmd["vel"]
            self.LED_emitor_client(curr_cmd["col"])
            self.set_velocity()
            # check if we are waiting then sleep 
            if self.vel == [0, 0]:
                rospy.sleep(curr_cmd["wait"])
            print(f"EXECUTING STATE: {self.state-1}")
            # terminate once all cmds are run
            if self.state > len(cmd):
                self.stop = True

if __name__ == '__main__':
    # create the node
    node = MovimentPlanningNode(node_name='moviment_planning_node')
    while not rospy.is_shutdown() and node.running:
        node.run()
        node.publish_exit()
        node.set_velocity()
        rospy.sleep(0.1)


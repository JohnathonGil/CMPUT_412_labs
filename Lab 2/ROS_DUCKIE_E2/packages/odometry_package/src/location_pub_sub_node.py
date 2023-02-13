#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped
from std_msgs.msg import Float32, String, Bool

class LocationNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LocationNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.veh_name = rospy.get_namespace().strip("/")

        #This is to keep the records of the previous encoder values
        self.starting_ticks_l = 0
        self.starting_ticks_r = 0
        self.previous_tick_count_set = [False, False]
        self.current_left_wheel_ticks = 0
        self.current_right_wheel_ticks = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.initial_robot_frame = [0, 0, 0]
        self.world_frame = [0, 0, 0]
        self.rotation = 0.0
        self.dist = 0.0

        self.running = True

        #Need constants for axel length and wheel radius
        self._baseline = 0.05
        self._radius = 0.0318

        # These variables are record the time taken between each segment
        self.starting_time = rospy.Time.now().to_sec()
        self.left_ticks_time = 0
        self.right_ticks_time = 0
        self.encoder_time = 0
        self.total_time_recorded = 0

        # Subscribing to the wheel encoders
        left_wheel_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/left_wheel_encoder_node/tick"
        self.sub_encoder_ticks_left = rospy.Subscriber(left_wheel_topic, WheelEncoderStamped, self.cb_encoder_data, callback_args="left", queue_size=1)
        right_wheel_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/right_wheel_encoder_node/tick"
        self.sub_encoder_ticks_right = rospy.Subscriber(right_wheel_topic, WheelEncoderStamped, self.cb_encoder_data, callback_args="right", queue_size=1)

        self.sub_encoder_ticks_right = rospy.Subscriber("/exit", Bool, self.cb_exit, queue_size=1)

        #Publishing the odometry information
        odometry_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/wheel_odometry/pose"
        self.pub_odometry = rospy.Publisher(odometry_topic, Pose2DStamped, queue_size=1)
        self.log("Initialized")
        
        #self.bag = rosbag.Bag('/data/world_frame.bag', 'w')


    # Call back function to obtain encoder data and encoder time stanp
    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """
        # When the callback argument is left, obtain encoder message from left wheel
        if wheel == "left":
            self.current_left_wheel_ticks = msg.data
            self.left_ticks_time = msg.header.stamp.to_sec()
            # Obtain the intial registered tick value from the encoder
            if not self.previous_tick_count_set[0]:
                self.starting_ticks_l = self.current_left_wheel_ticks
                self.previous_tick_count_set[0] = True
            # Set the actual left tick count by subtacting the intial count by the measured count
            else:
                self.left_ticks = self.current_left_wheel_ticks-self.starting_ticks_l

        # When the callback argument is right, obtain encoder message from right wheel
        elif wheel == "right":
            self.current_right_wheel_ticks = msg.data
            self.right_ticks_time = msg.header.stamp.to_sec()
            # Obtain the intial registered tick value from the encoder
            if not self.previous_tick_count_set[1]:
                self.starting_ticks_r = self.current_right_wheel_ticks
                self.previous_tick_count_set[1] = True
            # Set the actual right tick count by subtacting the intial count by the measured count
            else:
                self.right_ticks = self.current_right_wheel_ticks-self.starting_ticks_r

        # Standardize the time stamps from both encoders into a single value
        # Also obtain the time since the intialization of this node
        self.encoder_time = (self.left_ticks_time + self.left_ticks_time)/2
        self.total_time_recorded = self.encoder_time - self.starting_time
        self.total_time_recorded = self.r_value(self.total_time_recorded)
        self.new_robot_frame(self.left_ticks, self.right_ticks)

    def cb_exit(self, msg):
        if msg.data:
            rospy.loginfo("Total execution time: " + str(self.total_time_recorded))
            rospy.loginfo("The World Frame of final location: " + str(self.world_frame))
            self.running = False
            rospy.signal_shutdown("ROBO MOVED")

    # Set the robot frame based on the data obtained by the encoders
    def new_robot_frame (self, left_ticks, right_ticks):

        # Set the distance equivalent to the Xr of our robot frame
        distance_left = (2*np.pi*self._radius*left_ticks)/135
        distance_right = (2*np.pi*self._radius*right_ticks)/135
        self.dist = ((distance_left)+(distance_right))/2
        self.dist = self.r_value(self.dist)

        # Set the angle value equivalent to the theta of our robot frame
        self.rotation = ((distance_right-distance_left)/(2*self._baseline))
        self.rotation = self.r_value(self.rotation)
        
        self.initial_robot_frame[0] += self.dist* np.cos(self.rotation% (2*np.pi))
        self.initial_robot_frame[1] += self.dist * np.sin(self.rotation % (2*np.pi))
        self.initial_robot_frame[2] += self.rotation % (2*np.pi)

        self.world_frame[0] = self.initial_robot_frame[1] + 0.32
        self.world_frame[1] = self.initial_robot_frame[0] + 0.32
        self.world_frame[2] = self.initial_robot_frame[2] + (np.pi/2)
        # rospy.loginfo("Distance: " + str(self.dist))
        # rospy.loginfo("The  arc distance: " + str(self.rotation))
        # rospy.loginfo("The time: " + str(self.total_time_recorded))

        self.publish_info()

        # Runs write ros bag every two seconds
        # if np.round(self.total_time_recorded) % 2 == 0:
        #     self.write_to_rosbag()
 
    # This function is to round up our float values by two decimals
    def r_value(self, value):

        r_val = np.round(value, decimals=2)
        return r_val

    # Publish our odometry to our topic
    def publish_info(self):
        odometry_msg = Pose2DStamped()
        odometry_msg.x = self.dist
        odometry_msg.y = 0
        odometry_msg.theta = self.rotation
        self.pub_odometry.publish(odometry_msg)

    #------------------------ Ros Bag Code ---------------------------------------- 
    # def write_to_rosbag (self):

    #     wf_msg_x = Float32()
    #     wf_msg_x.data = self.world_frame[0]
    #     wf_msg_y = Float32()
    #     wf_msg_y.data = self.world_frame[1]
    #     wf_msg_theta = Float32()
    #     wf_msg_theta.data = self.world_frame[2]
    #     self.bag.write('wf_x', wf_msg_x)
    #     self.bag.write('wf_y', wf_msg_y)
    #     self.bag.write('wf_y', wf_msg_theta)


if __name__ == '__main__':
    node = LocationNode(node_name='my_location_node')
    # Keep it spinning to keep the node alive
    #node.publish_info()
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
    # if rospy.is_shutdown():
    #     print("------- Closing ROSBAG ---------")
    #     node.bag.close()
      
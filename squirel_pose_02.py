#!/usr/bin/env python3
# license removed for brevity
# rostopic echo /mavros/vision_pose/pose
import numpy as np
import rospy
from mavros_msgs.msg import State, Altitude, HomePosition, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamGet, StreamRateRequest, StreamRate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

# from trajectory_utility import *
# /vive/LHR_6C62CAC0_pose /mavros/vision_pose/pose


class PosePublisher:
    def __init__(self):
        rospy.init_node('vive_to_mavros', anonymous=True)
        self.rate = rospy.Rate(30)  # 10hz
        self.current_pose_name = '/mavros/local_position/pose'
        self.topic_name = '/mavros/setpoint_position/local'
        self.pose_publisher = rospy.Publisher(self.topic_name, PoseStamped, queue_size=10)
        self.start_position = PoseStamped()

        self.extended_state = ExtendedState()
        self.mode = ''
        self.current_yaw = 0

        # Setup services to set commands to drone
        self.service_timeout = 30

        rospy.wait_for_service('/mavros/cmd/arming', self.service_timeout)
        rospy.wait_for_service('/mavros/set_mode', self.service_timeout)
        rospy.wait_for_service('/mavros/cmd/takeoff', self.service_timeout)
        rospy.wait_for_service('/mavros/cmd/land', self.service_timeout)

        rospy.loginfo('Services are connected and ready')

        self.set_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)

        self.state = State()
        self.mode = ''
        self.waypoint_frame = 'map'

        # Frame of target position
        self.start_position.header.frame_id = self.waypoint_frame

        # Starting pose and orientation, orientation is a quaternion
        self.start_position.pose.position.x = 0.0
        self.start_position.pose.position.y = 0.0
        self.start_position.pose.position.z = 0.2
        self.start_position.pose.orientation.x = 0
        self.start_position.pose.orientation.y = 0
        self.start_position.pose.orientation.z = 0
        self.start_position.pose.orientation.w = 1
        self.waypoints = [self.start_position]

        # Waypoints are copied, make individual waypoints
        # set up initial vales for x,y,z
        x_wp = -0.2
        y_wp = 0.2
        z_wp = 0.95
        y_step = 0.2
        y_increment = 0.2
        z_step = 0.15
        z_increment = 0.15

        wp_a = PoseStamped()
        wp_a.header.frame_id = self.waypoint_frame

        wp_a.pose.position.x = x_wp
        wp_a.pose.position.y = y_wp
        wp_a.pose.position.z = z_wp
        wp_a.pose.orientation.x = 0
        wp_a.pose.orientation.y = 0
        wp_a.pose.orientation.z = 0
        wp_a.pose.orientation.w = 1
        self.waypoints.append(wp_a)

        # go through a list of y coordinates with corresponding yaws
        # p manipulate x_wp, z_wp as a list of points in a loop, incrementing x and z

        y_ls = ["0.0", "0,3", "-0.3"]
        rot_ls = ["0.0", "7.0", "-7.0"]

        for i, y_wp in enumerate(y_ls):
            z_rot = (rot_ls[i])
            q_rot = quaternion_from_euler(0, 0, z_rot)

            # Reset Z to restart the spiral
            z_wp = 1.15

            # loop describing a spiral path
            for wp_no in range(0, 3):
                # FIRST LEG
                y_wp = y_wp + y_step

                wp_a = PoseStamped()
                wp_a.header.frame_id = self.waypoint_frame
                wp_a.pose.position.x = x_wp
                wp_a.pose.position.y = y_wp
                wp_a.pose.position.z = z_wp
                wp_a.pose.orientation.x = q_rot[0]
                wp_a.pose.orientation.y = q_rot[1]
                wp_a.pose.orientation.z = q_rot[2]
                wp_a.pose.orientation.w = q_rot[3]
                self.waypoints.append(wp_a)

                # SECOND LEG
                z_wp = z_wp + z_step

                wp_a = PoseStamped()
                wp_a.header.frame_id = self.waypoint_frame
                wp_a.pose.position.x = x_wp
                wp_a.pose.position.y = y_wp
                wp_a.pose.position.z = z_wp
                wp_a.pose.orientation.x = q_rot[0]
                wp_a.pose.orientation.y = q_rot[1]
                wp_a.pose.orientation.z = q_rot[2]
                wp_a.pose.orientation.w = q_rot[3]
                self.waypoints.append(wp_a)

                # increase step and move back across and dow
                y_step = y_step + y_increment
                z_step = z_step + z_increment

                # THIRD LEG
                y_wp = y_wp - y_step

                wp_a = PoseStamped()
                wp_a.header.frame_id = self.waypoint_frame
                wp_a.pose.position.x = x_wp
                wp_a.pose.position.y = y_wp
                wp_a.pose.position.z = z_wp
                wp_a.pose.orientation.x = 0
                wp_a.pose.orientation.y = 0
                wp_a.pose.orientation.z = 0
                wp_a.pose.orientation.w = 1
                self.waypoints.append(wp_a)

                # FOURTH LEG
                z_wp = z_wp - z_step

                wp_a = PoseStamped()
                wp_a.header.frame_id = self.waypoint_frame
                wp_a.pose.position.x = x_wp
                wp_a.pose.position.y = y_wp
                wp_a.pose.position.z = z_wp
                wp_a.pose.orientation.x = 0
                wp_a.pose.orientation.y = 0
                wp_a.pose.orientation.z = 0
                wp_a.pose.orientation.w = 1
                self.waypoints.append(wp_a)

                # increase step for next loop
                y_step = y_step + y_increment
                z_step = z_step + z_increment

            # Finish where we started
            self.waypoints.append(self.start_position)

        self.current_wp = 0
        self.arrive_tol = 0.1

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def set_mode(self, mode):
        if self.state.mode != mode:
            try:
                mode_change_response = self.set_mode_srv(base_mode=0, custom_mode=mode)
                last_request_time = rospy.get_rostime()
                print (last_request_time)
                if not mode_change_response.mode_sent:
                    rospy.logerr('---Mode change failed---')
            except rospy.ServiceException as exception:
                rospy.logerr('Failed to change mode')

    def arm(self):
        last_request_time = rospy.get_rostime()
        print(last_request_time)
        if not self.state.armed:
            arm_response = self.set_arm_srv(True)
            if arm_response:
                rospy.loginfo('---Vehicle armed --')
            else:
                rospy.loginfo('---Arming failed ---')
            last_request_time = rospy.get_rostime()
            print(last_request_time)
        else:
            # vehicle is already armed
            pass

    def disarm(self):
        if self.set_arm_srv(False):
            rospy.loginfo('--Vehicle disarmed---')
        else:
            rospy.loginfo('---Disarming failed')

    def start_offboard(self):
        # wait to get heartbeat from fcu
        while not self.state.connected:
            self.rate.sleep()

        rospy.loginfo('--Got heartbeat from FCU----')

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def callback(self, msg):
        distance_to_next_point = 0
        current_position = msg.pose.position
        target_position = self.waypoints[self.current_wp].pose.position

        # vector subtraction between targe and current
        distance_vector = [target_position.x - current_position.x, target_position.y - current_position.y,
                           target_position.z - current_position.z]
        distance = np.linalg.norm(distance_vector)

        if distance < self.arrive_tol:
            # keep track of how long you've been herre using rospy.Time
            # if iteration is longer than wait time, then move on
            self.current_wp += 1

        # print(f"{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")

    def run(self):
        # rospy.init_node('vive_to_mavros', anonymous=True)
        # self.pose_publisher = rospy.Publisher(self.repeat_name, PoseStamped, queue_size=10)
        rospy.Subscriber(self.current_pose_name, PoseStamped, self.callback)
        rospy.loginfo('Pose publisher has started.')

        self.start_offboard()

        count = 0
        while not rospy.is_shutdown():
            count = count + 1

            if self.current_wp == len(self.waypoints):
                print("This is the final waypoint.")
            else:
                msg = self.waypoints[self.current_wp]
                msg.header.seq = count
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'

                self.pose_publisher.publish(msg)

            if count == 50:
                self.arm()
                self.set_mode("OFFBOARD")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        PosePublisher().run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from tf_transformations import euler_from_quaternion
import casadi as ca
import numpy as np
from auav_pylon_2025.tecs_xtrack_sample import TECSControl
from auav_pylon_2025.cross_tracker import *


def wrap(x):
    return (x % 1) - 1


dt = 0.01
alt = 5#3  # Desired Cruising altitude (m)

## SIM Coordinate Waypoints
x0 = 3.2
y0 = 0.0
theta0 = 0
xf = 10.0
yf = -20.0
control_point = [
    (x0, y0, 0),
    (35.0, 0.0, alt),
    (45.0, 0.0, alt),
    (45.0, 25.0, alt),
    (-8.0, 25.0, alt),
    (-10.0, 1.0, alt),
    (5, -15.0, 2.0),
    (xf, yf, 0.1),
]

## PURT Circuit
x0 = -3#1.6
y0 = 3.2#2.54
# theta0 = 2.97
# xf = 0.0
# yf = 10.0
xf = -3#1.6
yf = 3.2#2.54
#

control_point = [(x0, y0, alt), (-14.0, 4.00,alt), (-13.5, 16.0,alt), (-1.5, 16.5,alt),(-1.5, 5.0,alt), (xf, yf,alt)] #Square circuit, const altitude

# Get coordinates for reference line
ref_x_list = [point[0] for point in control_point]
ref_y_list = [point[1] for point in control_point]
ref_z_list = [point[2] for point in control_point]


class PIDPublisher(Node):
    def __init__(self):
        super().__init__("night_vapor_publisher")

        # Set up Parameter
        self.declare_parameter("mocap_vehicle_id", "/nightvapor1")
        self.declare_parameter("frame_id", "/map")  # default to map

        # self.pub_control_input = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_joy = self.create_publisher(Joy, "/auto_joy", 10)
        # self.sub_mocap = self.create_subscription(Odometry, self.get_parameter('mocap_vehicle_id').get_parameter_value().string_value+'/odom', self.pose_cb, 10)
        self.sub_mocap = self.create_subscription(
            Odometry, "/odom", self.pose_cb, 10
        )  # temporary with no namespace
        # self.pub_ref_point = self.create_publisher(PoseStamped, '/ref_point', 10)
        self.pub_ref_path = self.create_publisher(Path, "/ref_path", 10)
        self.pub_path = self.create_publisher(Path, "/path_real", 10)
        self.pub_ref_val = self.create_publisher(Float32MultiArray, "ref_values", 10)
        self.pub_actual_val = self.create_publisher(
            Float32MultiArray, "actual_values", 10
        )
        self.timer_path = self.create_timer(1, self.publish_ref_path)
        self.timer = self.create_timer(0.01, self.pub_night_vapor)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.time = 0
        self.dt = 0.01
        self.tecs_control = TECSControl(self.dt, 3)  # 1 denotes nv1, 3 denotes sim
        self.current_WP_ind = 0  # Current Waypoint Index
        self.last_WP_ind = 1  # Last Waypoint Index
        self.wpt_planner = XTrack_NAV(self.dt, control_point, self.current_WP_ind)
        self.wpt_planner.path_distance_buf = 2.0
        self.wpt_planner.wpt_switching_distance = 4.0
        self.flight_mode = "takeoff"
        self.pub_flight_mode = self.create_publisher(String, "flight_mode", 10)
        self.takeoff_time = 0.0
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []
        self.throttle = 0.7
        self.rudder = 0.0
        self.elev = 0.0  # elevator command (negative=elev_down --> pitches up)
        self.ail_roll = 0.0  # roll
        self.trail_size = 1000
        self.x_est = None
        self.prev_x = None
        self.x_est_last = None
        self.y_est = None
        self.prev_y = None
        self.y_est_last = None
        self.z_est = None
        self.prev_z = None
        self.prev_roll = None
        self.prev_pitch = None
        self.prev_yaw = None
        self.prev_v = None
        self.roll_est = None
        self.roll_est_last = None
        self.pitch_est = None
        self.pitch_est_last = None
        self.v_est = None
        self.v_est_last = None
        self.p_est = None
        self.p_est_last = None
        self.r_est = None  # Yaw rate
        self.r_est_last = None
        self.z_est = None  # Altitude filtering
        self.z_est_last = None
        self.yaw_est = None  # Heading filtering
        self.yaw_est_last = None
        self.vx_est = None
        self.vx_est_last = None
        self.vy_est = None
        self.vy_est_last = None
        self.vz_est = None
        self.vz_est_last = None
        self.gamma_est = None
        self.gamma_est_last = None
        self.vdot_est = None
        self.vdot_est_last = None
        self.end_cruise = False  # flight tag for end of cruise
        self.des_a = None
        self.prev_des_a = None
        self.prev_des_v = None
        self.ref_data = {
            "des_v": 0.0,
            "des_gamma": 0.0,
            "des_heading": 0.0,
            "des_a": 0.0,
        }
        self.actual_data = {
            "x_est": 0.0,
            "y_est": 0.0,
            "z_est": 0.0,
            "roll_est": 0.0,
            "pitch_est": 0.0,
            "yaw_est": 0.0,
            "vx_est": 0.0,
            "vy_est": 0.0,
            "vz_est": 0.0,
            "v_est": 0.0,
            "gamma_est": 0.0,
            "vdot_est": 0.0,
            "r_est": 0.0,
        }

    def pose_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

        # self.get_logger().info(
        #     "pose: x: %0.2f; y: %0.2f: z: %0.2f" % (self.x, self.y, self.z)
        # )

        if self.prev_x is None:
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_z = self.z
            self.prev_roll = self.roll
            self.prev_pitch = self.pitch
            self.prev_yaw = self.yaw
            self.prev_speed = np.sqrt(self.x**2 + self.y**2 + self.z**2)

        alpha = np.exp(-2 * np.pi * 10 * self.dt)  # exp(w*T)
        # # alpha = 0.001

        v_est_new = (
            np.sqrt(self.x**2 + self.y**2 + self.z**2)
            - np.sqrt(self.prev_x**2 + self.prev_y**2 + self.prev_z**2)
        ) / self.dt  # Velocity
        if self.v_est_last is None:
            self.v_est_last = v_est_new
        self.v_est = v_est_new * alpha + self.v_est_last * (1 - alpha)
        self.v_est_last = self.v_est

        x_est_new = self.x
        if self.x_est_last is None:
            self.x_est_last = x_est_new
        self.x_est = x_est_new * alpha + self.x_est_last * (1 - alpha)
        self.x_est_last = self.x_est

        y_est_new = self.y
        if self.y_est_last is None:
            self.y_est_last = y_est_new
        self.y_est = y_est_new * alpha + self.y_est_last * (1 - alpha)
        self.y_est_last = self.y_est

        z_est_new = self.z
        if self.z_est_last is None:
            self.z_est_last = z_est_new
        self.z_est = z_est_new * alpha + self.z_est_last * (1 - alpha)
        self.z_est_last = self.z_est

        x_est_new = self.x
        if self.x_est_last is None:
            self.x_est_last = x_est_new
        self.x_est = x_est_new * alpha + self.x_est_last * (1 - alpha)
        self.x_est_last = self.x_est

        # filter roll
        roll_est_new = self.roll
        if self.roll_est_last is None:
            self.roll_est_last = roll_est_new
        self.roll_est = roll_est_new * alpha + self.roll_est_last * (1 - alpha)
        self.roll_est_last = self.roll_est

        # filter Pitch
        pitch_est_new = self.pitch
        if self.pitch_est_last is None:
            self.pitch_est_last = pitch_est_new
        self.pitch_est = pitch_est_new * alpha + self.pitch_est_last * (1 - alpha)
        self.pitch_est_last = self.pitch_est

        # filter heading
        yaw_est_new = self.yaw
        if self.yaw_est_last is None:
            self.yaw_est_last = yaw_est_new
        self.yaw_est = yaw_est_new * alpha + self.yaw_est_last * (1 - alpha)
        self.yaw_est_last = self.yaw_est

        vx_est_new = (self.x - self.prev_x) / self.dt  # Vx
        if self.vx_est_last is None:
            self.vx_est_last = vx_est_new
        self.vx_est = vx_est_new * alpha + self.vx_est_last * (1 - alpha)
        self.vx_est_last = self.vx_est

        vy_est_new = (self.y - self.prev_y) / self.dt  # Vy
        if self.vy_est_last is None:
            self.vy_est_last = vy_est_new
        self.vy_est = vy_est_new * alpha + self.vy_est_last * (1 - alpha)
        self.vy_est_last = self.vy_est

        vz_est_new = (self.z - self.prev_z) / self.dt  # Vz
        if self.vz_est_last is None:
            self.vz_est_last = vz_est_new
        self.vz_est = vz_est_new * alpha + self.vz_est_last * (1 - alpha)
        self.vz_est_last = self.vz_est

        p_est_new = self.roll - self.prev_roll
        r_est_new = self.yaw - self.prev_yaw  # change in yaw
        if self.p_est_last is None:
            self.p_est_last = p_est_new
        if self.r_est_last is None:
            self.r_est_last = r_est_new
        self.p_est = p_est_new * alpha + self.p_est_last * (1 - alpha)
        self.r_est = r_est_new * alpha + self.r_est_last * (1 - alpha)
        self.p_est_last = self.p_est
        self.r_est_last = self.r_est

        # Flight Path Angle Estimated GAMMA
        # v_temp = (np.sqrt(self.x**2+self.y**2+self.z**2) - np.sqrt(self.prev_x**2+self.prev_y**2+self.prev_z**2))/self.dt
        v_temp = self.v_est
        if np.abs(v_temp) < 1e-5:
            v_temp = 1e-5  # avoid divide by zero error
        gamma_est_new = np.arcsin(
            np.clip(((self.z - self.prev_z) / self.dt) / np.abs(v_temp), -1, 1)
        )
        if self.gamma_est_last is None:
            self.gamma_est_last = gamma_est_new
        self.gamma_est = gamma_est_new * alpha + self.gamma_est_last * (1 - alpha)
        self.gamma_est_last = self.gamma_est

        # Acceleration
        # speed_temp = np.sqrt(self.x**2+self.y**2+self.z**2)
        vdot_est_new = v_temp - self.prev_speed  # /self.dt
        if self.vdot_est_last is None:
            self.vdot_est_last = vdot_est_new
        self.vdot_est = vdot_est_new * alpha + self.vdot_est_last * (1 - alpha)
        self.vdot_est_last = self.vdot_est

        self.actual_data = {
            "x_est": self.x_est,
            "y_est": self.y_est,
            "z_est": self.z_est,
            "roll_est": self.roll_est,
            "pitch_est": self.pitch_est,
            "yaw_est": self.yaw_est,
            "vx_est": self.vx_est,
            "vy_est": self.vy_est,
            "vz_est": self.vz_est,
            "v_est": self.v_est,
            "gamma_est": self.gamma_est,
            "vdot_est": self.vdot_est,
            "r_est": self.r_est,
        }

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
        self.prev_speed = self.v_est
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.prev_yaw = self.yaw

    def pub_night_vapor(self):
        self.last_WP_ind = np.shape(control_point)[0]  # determine last waypoint

        ######################################## FLIGHT MODE HANDLING ####################################
        flight_mode_msg = String()
        if (self.z <= 0.5) and self.end_cruise == False:
            new_mode = "takeoff"
        else:
            new_mode = "airborne"

        if new_mode != self.flight_mode:
            self.get_logger().info(
                "Flight mode changed from: %s to %s" % (self.flight_mode, new_mode)
            )
            self.flight_mode = new_mode
            flight_mode_msg.data = new_mode

        if flight_mode_msg.data == "":  # initialize flight mode
            flight_mode_msg.data = self.flight_mode

        self.pub_flight_mode.publish(flight_mode_msg)  # Publish Flight mode

        #################################################################################################

        self.time += self.dt

        if self.flight_mode == "takeoff":
            self.takeoff_time += self.dt
            self.throttle += 2 * self.dt
            if self.throttle < 0.7:
                self.throttle = 0.7
            self.throttle = ca.if_else(self.throttle > 1, 1.0, self.throttle)
            self.rudder = 0.0  # Rudder
            self.elev = 0.15
            self.ail_roll = 0.0  # Aileron

        # Enforce Looping in cruise
        if self.current_WP_ind == self.last_WP_ind: 
            self.current_WP_ind = 0 # go back to cruise altitude waypoint
            self.end_cruise = False
            self.wpt_planner = XTrack_NAV(self.dt, control_point, self.current_WP_ind)

            print("Continueing circuit...Returning to Waypoint %s" %(self.current_WP_ind))

        if self.flight_mode == "airborne":
            if (
                self.current_WP_ind == self.last_WP_ind
            ):  ## TODO Implement "land" waypoint mode
                self.end_cruise = False
                self.current_WP_ind = 0
                # self.rudder = 0.0
                # self.elev = 0.0
                # self.ail_roll = 0.0
                # self.throttle = 0.0
                print("ending Cruise")
            else:
                v_array = [self.vx_est, self.vy_est, self.vz_est]

                des_v, des_gamma, des_heading, along_track_err, cross_track_err = (
                    self.wpt_planner.wp_tracker(
                        control_point,
                        self.x_est,
                        self.y_est,
                        self.z_est,
                        v_array,
                        verbose=False,
                    )
                )

                ## Calculating Desired Acceleration based on desired velocity
                if self.prev_v is None:
                    self.prev_v = self.v_est
                K_V = 2.0
                self.des_a = K_V * (
                    des_v - np.abs(self.v_est)
                )  # /self.dt # Desired Acceleration from current velocity

                self.prev_des_a = self.des_a
                self.prev_des_v = des_v

                self.ref_data = {
                    "des_v": des_v,
                    "des_gamma": des_gamma,
                    "des_heading": des_heading,
                    "des_a": self.des_a,
                }

                self.throttle, self.rudder, self.elev, self.ail_roll = (
                    self.tecs_control.compute_control(
                        int(self.time / self.dt), self.ref_data, self.actual_data
                    )
                )
                self.current_WP_ind = self.wpt_planner.check_arrived(
                    along_track_err, verbose=False
                )

                self.get_logger().info(
                    "Current Waypoint: %0.0f" % (self.current_WP_ind)
                )

        # Publish Reference Data for Analysis
        ref_val_msg = Float32MultiArray()
        ref_val_msg.data = [
            self.ref_data["des_v"],
            self.ref_data["des_gamma"],
            self.ref_data["des_heading"],
            self.ref_data["des_a"],
        ]
        self.pub_ref_val.publish(ref_val_msg)

        # Publish Actual Filtered Data for Analysis
        actual_val_msg = Float32MultiArray()
        actual_val_msg.data = [
            self.actual_data["x_est"],
            self.actual_data["y_est"],
            self.actual_data["z_est"],
            self.actual_data["roll_est"],
            self.actual_data["pitch_est"],
            self.actual_data["yaw_est"],
            self.actual_data["vx_est"],
            self.actual_data["vy_est"],
            self.actual_data["vz_est"],
            self.actual_data["v_est"],
            self.actual_data["gamma_est"],
            self.actual_data["vdot_est"],
            self.actual_data["r_est"],
        ]
        self.pub_actual_val.publish(actual_val_msg)

        ###########################################################################################
        ########## SET CONTROL SIGNALS ###########

        # Set Channel Messages
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 5

        # Night Vapor 3 Channels
        # joy_msg.axes[0] = self.throttle # Throttle
        # joy_msg.axes[1] = delta # Rudder
        # joy_msg.axes[2] = elev # Elevator
        # joy_msg.axes[3] = 0
        # joy_msg.axes[4] = 1900 # Force onboard stabilizing

        # Simulator 3-Channel
        joy_msg.axes[0] = self.throttle # Throttle
        joy_msg.axes[1] = self.rudder + -1 * self.ail_roll #Rudder Input + "ail_roll" represents auto stabilizer
        joy_msg.axes[2] = self.elev #Elevator
        joy_msg.axes[3] = 0
        joy_msg.axes[4] = 1900  # Force onboard stabilizing

        self.pub_joy.publish(joy_msg)

        # Append current position to the list for display
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.z_list.append(self.z)
        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)

        # Publish the trajectory of the vehicle
        self.publish_path()
        # self.pub_path.publish

    def publish_ref_path(self):
        msg_path = Path()
        msg_path.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        msg_path.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in zip(ref_x_list, ref_y_list, ref_z_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = (
                self.get_parameter("frame_id").get_parameter_value().string_value
            )
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            msg_path.poses.append(pose)
        self.pub_ref_path.publish(msg_path)

    def publish_path(self):
        msg_path = Path()
        msg_path.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )  #'qualisys'
        msg_path.header.stamp = self.get_clock().now().to_msg()
        if len(self.x_list) > self.trail_size:
            del self.x_list[0]
            del self.y_list[0]
            del self.z_list[0]
            del self.yaw_list[0]
        for x, y, z, yaw in zip(self.x_list, self.y_list, self.z_list, self.yaw_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = (
                self.get_parameter("frame_id").get_parameter_value().string_value
            )  #'qualisys'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = np.cos(yaw / 2)
            pose.pose.orientation.z = np.sin(yaw / 2)
            msg_path.poses.append(pose)
        self.pub_path.publish(msg_path)


def main(args=None):
    rclpy.init(args=args)
    PID_publisher = PIDPublisher()
    rclpy.spin(PID_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PID_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

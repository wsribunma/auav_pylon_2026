#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion
import casadi as ca
import numpy as np

from auav_pylon_2026.tecs_controller_xtrack_sample import TECSControl_cub

from auav_pylon_2026.cross_tracker_nav_sample import *


def wrap(x):
    return (x % 1) - 1


dt = 0.01
alt = 4  # Desired Cruising altitude (m)

# ## SIM
alt = 7.0
control_point = [
    (-10, -5, alt),
    (-30.0, -10, alt),
    (-30, -40.0, alt),
    (30.00, -30.0, alt),
    (30, 5.0, alt),
    (10, 5, alt),
    (-10, -5, alt),
]  # Rectangle Circuit Full Facility, const altitude

# ## PURT Circuit in Real Facility
# control_point = [(3.88, -6.1, alt), (-4.0, -5,alt), (-3, 2.0, alt), (15.20, 2.0, alt),(15, -3.22, alt), (5.88, -6.1, alt)] #Rectangle Circuit Full Facility, const altitude


# Get coordinates for reference line
ref_x_list = [point[0] for point in control_point]
ref_y_list = [point[1] for point in control_point]
ref_z_list = [point[2] for point in control_point]


###### FILTER ######
def _lpf(self, attr: str, new_value, alpha: float):
    """
    Exponential low-pass update for a single signal.
    Creates/uses <attr>_est and <attr>_est_last attributes.
    Returns the filtered estimate.
    """
    if not (0.0 <= alpha <= 1.0):
        raise ValueError("alpha must be in [0, 1]")
    last_name = f"{attr}_est_last"
    est_name = f"{attr}_est"

    last = getattr(self, last_name, None)
    if last is None:
        last = new_value  # initialize on first call

    est = alpha * new_value + (1.0 - alpha) * last
    setattr(self, est_name, est)
    setattr(self, last_name, est)
    return est


def _lpf_many(self, mapping: dict, alpha: float):
    """
    Batch low-pass updates. `mapping` is {attr_name: new_value}.
    Each `attr_name` will use <name>_est / <name>_est_last.
    """
    for name, val in mapping.items():
        self._lpf(name, val, alpha)


##################


class PIDPublisher(Node):
    def __init__(self):
        super().__init__("sports_cub_publisher")

        # Set up Parameter
        self.declare_parameter("mocap_vehicle_id", "/sim")
        self.declare_parameter("frame_id", "/map")

        # self.pub_control_input = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pub_joy = self.create_publisher(Joy, '/auto_joy', 10)
        self.pub_joy = self.create_publisher(
            Joy,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/auto_joy",
            10,
        )
        self.sub_mocap = self.create_subscription(
            Odometry,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/odom",
            self.pose_cb,
            10,
        )
        self.pub_ref_path = self.create_publisher(
            Path,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/ref_path",
            10,
        )
        self.pub_path = self.create_publisher(
            Path,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/path_real",
            10,
        )
        self.pub_ref_val = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/ref_values",
            10,
        )
        self.pub_actual_val = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("mocap_vehicle_id").get_parameter_value().string_value
            + "/actual_values",
            10,
        )
        self.timer_path = self.create_timer(1, self.publish_ref_path)
        self.timer = self.create_timer(0.01, self.pub_sports_cub)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.time = 0
        self.dt = 0.01
        self.tecs_control = TECSControl_cub(self.dt, "sim")
        self.current_WP_ind = 0  # Starting WP index
        self.last_WP_ind = 1  # Last Waypoint Index, this gets overwritten later
        self.wpt_planner = XTrack_NAV_lookAhead(
            self.dt, control_point, self.current_WP_ind
        )
        self.wpt_planner.path_distance_buf = 5.0  # 2.0
        self.wpt_planner.wpt_switching_distance = 1.0  # 4.0
        self.wpt_planner.v_cruise = 10.0  # 0.5
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
        self.aileron = 0.0  # roll
        self.trail_size = 1000
        self.x_est = None
        self.prev_x = None
        # self.x_est_last = None
        self.y_est = None
        self.prev_y = None
        # self.y_est_last = None
        self.z_est = None
        self.prev_z = None
        self.prev_roll = None
        self.prev_pitch = None
        self.prev_yaw = None
        self.prev_v = None
        self.roll_est = None
        self.pitch_est = None
        self.v_est = None
        self.p_est = None
        self.q_est = None
        self.r_est = None  # Yaw rate
        self.z_est = None  # Altitude filtering
        self.yaw_est = None  # Heading filtering
        self.vx_est = None
        self.vy_est = None
        self.vy_est_last = None
        self.vz_est = None
        self.gamma_est = None
        self.gamma_est_last = None
        self.vdot_est = None
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
            "p_est": 0.0,
            "q_est": 0.0,
            "r_est": 0.0,
        }
        self.prev_t = None

    @staticmethod
    def _angdiff(a, b):
        return (a - b + np.pi) % (2 * np.pi) - np.pi

    def _lpf(self, name: str, new_value, alpha: float):
        """
        Exponential low-pass update for <name>.
        Uses/creates attributes: <name>_est and <name>_est_last.
        """
        if not (0.0 <= alpha <= 1.0):
            raise ValueError("alpha must be in [0, 1]")
        last_name = f"{name}_est_last"
        est_name = f"{name}_est"

        last = getattr(self, last_name, None)
        if last is None:
            last = new_value  # initialize on first call

        est = alpha * new_value + (1.0 - alpha) * last
        setattr(self, est_name, est)
        setattr(self, last_name, est)
        return est

    def _lpf_many(self, mapping: dict, alpha: float):
        """Batch low-pass updates: mapping = {name: new_value}."""
        for k, v in mapping.items():
            self._lpf(k, v, alpha)

    def reload_gains_callback(self, request, response):
        # Trigger to call to reload param and gains
        # ros2 service call /reload_gains std_srvs/srv/Trigger
        try:
            self.controller.reload_gains()
            response.success = True
            response.message = "Gains reloaded successfully."
        except Exception as e:
            self.get_logger().error(f"Failed to reload gains: {e}")
            response.success = False
            response.message = str(e)
        return response

    def pose_cb(self, msg: Odometry):
        # Get time step
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.prev_t is None:
            self.prev_t = t - 0.01
        dt = t - self.prev_t
        if dt <= 0:
            self.prev_t = t - 0.01
        elif dt == 0:
            dt = 0.01
        self.dt = max(dt, 0.01) # prevent dt = 0
        self.prev_t = t

        # Unpack raw data from topic
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Initialize on first time step
        if self.prev_x is None:
            self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z
            self.prev_roll, self.prev_pitch, self.prev_yaw = (
                self.roll,
                self.pitch,
                self.yaw,
            )
            self.prev_speed = np.sqrt(self.x**2 + self.y**2 + self.z**2)

        fc = 10.0  # Hz
        alpha = np.exp(-2 * np.pi * fc * self.dt)

        # Finite differences
        vx_new = (self.x - self.prev_x) / self.dt
        vy_new = (self.y - self.prev_y) / self.dt
        vz_new = (self.z - self.prev_z) / self.dt
        speed_new = np.sqrt(vx_new**2 + vy_new**2 + vz_new**2)

        # Angular rates from successive Euler angles (wrap yaw)
        p_new = self._angdiff(self.roll, self.prev_roll) / self.dt
        q_new = self._angdiff(self.pitch, self.prev_pitch) / self.dt
        r_new = self._angdiff(self.yaw, self.prev_yaw) / self.dt

        eps = 1e-5
        denom = max(speed_new, eps)
        gamma_new = np.arcsin(
            np.clip(vz_new / denom, -1.0, 1.0)
        )  # Flight path angle (gamma): asin(vz / |v|)

        vdot_new = (
            speed_new - self.prev_speed
        )  # / self.dt # Acceleration magnitutde diff

        # Low-pass filer
        self._lpf_many(
            {
                "x": self.x,
                "y": self.y,
                "z": self.z,
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "vx": vx_new,
                "vy": vy_new,
                "vz": vz_new,
                "v": speed_new,
                "gamma": gamma_new,
                "vdot": vdot_new,
                "p": p_new,
                "q": q_new,
                "r": r_new,
            },
            alpha,
        )

        self.actual_data = {
            "x_est": self.x_est,
            "y_est": self.y_est,
            "z_est": self.z_est,  # Position
            "roll_est": self.roll_est,
            "pitch_est": self.pitch_est,
            "yaw_est": self.yaw_est,  # Euler Orientation
            "vx_est": self.vx_est,
            "vy_est": self.vy_est,
            "vz_est": self.vz_est,  # Velocity
            "v_est": self.v_est,
            "gamma_est": self.gamma_est,
            "vdot_est": self.vdot_est,
            "p_est": self.p_est,
            "q_est": self.q_est,
            "r_est": self.r_est,  # Angular Rates
        }

        # Update history
        self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z
        self.prev_roll, self.prev_pitch, self.prev_yaw = self.roll, self.pitch, self.yaw
        self.prev_speed = self.v_est

    def pub_sports_cub(self):
        self.last_WP_ind = np.shape(control_point)[0]  # determine last waypoint

        ######################################## FLIGHT MODE ####################################
        flight_mode_msg = String()
        if (self.z <= 1.0) and self.end_cruise == False:
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

        ###########################################################################################

        self.time += self.dt

        if self.flight_mode == "takeoff":
            self.takeoff_time += self.dt

            # Throttle ramp with floor/ceiling
            self.throttle = ca.fmin(1.0, ca.fmax(0.7, self.throttle + 2.0 * self.dt))

            self.rudder = 0.0  # No yaw during takeoff
            self.aileron = 0.0  # Wings-level during takeoff

            # Elevator schedule (taildragger hold-down, then smooth pitch-up)
            v_to = 0.5  # takeoff speed threshold
            e_down = -0.02  # elevator up while accelerating (tail on ground)
            e_up = 0.15  # target pitch-up elevator
            e_rate = 0.40  # max elevator change per second
            if self.v_est == None:
                self.v_est = 0.0  # Initialize V_est, assume start at stationary

            self.elev = ca.if_else(
                self.v_est < v_to, e_down, ca.fmin(e_up, self.elev + e_rate * self.dt)
            )

        # Enforce Looping in cruise
        if self.current_WP_ind == self.last_WP_ind:
            self.current_WP_ind = 0  # go back to cruise altitude waypoint
            self.end_cruise = False
            self.wpt_planner = XTrack_NAV_lookAhead(
                self.dt, control_point, self.current_WP_ind
            )

            print(
                "Continueing circuit...Returning to Waypoint %s" % (self.current_WP_ind)
            )

        if self.flight_mode == "airborne":

            if self.current_WP_ind == self.last_WP_ind:  # End Cruise
                self.current_WP_ind = 0  # go back to cruise altitude waypoint
                self.end_cruise = False
                self.wpt_planner = XTrack_NAV_lookAhead(
                    self.dt, control_point, self.current_WP_ind
                )
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
                K_V = 1.0  # 2.0
                self.des_a = K_V * (
                    des_v - np.abs(self.v_est)
                )  # Desired Acceleration from current velocity

                self.prev_des_a = self.des_a
                self.prev_des_v = des_v

                self.ref_data = {
                    "des_v": des_v,
                    "des_gamma": des_gamma,
                    "des_heading": des_heading,
                    "des_a": self.des_a,
                }

                self.aileron, self.elev, self.throttle, self.rudder = (
                    self.tecs_control.compute_control(
                        int(self.time / self.dt), self.ref_data, self.actual_data
                    )
                )
                # self.current_WP_ind = self.wpt_planner.check_arrived(self.x_est, self.y_est, self.z_est)
                self.current_WP_ind = self.wpt_planner.check_arrived(
                    along_track_err, v_array, verbose=False
                )

                self.get_logger().info(
                    "Control Command: Aileron: %0.2f: Elevator: %0.2f; Throttle: %0.2f Rudder: %0.2f"
                    % (self.aileron, self.elev, self.throttle, self.rudder)
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
            self.actual_data["p_est"],
            self.actual_data["q_est"],
            self.actual_data["r_est"],
        ]
        self.pub_actual_val.publish(actual_val_msg)

        ###########################################################################################
        ########## SET CONTROL SIGNALS ###########

        # Set Channel Messages
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 5

        # Cub Control PPM AETR
        joy_msg.axes[0] = self.aileron
        joy_msg.axes[1] = self.elev
        joy_msg.axes[2] = self.throttle
        joy_msg.axes[3] = self.rudder
        joy_msg.axes[4] = 2000  # Force onboard stabilizing

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
    PID_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

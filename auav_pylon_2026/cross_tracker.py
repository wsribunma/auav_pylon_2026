#!/usr/bin/env python3
import numpy as np

######################
# Waypoint Target Algorithm with cross-track flow field and Alongtrack switching modes
######################


def angle_rad_wrapper(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class XTrack_NAV:
    def __init__(self, dt, waypoints, start_WP_ind):
        self.dt = dt

        ## CURRENT_WP index might need to be parsed from the class variable in "init" and cycle directly from ros script
        self.current_WP_ind = start_WP_ind  # Current Active Next Waypoint Index
        self.next_wpt = None  # Current Active Next Waypoint Coordinate
        self.prev_wpt = (0, 0, 0)  # Previous Waypoint Coordinate
        self.last_WP = len(waypoints) - 1  # Last Waypoint Index
        self.current_pose_est = [0, 0, 0]  # Filtered pose position
        self.waypoints_list = waypoints

        self.v_max_vert = 0.5  # maximum vertical velocity (positive up) m/s
        self.v_max_horz = 0.55  # maximum horizontal velocity m/s
        self.v_min_horz = (
            0.5  # minimum horizontal velocity m/s enforce to prevent stall
        )
        self.v_cruise = 0.5  # cruise airspeed
        self.wpt_rad = 3.0  # allowable error from target waypoint (m)

        self.wpt_switching_distance = (
            4.0  # Look ahead for 3 m along track and jump to next waypoint
        )
        self.path_distance_buf = 2.0  # Cross-track distance buffer

    def get_desired_flight(
        self, next_wpt, current_pose, Vx_speed, Vy_speed, verbose=False
    ):
        x_err = next_wpt[0] - current_pose[0]
        y_err = next_wpt[1] - current_pose[1]
        z_err = next_wpt[2] - current_pose[2]

        horz_dist_err = np.sqrt(x_err**2 + y_err**2)  # Distance Error

        # Compute desired airspeed (velocity)
        des_v = self.v_cruise  # fix desired velocity to be desired cruise speed

        # Compute desired flight path angle
        K_h = 3.0  # gain on hdot --> higher = steeper gamma
        if horz_dist_err == 0:
            des_gamma = 0
        else:
            des_gamma = (
                K_h * z_err / horz_dist_err
            )  # Alternatively, this can be calculated from vertical-track error

        # # Compute along-track and cross-track error
        V_vector = np.array([Vx_speed, Vy_speed])
        V_speed_horz = np.linalg.norm(V_vector)
        x_est, y_est, _ = self.current_pose_est

        gamma_p = np.arctan2(
            self.next_wpt[1] - self.prev_wpt[1], self.next_wpt[0] - self.prev_wpt[0]
        )  # Path Tangential angle

        xdot_t = V_speed_horz * np.cos(gamma_p)  # path-tracking velocity x
        ydot_t = V_speed_horz * np.sin(gamma_p)  # path-tracking velocity y
        x_t = xdot_t * self.dt
        y_t = ydot_t * self.dt

        path_vect = np.array(self.next_wpt) - np.array(
            self.prev_wpt
        )  # Vector between two waypoints
        path_len = np.linalg.norm(path_vect)  # Distance between the two waypoints
        path_angle = np.arctan2(
            self.next_wpt[1] - self.prev_wpt[1], self.next_wpt[0] - self.prev_wpt[0]
        )
        unit_along_path = path_vect[:2] / path_len  # path unit vector
        unit_normal = (
            np.array([-path_vect[1], path_vect[0]]) / path_len
        )  # unit normal to the path

        pose_vect = [x_est, y_est] - np.array(self.prev_wpt)[:2]

        along_track_err_w0 = np.dot(
            pose_vect, unit_along_path
        )  # along track error from prev_wpt w0
        along_track_err_w1 = (
            path_len - along_track_err_w0
        )  # along track error based on w2
        cross_track_err = np.dot(pose_vect, unit_normal)

        ### Cross track heading
        des_heading = path_angle + np.arctan2(
            -1 * cross_track_err, self.path_distance_buf
        )  # Obtain vector field for heading angle tracking
        des_heading = (des_heading + np.pi) % (
            2 * np.pi
        ) - np.pi  # normalize heading to [-pi, pi]

        if verbose == True:
            print(
                f"x_t : {x_t:.2f}\
                    \ny_t : {y_t:.2f}\
                    \nAlong-Track Error from w0: {along_track_err_w0:.2f}\
                    \nAlong-Track Error from w1: {along_track_err_w1:.2f}\
                    \nCross-Track Error : {cross_track_err:.2f}\
                    \nPath Tangential Angle (Gamma_p): {gamma_p:0.2f}\
                    \nDesired Heading : {des_heading:0.2f}"
            )

        return des_v, des_gamma, des_heading, along_track_err_w1, cross_track_err

    def wp_tracker(self, waypoints, x_est, y_est, z_est, V_array, verbose=False):
        """
        Accepts desired waypoints, iterate through waypoints
        Waypoints: List and Waypoitn Types
        Valid Waypoint type: takeoff, nav, descent, land

        Accepts filtered positions (x,y,z)
        """
        # self.last_WP = len(waypoints) - 1# Last Waypoint index
        if self.next_wpt == None:
            self.next_wpt = waypoints[self.current_WP_ind]

        self.current_pose_est = [x_est, y_est, z_est]  # Filtered position

        self.next_wpt = waypoints[self.current_WP_ind]
        if self.current_WP_ind != 0:
            self.prev_wpt = waypoints[self.current_WP_ind - 1]

        vx, vy, _ = V_array

        # Compute Desired Speed, Desired Heading, Desired Glide Path
        des_v, des_gamma, des_heading, along_track_err, cross_track_err = (
            self.get_desired_flight(
                self.next_wpt, self.current_pose_est, vx, vy, verbose=False
            )
        )

        if verbose == True:
            print(
                f"Desired Velocity : {des_v:.2f}\
                   \nDesired Flight Path Angle (Gamma) : {des_gamma:.2f}\
                   \nAlong-Track Error : {along_track_err:.2f}\
                   \nCross-Track Error : {cross_track_err:.2f}\
                   \nDesired Heading : {des_heading:0.2f}\
                   \nCurrent Waypoint Index: {self.current_WP_ind:0.0f}\
                   "
            )
        return des_v, des_gamma, des_heading, along_track_err, cross_track_err

    def check_arrived(self, along_track_err, verbose=False):
        # Horizontal Distance Waypoint checker
        # Check based on along-track error
        if along_track_err < self.wpt_switching_distance:
            self.current_WP_ind += 1
            print(f"Waypoint reached, going to waypoint {self.current_WP_ind}...")

        if verbose == True:
            print(f"Updated Waypoint Index: {self.current_WP_ind}")

        return self.current_WP_ind

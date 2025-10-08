#!/usr/bin/env python3

# import numpy as np
import casadi as ca

"""
This module performs Total Energy control designed for Night Vapor trajectory tracking.
"""


class TECSControl:
    def __init__(self, dt, args) -> None:
        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0
        self.prev_V = 0
        self.prev_roll = 0
        self.prev_pitch = 0
        self.prev_yaw = 0
        self.prev_ref_yaw = 0
        self.dt = dt
        self.g = 9.81
        self.args = args  # Vehicle selection
        self.error_norm_Es_dot_integral = 0  # Integral of error of specific error rate normalized by velocity
        self.error_dist_term_integral = 0  # Integral of error of (V_E_dot/g - gamma_E) term from energy rate distribution adjustment
        self.error_pitch_integral = 0  # Integral of error of pitch
        self.error_thrust_integral = 0  # Integral of error of thrust
        self.error_r_integral = 0  # Integral of error of yaw rate
        self.error_r_last = 0  # last error of yaw rate
        self.error_roll_integral = 0  # integral of error of roll
        self.throttle_cmd = None  # throttle command
        self.error_xtrack_integral = 0  # Integral of error in side acceleration
        self.error_xtrack_last = 0  # Last value of error in side acceleration

    def compute_thrust_pitch(
        self, t, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est
    ):
        # ref data in function of time
        ref_airspeed = ref_data["des_v"]
        ref_gamma = ref_data["des_gamma"]  # Glide slope angle
        ref_accel = ref_data["des_a"]

        r_gamma = float(ref_gamma)  # desired flight path angle
        r_V_dot = float(ref_accel)  # desired acceleration

        # Night Vapor Sim
        if self.args == 3:
            # Thrust gains
            K_thrustp = 0.05
            K_thrusti = 0.17

            # Pitch gains
            K_pitchp = 0.05
            K_pitchi = 0.16

            # Integral error bounds
            norm_Es_dot_integral_max = 4.5
            dist_term_integral_max = 4.5

            # Set Trim
            # trim_thtl = 0.5 # Trim throttle condition
            # trim_elev = 0 # Trim Elevator
            trim_thrust = 3.0  # Thrust trim for cruise

            # Weight
            weight = 0.5 * self.g  # mass = 0.5

        #######################################################
        # Envelope Protection Feature
        thr_max = 4.5  # Maximum Thrust
        drag = 1.0  # Maximum Drag estimate TODO Quantify max drag
        # r_gamma = np.clip(r_gamma, -drag/weight, (thr_max-drag)/weight)
        r_V_dot = ca.fmax(
            ca.fmin(r_V_dot, (thr_max - drag) / weight), -drag / weight
        )  # Limit function for Casadi
        #######################################################

        # -------------------Desired Thrust-------------------#
        # Specific energy rate error
        error_norm_Es_dot = (r_gamma - gamma_est) + (r_V_dot - vdot_est) / self.g

        # Specific energy rate integral error
        self.error_norm_Es_dot_integral += error_norm_Es_dot * self.dt
        if self.error_norm_Es_dot_integral > norm_Es_dot_integral_max:
            self.error_norm_Es_dot_integral = norm_Es_dot_integral_max
        elif self.error_norm_Es_dot_integral < -norm_Es_dot_integral_max:
            self.error_norm_Es_dot_integral = -norm_Es_dot_integral_max

        # Desired thrust
        thrust = trim_thrust + weight * (
            K_thrustp * (gamma_est + vdot_est / self.g)
            + K_thrusti * self.error_norm_Es_dot_integral
        )
        # # Saturation for Thrust
        if thrust < 0:
            thrust = 0

        err_gamma = r_gamma - gamma_est

        # -------------------Desired Pitch-------------------#
        # Energy rate distribution term error
        error_dist_term = (r_gamma - gamma_est) - (r_V_dot - vdot_est) / self.g

        # Energy rate distribution term integral error
        self.error_dist_term_integral += error_dist_term * self.dt
        if self.error_dist_term_integral > dist_term_integral_max:
            self.error_dist_term_integral = dist_term_integral_max
        elif self.error_dist_term_integral < -dist_term_integral_max:
            self.error_dist_term_integral = -dist_term_integral_max

        # Desired pitch
        pitch = K_pitchi * self.error_dist_term_integral - K_pitchp * (
            gamma_est - vdot_est / self.g
        )

        return thrust, pitch

    def compute_control(
        self, t, ref_data, actual_data, ref_thrust=None, ref_pitch=None
    ):
        # actual data
        x = actual_data["x_est"]
        y = actual_data["y_est"]
        z = actual_data["z_est"]
        roll = actual_data["roll_est"]
        pitch = actual_data["pitch_est"]
        yaw = actual_data["yaw_est"]
        vx_est = actual_data["vx_est"]
        vy_est = actual_data["vy_est"]
        vz_est = actual_data["vz_est"]
        V_est = actual_data["v_est"]
        gamma_est = actual_data["gamma_est"]
        vdot_est = actual_data["vdot_est"]
        r_est = actual_data["r_est"]

        # Get desired thrust and pitch (we can remove this if we want to fully separate the two functions during implementation)
        if ref_thrust == None or ref_pitch == None:
            # print("calculating ref thrust and pitch")
            ref_thrust, ref_pitch = self.compute_thrust_pitch(
                t, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est
            )
        ref_heading = ref_data["des_heading"]
        r_heading = float(ref_heading)  # desired heading yaw rate

        # nvp sim
        if self.args == 3:
            # Throttle gains
            K_thtlp = 0.2
            K_thtli = 0.01

            # Elevator gains
            K_elevp = 0.3
            K_elevi = 0.1

            # Rudder gain XTrack
            K_deltap = 0.35
            K_deltai = 0.25
            K_deltad = 0.20

            # Pitch rate gain
            K_q = 1

            # Integral error bounds
            pitch_integral_max = 0.5
            thrust_integral_max = 1
            r_integral_max = 0.4
            xtrack_integral_max = 1.0

            # Set Trim
            trim_thtl = 0.2  # Trim throttle condition
            trim_elev = 0.2  # -0.1 # Trim Elevator
            trim_rud = 0  # Trim on Rudder

            # Weight
            weight = 0.5 * self.g  # mass = 0.5

        # -------------------Elevator Control-------------------#
        # Compute errors
        pitch = -1 * pitch  # Pitch appears to have reversed sign
        error_pitch = ref_pitch - pitch

        # Integral of pitch error
        self.error_pitch_integral += error_pitch * self.dt
        if self.error_pitch_integral > pitch_integral_max:
            self.error_pitch_integral = pitch_integral_max
        elif self.error_pitch_integral < -pitch_integral_max:
            self.error_pitch_integral = -pitch_integral_max

        # Control commands for elevator
        elev = trim_elev + (
            K_elevp * error_pitch + K_elevi * self.error_pitch_integral
        )  # + K_q * error_q

        elev = ca.fmax(ca.fmin(elev, 1), -1)  # Limit function for Casadi

        # -------------------Throttle Control-------------------#
        # This can be modified to account for dynamic thrust to throttle ratio
        thr_max = 4.5  # Maximum Thrust
        self.throttle_cmd = ca.fmax(
            ca.fmin(ref_thrust / thr_max, 1), 0
        )  # Limit function for Casadi

        # #-------------------Heading Control using WP_NAV-------------------#
        # Compute yaw
        err_yaw = r_heading - yaw  # error of haeding angle
        err_yaw = (err_yaw + ca.pi) % (2 * ca.pi) - ca.pi  # wrapper

        print(f"r_heading: {r_heading:0.2f}, yaw: {yaw:0.2f}, err_yaw: {err_yaw:0.2f}")

        error_r_deriv = (err_yaw - self.error_r_last) / self.dt
        self.error_r_last = err_yaw
        self.error_r_integral += err_yaw * self.dt
        if self.error_r_integral > r_integral_max:
            self.error_r_integral = r_integral_max
        elif self.error_r_integral < -r_integral_max:
            self.error_r_integral = -r_integral_max

        rud = (
            trim_rud
            + K_deltap * (err_yaw)
            + K_deltai * self.error_r_integral
            + K_deltad * error_r_deriv
        )
        rud = ca.fmax(ca.fmin(rud, 1), -1)

        # -------------------Roll Control-------------------#
        # This is only used for sim
        error_roll = roll - 0  # reference roll of 0 for level
        roll_integral_max = 0.2

        # Gains
        K_rollp = 0.8
        K_rolli = 0.8

        # error_r_deriv = (error_r - self.error_r_last)/self.dt
        self.error_roll_last = error_roll
        self.error_roll_integral += error_roll * self.dt
        if self.error_roll_integral > roll_integral_max:
            self.error_roll_integral = roll_integral_max
        elif self.error_roll_integral < -roll_integral_max:
            self.error_roll_integral = -roll_integral_max

        ail = (
            K_rollp * (error_roll) + K_rolli * self.error_roll_integral
        )  # + K_deltad * error_r_deriv
        ail = -1 * ca.fmax(ca.fmin(ail, 1), -1)

        # Set history variables
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_pitch = pitch
        self.prev_yaw = yaw
        # self.prev_ref_yaw = r_heading

        return self.throttle_cmd, rud, elev, ail

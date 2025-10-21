import numpy as np
import yaml
from pathlib import Path
from types import SimpleNamespace


"""
This module performs Total Energy control designed for Sport Cub trajectory tracking.
"""

def _wrap_pi(a):
    return np.arctan2(np.sin(a), np.cos(a))

def _safe_div(x, y, eps=1e-6):
    return x / (y if abs(y) > eps else np.sign(y)*eps if y != 0.0 else eps)


class TECSControl_cub:
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
        self.thr_max = 7.5 # 4.5 #Maximum Thrust

        self.args = args #Vehicle selection
        this_file = Path(__file__).resolve()
        self.base_dir = this_file.parent / "param"

        self.error_norm_Es_dot_integral = 0 #Integral of error of specific error rate normalized by velocity
        self.error_dist_term_integral = 0 #Integral of error of (V_E_dot/g - gamma_E) term from energy rate distribution adjustment
        self.error_pitch_integral = 0 #Integral of error of pitch
        self.error_thrust_integral = 0 #Integral of error of thrust
        self.error_r_integral = 0 #Integral of error of yaw rate
        self.error_r_last = 0 #Integral of error of yaw rate
        self.error_roll_integral =0 #integral of error of roll
        self.throttle_cmd = None # throttle command
        self.error_xtrack_integral = 0 # Integral of error in side acceleration
        self.error_xtrack_last = 0 # Last value of error in side acceleration

        self.roll_mode = "stabilized"   # choices: "stabilized" | "phi_stick" | "direct"
        self._phi_cmd = 0.0         # last commanded bank
        self._e_phi_int = 0.0       # roll PID integrator

        # self._last_chi_err = 0.0

        self.reload_gains()

    def reload_gains(self):
        print(f"[TECSControl] Using gain path: {self.base_dir / f"{self.args}.yaml"}")

        gain_path = self.base_dir / f"{self.args}.yaml"

        if not gain_path.exists():
            raise FileNotFoundError(f"[TECSControl] Gain file not found: {gain_path}")

        with open(gain_path, "r") as f:
            raw = yaml.safe_load(f)

        self.param = SimpleNamespace(**raw)

        self.phi_lim       = np.deg2rad(self.param.phi_lim_deg)
        self.chi_deadband  = np.deg2rad(self.param.chi_deadband_deg)
        self.phi_dot_lim   = np.deg2rad(self.param.phi_dot_lim_deg_s)
        # self.k_chi_rate    = np.deg2rad(self.param.phi_dot_per_rad_deg_s)


        self.mass = self.param.mass
        self.weight = self.mass * self.g
        
        print(f"[TECSControl] Gains reloaded from: {gain_path}")

    def compute_thrust_pitch(self, t, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est):
        # ref data in function of time
        ref_airspeed = ref_data['des_v']
        ref_gamma = ref_data['des_gamma'] #Glide slope angle
        # ref_xtrack_err = ref_data['xtrack_err']
        ref_accel = ref_data['des_a']

        r_V = float(ref_airspeed)  # desired body-frame speed
        r_gamma = float(ref_gamma) # desired flight path angle
        r_V_dot = float(ref_accel) # desired acceleration


        #######################################################
        # Envelope Protection Feature
        drag = 1.0 # Maximum Drag estimate TODO -- can be improved to be a function of velocity
        # r_gamma = np.clip(r_gamma, -drag/weight, (thr_max-drag)/weight)
        r_V_dot = np.clip(r_V_dot, -drag/self.weight, (self.thr_max-drag)/self.weight)
        # print(f"desired_gamma: {r_gamma:5.2f}, gamma_est: {gamma_est:5.2f} Vdot_ref:{r_V_dot/self.g:5.2f} Vdot_actual:{vdot_est/self.g:5.2f} err_vdot:{(r_V_dot-vdot_est)/self.g:5.2f}")

        #######################################################

        #-------------------Desired Thrust-------------------#
        #Specific energy rate error
        error_norm_Es_dot = (r_gamma - gamma_est) + (r_V_dot - vdot_est) / self.g

        #Desired thrust
        # thrust = self.param.trim_thrust +self.weight * (self.param.K_thrustp * (gamma_est + vdot_est / self.g) + self.param.K_thrusti * self.error_norm_Es_dot_integral)
        thrust_unsat = ( self.param.trim_thrust
               + self.weight * ( self.param.K_thrustp * (gamma_est + vdot_est / self.g)
                                 + self.param.K_thrusti * self.error_norm_Es_dot_integral ) )

        thrust = float(np.clip(thrust_unsat, 0.0, self.thr_max))

        # Thrust anti-windup
        # If at upper limit and error > 0, integrating would push further into sat -> freeze integral.
        # If at lower limit and error < 0, freeze integral.
        allow_I = True
        if thrust >= self.thr_max - 1e-9 and error_norm_Es_dot > 0.0:
            allow_I = False
        if thrust <= 0.0 + 1e-9 and error_norm_Es_dot < 0.0:
            allow_I = False

        if allow_I:
            self.error_norm_Es_dot_integral += error_norm_Es_dot * self.dt
            self.error_norm_Es_dot_integral = np.clip(
                self.error_norm_Es_dot_integral,
                -self.param.norm_Es_dot_integral_max, self.param.norm_Es_dot_integral_max
            )

        #-------------------Desired Pitch-------------------#
        #Energy rate distribution term error
        error_dist_term = (r_gamma - gamma_est) - (r_V_dot - vdot_est) / self.g

        #Desired pitch
        pitch_unsat = self.param.K_pitchi * self.error_dist_term_integral - self.param.K_pitchp * (gamma_est - vdot_est / self.g)

        pitch = float(np.clip(pitch_unsat, np.deg2rad(-20), np.deg2rad(20)))

        allow_I = True
        if pitch >= np.deg2rad(20) - 1e-9 and error_dist_term > 0.0:
            allow_I = False
        if pitch <= np.deg2rad(-20) + 1e-9 and error_dist_term < 0.0:
            allow_I = False

        if allow_I:
            self.error_dist_term_integral += error_dist_term * self.dt
            self.error_dist_term_integral = np.clip(
                self.error_dist_term_integral,
                -self.param.dist_term_integral_max, self.param.dist_term_integral_max
            )
        # print("r_gamma: {:5.2f} gamma_est: {:5.2f}, r_V_dot:{:5.2f}; vdot_est:{:5.2f}".format(r_gamma, gamma_est,r_V_dot,vdot_est))

        return thrust, pitch
    
    def compute_control(self, t, ref_data, actual_data, ref_thrust= None, ref_pitch=None):
        # actual data
        x = actual_data['x_est']
        y= actual_data['y_est']
        z = actual_data['z_est']
        roll = actual_data['roll_est']
        pitch = actual_data['pitch_est']
        yaw = actual_data['yaw_est']
        vx_est = actual_data['vx_est']
        vy_est = actual_data['vy_est']
        vz_est = actual_data['vz_est']
        V_est = actual_data['v_est']
        gamma_est = actual_data['gamma_est']
        vdot_est =actual_data['vdot_est']
        p_est = actual_data['p_est']
        q_est = actual_data['q_est']
        r_est = actual_data['r_est']

        #-------------------Compute Reference Outer Loop and Heading------------------#
        #Get desired thrust and pitch (we can remove this if we want to fully separate the two functions during implementation)
        if ref_thrust == None or ref_pitch == None:
            ref_thrust, ref_pitch = self.compute_thrust_pitch(t, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est) #Outer loop TECS controller
 
        ref_heading = ref_data['des_heading']
        r_heading = float(ref_heading) #desired heading yaw rate

        #-------------------Elevator Control-------------------#
        #Compute errors
        pitch = -1 * pitch # remaps nose-up-negative to nose-up-positive (NED)
        error_pitch = _wrap_pi(ref_pitch - pitch)

        q_turn = np.sin(roll) * np.cos(pitch) * np.tan(roll) * self.g / V_est
        error_q = q_turn - q_est # turning pitch
        error_q  = (error_q + np.pi) % (2 * np.pi) - np.pi

        nz_excess = (1.0/np.cos(roll)) - 1.0 # Steady-turn feed-forward
        ele_ff_phi = self.param.K_phi_elev * nz_excess   

        #Integral of pitch error
        self.error_pitch_integral += error_pitch * self.dt
        if self.error_pitch_integral > self.param.pitch_integral_max:
            self.error_pitch_integral = self.param.pitch_integral_max
        elif self.error_pitch_integral < -self.param.pitch_integral_max:
            self.error_pitch_integral = -self.param.pitch_integral_max
        
        #Control commands for elevator
        elev_cmd = self.param.trim_elev + (self.param.K_elevp * error_pitch + self.param.K_elevi * self.error_pitch_integral) + self.param.K_q * error_q
        elev_cmd += ele_ff_phi # feed-forward elevator wrt to roll angle
        elev_cmd = np.clip(elev_cmd, -1,1 ) #Saturation

        #-------------------Throttle Control-------------------#
        self.throttle_cmd = np.clip(ref_thrust / self.thr_max, 0.0 ,1.0) # This bypasses the throttle and assume throttle level is the thrust percentage

        # #-------------------Lateral Heading Control using WP_NAV-------------------#
        chi     = np.arctan2(vy_est, vx_est) # current ground course track angle
        chi_ref = r_heading                  # navigation desired heading
        chi_err = _wrap_pi(chi_ref-chi)*-1
        if abs(chi_err) < self.chi_deadband:   # small deadband to prevent sudden flip near wrap
            chi_err = 0.0

        chi_dot_des = self.param.k_chi * chi_err  # desired yaw rate to correct heading error
        Vg = max(V_est, 0.05) #ground speed, avoid div by zero
        phi_des = np.arctan2(Vg * chi_dot_des , self.g) #"Modelling and Control of Fixed Wing UAV pg 38"

        phi_des = float(np.clip(phi_des, -self.phi_lim, self.phi_lim)) 
        dphi_max = self.phi_dot_lim * self.dt
        phi_des = np.clip(phi_des - self._phi_cmd, -dphi_max, dphi_max) + self._phi_cmd
        self._phi_cmd = float(np.clip(phi_des, -self.phi_lim, self.phi_lim))

        # innter loop roll control modes
        if self.roll_mode == "stabilized":
            # Emulate onboard roll stabilizer in sim: PD on (phi, p) -> aileron
            e_phi = _wrap_pi(self._phi_cmd - roll)
            # Integrator with clamp
            self._e_phi_int += e_phi * self.dt
            self._e_phi_int = float(np.clip(self._e_phi_int, -self.param.i_phi_max, self.param.i_phi_max))

            # D on measured roll-rate
            d_term = - self.param.K_phi_d * p_est    # p_est in rad/s

            ail_cmd = ( self.param.trim_ail
                + self.param.K_phi_p * e_phi
                + self.param.K_phi_i * self._e_phi_int
                + d_term )

            ail_cmd = float(np.clip(ail_cmd, -self.param.da_max, self.param.da_max))

        elif self.roll_mode == "phi_stick": # heading error -> desired bank -> aileron cmd
            # Output bank stick directly (designed to command on real onboard gyro)
            # maps [-phi_lim, +phi_lim] -> [-1, +1]
            ail_cmd = float(np.clip(phi_des / self.phi_lim, -1.0, 1.0))

        else:  # "direct" = yaw error --> aileron cmd
            err_yaw = (r_heading - yaw) # error of haeding angle
            err_yaw  = (err_yaw + np.pi) % (2 * np.pi) - np.pi
            error_r_deriv = (err_yaw - self.error_r_last)/self.dt
            self.error_r_last = err_yaw
            self.error_r_integral += err_yaw*self.dt
            if self.error_r_integral > self.param.r_integral_max:
                self.error_r_integral = self.param.r_integral_max
            elif self.error_r_integral < -self.param.r_integral_max:
                self.error_r_integral = -self.param.r_integral_max

            ail_cmd = (self.param.trim_ail
                   + self.param.K_deltap * err_yaw
                   + self.param.K_deltai * self.error_r_integral
                   + self.param.K_deltad * error_r_deriv)
            ail_cmd = float(np.clip(ail_cmd, -1.0, 1.0))


        # #-------------------Coordinated Turn Control-------------------#
        rud_cmd = 0
        rud_cmd = np.clip(rud_cmd,-1,1)
        #Set history variables
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_pitch = pitch
        self.prev_yaw = yaw

        return ail_cmd, elev_cmd, self.throttle_cmd, rud_cmd
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import json
import matplotlib.animation as animation
from matplotlib.patches import Circle, Wedge, Arrow

# --- Configuration & Constants (Matching Firmware) ---
RSSI_BUF_SIZE = 6000
INTERPOLATION_INTERVAL_US = 100
DSHOT_ESC_RESOLUTION_HZ = 40000000
M_PI = math.pi
M_PI_2 = math.pi / 2.0

class PhysicsConfig:
    def __init__(self):
        # Disc
        self.disc_diameter_m = 0.350
        self.disc_radius_m = self.disc_diameter_m / 2.0
        self.disc_mass_kg = 0.500
        self.disc_height_m = 0.040
        
        # Motors/Wheels ("Opposite ends" -> mounted at Disc Radius)
        self.motor_mount_radius_m = self.disc_radius_m
        # Combined mass of motor + wheel assembly
        self.motor_assembly_mass_kg = 0.050 
        
        # Wheels
        self.wheel_diameter_m = 0.050
        self.wheel_radius_m = self.wheel_diameter_m / 2.0
        self.wheel_width_m = 0.020
        
        # Calculated Inertia
        # I_disc = 0.5 * M * R^2
        self.I_disc = 0.5 * self.disc_mass_kg * (self.disc_radius_m ** 2)
        # I_motors = 2 * (m * R^2) (Point mass approximation at rim)
        self.I_motors = 2 * (self.motor_assembly_mass_kg * (self.motor_mount_radius_m ** 2))
        self.I_total = self.I_disc + self.I_motors
        
        # Tuning: Throttle 400 -> 35.97 Hz Bot Rotation
        # 35.97 Hz Bot = 226.0 rad/s
        # V_wheel_linear = 226.0 * 0.175 = 39.55 m/s
        # Omega_wheel = 39.55 / 0.025 = 1582.0 rad/s
        # RPM_wheel = 1582.0 * 60 / 2pi = 15107 RPM
        # Ratio = 15107 / 400 = 37.77 RPM/ThrottleUnit
        
        self.motor_kv_rpm_per_unit = 37.77
        
        # Physics Parameters
        self.drag_coeff = 0.0001 # Small linear drag
        self.motor_torque_k = 1.0 # Stiffness of motor loop (response speed)
        
        # Total Mass for Translation
        # 0.5 disc + 0.1 motors + misc
        self.total_mass_kg = 0.6 

class PhysicsState:
    def __init__(self):
        self.angular_velocity_rad_s = 0.0
        self.angle_rad = 0.0
        # Translation (World Frame)
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

class RSSICircularBuffer:
    def __init__(self):
        # Using numpy arrays for fixed size simulation or just list
        self.rssi = [0] * RSSI_BUF_SIZE
        self.timestamp = [0] * RSSI_BUF_SIZE
        self.head = 0
        self.tail = 0
        self.last_timestamp = 0
        self.last_rssi = 0
        
class ControlInput:
    def __init__(self):
        self.throttle = 0
        self.vector_x = 0.0
        self.vector_y = 0.0

class RotationState:
    def __init__(self):
        self.rotation_rate = 0.5 # Default 0.5 Hz to match C fallback
        self.phase_offset = 0.0
        self.last_peak_timestamp = 0
        self.estimated_period_us = 2000000.0 # 2 seconds

class AppConfig:
    def __init__(self):
        self.dshot_pin_a = 8
        self.dshot_pin_b = 9
        self.led_pin = 48
        self.rotation_source = 1 # ESPNOW
        self.step_lag = 5
        self.step_window = 5
        self.smoothing_window = 20
        self.throttle_multiplier = 1.0
        self.translation_multiplier = 4.0
        self.correlation_window = 1000
        self.phase_offset = 0.0

# --- Global / System State ---
# In a real system these are static/globals. Here they are part of the Sim class but we can treat them as "system" state.

class ESPFirmwareSimulation:
    def __init__(self, config_dict=None):
        self.g_interpolated_rssi_buf = RSSICircularBuffer()
        self.g_raw_rssi_buf = RSSICircularBuffer() # Used for dumping in C, we simulate it too
        self.g_control_input = ControlInput()
        self.g_rotation_state = RotationState()
        self.g_config = AppConfig()
        
        if config_dict:
            print("Applying config from JSON...")
            # Map JSON to AppConfig
            # AppConfig keys matching JSON keys roughly
            for k, v in config_dict.items():
                if hasattr(self.g_config, k):
                    setattr(self.g_config, k, v)
                    print(f"  {k} = {v}")
        
        
        # Physics Engine
        self.phys_config = PhysicsConfig()
        self.phys_state = PhysicsState()
        
        # Simulation State
        self.current_time_us = 0
        
        # Rotation Task Internal Static State
        self.rot_errors = [0] * 1000 # Max Lags size approx
        self.rot_lags = [0] * 1000
        
        # Logs for Plotting
        self.log_time = []
        self.log_rssi_sample = [] # Sample from head
        self.log_dshot_a = []
        self.log_dshot_b = []
        self.log_led_r = []
        self.log_led_g = []
        self.log_led_b = []
        self.log_rate = []
        self.log_phase = []
        self.log_phys_rate_hz = [] # Physics Truth
        self.log_phys_angle = [] # Physics Truth (Orientation)
        self.log_phys_x = []
        self.log_phys_y = []
        self.log_peak = [] # Boolean for peak detected frame
        
        # Set some default control input to see motor action
        self.g_control_input.throttle = 400
        self.g_control_input.vector_x = 0.5
        self.g_control_input.vector_y = 0.5

    def update_physics(self, dt_sec, dshot_a, dshot_b):
        # 1. Determine Target Wheel RPMs
        # 48 is min throttle (stop), < 48 is specialized
        
        ta = dshot_a if dshot_a >= 48 else 0
        tb = dshot_b if dshot_b >= 48 else 0
        
        # If both motors spin to ROTATE the bot, they must push in opposite directions relative to the hub?
        # Actually, "Motors spin in opposite directions" -> They create a couple.
        # If mounted +Y and -Y.
        # Motor A at +Y pushes -X (CW torque?). Motor B at -Y pushes +X (CW torque?).
        # So they cooperate.
        # Let's average the throttle for the rotational component.
        # (Differential throttle causes translation, but for simple rotation physics we can take the mean common mode)
        
        avg_throttle = (ta + tb) / 2.0
        
        # Target Wheel Speed (no slip)
        target_wheel_rpm = avg_throttle * self.phys_config.motor_kv_rpm_per_unit
        target_wheel_rad_s = target_wheel_rpm * 2 * M_PI / 60.0
        
        # Kinematic relationship to Bot Speed
        # Omega_bot * R_bot = Omega_wheel * R_wheel
        target_bot_rad_s = target_wheel_rad_s * self.phys_config.wheel_radius_m / self.phys_config.motor_mount_radius_m
        
        # Dynamics
        # Torque = (Target - Current) * K
        error_rad_s = target_bot_rad_s - self.phys_state.angular_velocity_rad_s
        
        drive_torque = error_rad_s * self.phys_config.motor_torque_k
        
        # --- Torque to Force Breakdown ---
        # drive_torque is the NET torque.
        # Assume it's distributed based on the throttle ratio?
        # Actually, "Target Wheel Speed" was based on AVERAGE throttle.
        # But real physics: Each motor has its own Target RPM -> Own Force.
        
        # Let's refine:
        # Motor A Target Speed:
        ta_rpm = ta * self.phys_config.motor_kv_rpm_per_unit
        ta_rad_s = ta_rpm * 2 * M_PI / 60.0
        # Motor B Target Speed:
        tb_rpm = tb * self.phys_config.motor_kv_rpm_per_unit
        tb_rad_s = tb_rpm * 2 * M_PI / 60.0
        
        # Current Wheel Speed (Assuming no slip, tied to bot angular vel for now? No, that's coupling.)
        # If we assume perfect grip, Wheel Speed = Bot Angular Speed * Ratio.
        # But if we differentiate:
        # Force A = K * (Target_A_Wheel_Linear_Vel - Current_Wheel_Linear_Vel) ?
        
        # Let's simplify:
        # We calculated `drive_torque` based on the Loop of "Target Bot Rate".
        # Let's split this torque back into forces.
        # Or better: Calculate Force A and Force B independently.
        
        # Force = Torque_at_wheel / Wheel_Radius?
        # Torque_at_wheel approx proportional to (TargetRPM - CurrentRPM).
        # Linear Speed at Rim = Bot_Omega * Disc_Radius.
        # Wheel Linear Speed (ground) = Wheel_Omega * Wheel_Radius.
        # No slip -> Linear Speed at Rim == Wheel Linear Speed (ground)?
        # Actually, if bot rotates CW:
        # Rim at Motor A moves "Forward" relative to ground?
        # Velocity_Rim_A = Omega_Bot * R_Mount (Tangential).
        # Wheel A must spin to match that?
        
        # Effective Linear Velocity of Motor Mount:
        v_mount_mag = self.phys_state.angular_velocity_rad_s * self.phys_config.motor_mount_radius_m
        
        # Motor A (Left, +Y in body if X is fwd? No, earlier we said A pushes -X).
        # Let's stick to the "Torque" model we tuned.
        # Torque_A = K_motor * (Target_A - Current_Speed_A)
        # Torque_B = K_motor * (Target_B - Current_Speed_B)
        # Net Torque = (Torque_A + Torque_B) * Leverage?
        
        # Let's infer Force from the tuned `drive_torque`.
        # `drive_torque` was derived from `avg_throttle`.
        # Let `force_magnitude_per_throttle` be implied.
        
        # Re-derive Force A and B consistent with the tuned torque:
        # If ta == tb, Net Torque = drive_torque. Net Force = ?
        # If ta == tb, we want PURE ROTATION -> Net Force = 0?
        # NO. The user says "Motors pulsing... translational drift".
        # If A and B oppose each other for Torque, do they ADD for Force?
        # Setup:
        # Disc. Motor A at 9 o'clock. Motor B at 3 o'clock.
        # To spin CCW: A pushes DOWN. B pushes UP. -> Couple. Net Force 0.
        # To Translate UP: A pushes UP. B pushes UP. -> Net Force UP. Torque 0.
        # So:
        # Force_A ~ Throttle_A. Direction? 
        # For Rotation (CCW): A needs to push "Back". B needs to push "Forward".
        # So Vector A is "Back". Vector B is "Forward".
        # If Throttle A > Throttle B (Pulse):
        # Result: More "Back" force. Net Force is "Back".
        # Net Torque is CCW (since A contributes to CCW).
        # So `Force_A` contributes to CCW Torque AND "Back" Translation.
        # `Force_B` contributes to CCW Torque (if positive?) No.
        # If B pushes "Forward", it helps CCW Torque.
        # So BOTH A and B help CCW Torque?
        # Yes, standard differential drive on rim:
        # Left Wheel pushes Back -> Bot turns Left (CCW).
        # Right Wheel pushes Fwd -> Bot turns Left (CCW).
        # So "Forward" rotation requires A(Back) + B(Fwd).
        # Net Force = B(Fwd) + A(Back) = B - A.
        
        # So:
        # Force_A_mag = calc_force(ta)
        # Force_B_mag = calc_force(tb)
        # Torque = (Force_A_mag + Force_B_mag) * Radius.
        # Net Linear Force (Body Framework X-axis) = Force_B_mag - Force_A_mag.
        
        # We need to preserve the Tuned Torque behavior.
        # In tuned model: drive_torque ~ (Target - Current).
        # Let's split `drive_torque` into A and B components based on Throttle ratio.
        # Or calculate explicitly:
        
        # Current Linear Speed (Virtual) for Motor Loop:
        v_current = self.phys_state.angular_velocity_rad_s * self.phys_config.motor_mount_radius_m
        
        # Target Linear Speeds
        v_target_a = ta_rpm * (2*M_PI/60) * 0.025 * (0.175/0.175) # ... simplify
        # actually target_wheel_rad_s * wheel_radius = target_linear_v_rim
        v_target_a = ta_rad_s * self.phys_config.wheel_radius_m
        v_target_b = tb_rad_s * self.phys_config.wheel_radius_m
        
        # K_force (Linear version of torque k)
        # Torque = Force * R_mount.
        # Force = Torque / R_mount.
        # K_force = K_torque / (R_mount^2) ? Dimensional analysis...
        # Let's just use a scaling factor `k_f`
        
        # We know `drive_torque = K * (v_target_avg_ang - v_curr_ang)`.
        # approx `K * (v_target_lin - v_curr_lin) / R_mount` * R_mount?
        # Let's calculate Force A and B directly using a `k_force`.
        
        k_force = self.phys_config.motor_torque_k / self.phys_config.motor_mount_radius_m
        
        # Note: v_target are speeds "along the torque generation direction".
        # For A: "Back". For B: "Forward".
        # But `v_current` is scalar rotational speed (tangential).
        # So `v_current` corresponds to "Forward at B" and "Back at A".
        
        f_a = k_force * (v_target_a - v_current) # Positive f_a means "Pushing Back harder than current speed"
        f_b = k_force * (v_target_b - v_current)
        
        drive_torque_new = (f_a + f_b) * self.phys_config.motor_mount_radius_m
        
        # Check consistency: if ta=tb=400, v_target_a=v_target_b. f_a=f_b. 
        # Torque = 2 * f_a * R.
        # Matches logic? Yes.
        
        # Net Linear Force (Body Frame)
        # B pushes Forward (+X body). A pushes Back (-X body).
        # F_body = f_b - f_a
        
        f_net_body = f_b - f_a
        
        # Drag
        drag_torque = self.phys_config.drag_coeff * self.phys_state.angular_velocity_rad_s
        net_torque = drive_torque_new - drag_torque
        
        alpha = net_torque / self.phys_config.I_total
        self.phys_state.angular_velocity_rad_s += alpha * dt_sec
        self.phys_state.angle_rad += self.phys_state.angular_velocity_rad_s * dt_sec
        self.phys_state.angle_rad %= (2 * M_PI)
        
        # Translational Dynamics
        # Rotate F_net_body to World Frame
        # Bot Angle 0 -> Body X aligns with World X?
        # Let's assume Angle 0 means Heading +X.
        theta = self.phys_state.angle_rad
        fx_world = f_net_body * math.cos(theta)
        fy_world = f_net_body * math.sin(theta)
        
        # Linear Drag (Translation)
        lin_drag_k = 0.5 
        fx_world -= lin_drag_k * self.phys_state.vx
        fy_world -= lin_drag_k * self.phys_state.vy
        
        ax = fx_world / self.phys_config.total_mass_kg
        ay = fy_world / self.phys_config.total_mass_kg
        
        self.phys_state.vx += ax * dt_sec
        self.phys_state.vy += ay * dt_sec
        self.phys_state.x += self.phys_state.vx * dt_sec
        self.phys_state.y += self.phys_state.vy * dt_sec

    def interpolate_rssi(self, buf, timestamp, rssi):
        # Implementation of interpolate_rssi from C
        
        # 1. Add to Raw Buffer (Logic from C)
        self.g_raw_rssi_buf.rssi[self.g_raw_rssi_buf.head] = rssi
        self.g_raw_rssi_buf.timestamp[self.g_raw_rssi_buf.head] = timestamp
        self.g_raw_rssi_buf.last_timestamp = timestamp
        self.g_raw_rssi_buf.head = (self.g_raw_rssi_buf.head + 1) % RSSI_BUF_SIZE
        if self.g_raw_rssi_buf.head == self.g_raw_rssi_buf.tail:
            self.g_raw_rssi_buf.tail = (self.g_raw_rssi_buf.tail + 1) % RSSI_BUF_SIZE
            
        # 2. Main Interpolation Logic
        if buf.last_timestamp == 0:
            buf.rssi[buf.head] = rssi
            buf.timestamp[buf.head] = timestamp
            buf.last_timestamp = timestamp
            buf.last_rssi = rssi
            buf.head = (buf.head + 1) % RSSI_BUF_SIZE
            return
            
        if timestamp - buf.last_timestamp > 100000:
            # Gap reset
            buf.last_timestamp = timestamp
            buf.last_rssi = rssi
            buf.rssi[buf.head] = rssi
            buf.timestamp[buf.head] = timestamp
            buf.head = (buf.head + 1) % RSSI_BUF_SIZE
            if buf.head == buf.tail:
                buf.tail = (buf.tail + 1) % RSSI_BUF_SIZE
            return
            
        if timestamp <= buf.last_timestamp:
            return # out of order
            
        prev_rssi = buf.last_rssi
        prev_ts = buf.last_timestamp
        
        # Reconstruct logical "last_idx" from head
        last_idx = (buf.head - 1 + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
        # In C: target_ts = buf->timestamp[last_idx] + INTERPOLATION_INTERVAL_US
        # Note: if buf->head was just incremented, last_idx points to the last written.
        # But if we just started, head might be 1. last_idx 0.
        
        # Initial target logic correction:
        # If we have only 1 point, target is that point + 100us.
        target_ts = buf.timestamp[last_idx] + INTERPOLATION_INTERVAL_US
        
        while target_ts <= timestamp:
            ratio = (target_ts - prev_ts) / (timestamp - prev_ts)
            val = int(prev_rssi + (rssi - prev_rssi) * ratio)
            
            buf.rssi[buf.head] = val
            buf.timestamp[buf.head] = target_ts
            buf.head = (buf.head + 1) % RSSI_BUF_SIZE
            if buf.head == buf.tail:
                buf.tail = (buf.tail + 1) % RSSI_BUF_SIZE
            
            target_ts += INTERPOLATION_INTERVAL_US
            
        buf.last_timestamp = timestamp
        buf.last_rssi = rssi

    def calculate_autocorr_error(self, buf, head, lag, corr_window):
        total_diff = 0
        
        # C uses start_idx = (head - corr_window + SIZE) % SIZE
        start_idx = (head - corr_window + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
        cur_idx = start_idx
        idx_B = (cur_idx - lag + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
        
        # Python implementation of sum(|A - B|)
        # Slower than C but logic is same
        # We can optimize slightly by slicing if array isn't circular, but here it wraps.
        
        for _ in range(corr_window):
            val_a = buf.rssi[cur_idx]
            val_b = buf.rssi[idx_B]
            total_diff += abs(val_a - val_b)
            
            cur_idx = (cur_idx + 1) % RSSI_BUF_SIZE
            idx_B = (idx_B + 1) % RSSI_BUF_SIZE
            
        return total_diff

    def task_rotation(self):
        # Mimic rotation_task loop body
        # 1. Check data availability
        head = self.g_interpolated_rssi_buf.head
        tail = self.g_interpolated_rssi_buf.tail
        count = (head - tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
        corr_window = self.g_config.correlation_window
        
        if count < corr_window * 2:
            return
            
        start_lag = 200
        end_lag = 1000
        step_lag = self.g_config.step_lag
        
        # Coarse Search
        min_diff = float('inf')
        max_diff = float('-inf')
        
        count_lags = 0
        # Re-use buffer arrays
        errors = self.rot_errors
        lags = self.rot_lags
        
        curr_lag = start_lag
        while curr_lag < end_lag:
            diff = self.calculate_autocorr_error(self.g_interpolated_rssi_buf, head, curr_lag, corr_window)
            errors[count_lags] = diff
            lags[count_lags] = curr_lag
            
            if diff < min_diff: min_diff = diff
            if diff > max_diff: max_diff = diff
            
            count_lags += 1
            curr_lag += step_lag
            
        # Process Slopes
        best_lag = 0
        found_valid = False
        
        if count_lags > 3:
            slopes = [] 
            max_slope = 0
            
            for i in range(count_lags - 1):
                s = errors[i+1] - errors[i]
                slopes.append(s)
                if abs(s) > max_slope: max_slope = abs(s)
                
            if max_slope < 1.0: max_slope = 1.0
            
            LAG_WINDOW = 1
            
            for i in range(count_lags - 2):
                if i >= len(slopes) - 1: break # Safety
                
                norm_curr = slopes[i] / max_slope
                norm_next = slopes[i+1] / max_slope
                
                if norm_curr < 0 and norm_next > 0:
                    valley_idx = i + 1
                    
                    norm_error = (errors[valley_idx] - min_diff) / (max_diff - min_diff)
                    
                    if norm_error < 0.5:
                        d2_sum = 0
                        count_d2 = 0
                        
                        # Look back
                        # for (int k = i; k >= i - (2 * LAG_WINDOW); k--)
                        for k in range(i, i - (2 * LAG_WINDOW) - 1, -1):
                            if k < 0: continue
                            d2 = (slopes[k+1] - slopes[k]) / max_slope
                            d2_sum += d2
                            count_d2 += 1
                            
                        if count_d2 > 0:
                            avg_d2 = d2_sum / count_d2
                            if avg_d2 > 0.05:
                                best_lag = lags[valley_idx]
                                found_valid = True
                                break
                                
        final_lag = best_lag
        
        if not found_valid:
            self.g_rotation_state.estimated_period_us = 2000000.0
            self.g_rotation_state.rotation_rate = 0.5
            final_lag = 0
        else:
            # Fine Search
            fine_min_diff = float('inf')
            
            for i in range(-step_lag, step_lag + 1):
                lag = best_lag + i
                if lag < start_lag or lag > end_lag: continue
                diff = self.calculate_autocorr_error(self.g_interpolated_rssi_buf, head, lag, corr_window)
                if diff < fine_min_diff:
                    fine_min_diff = diff
                    final_lag = lag
                    
        if final_lag > 0:
            self.g_rotation_state.estimated_period_us = float(final_lag * INTERPOLATION_INTERVAL_US)
            self.g_rotation_state.rotation_rate = 1000000.0 / self.g_rotation_state.estimated_period_us
            
            # IQ Demodulation
            period_us = self.g_rotation_state.estimated_period_us
            window_duration = 4.0 * period_us
            
            # Limit window
            max_win = RSSI_BUF_SIZE * INTERPOLATION_INTERVAL_US
            if window_duration > max_win: window_duration = max_win
            
            samples_to_process = int(window_duration / INTERPOLATION_INTERVAL_US)
            if samples_to_process > RSSI_BUF_SIZE: samples_to_process = RSSI_BUF_SIZE
            
            sum_I = 0.0
            sum_Q = 0.0
            omega = 2.0 * M_PI / period_us
            
            # Ref time: timestamp[ref_idx] where ref_idx = head - 1
            ref_idx = (head - 1 + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
            t_ref = self.g_interpolated_rssi_buf.timestamp[ref_idx]
            
            for i in range(samples_to_process):
                idx = (ref_idx - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
                val = self.g_interpolated_rssi_buf.rssi[idx]
                t = self.g_interpolated_rssi_buf.timestamp[idx]
                
                dt = float(t - t_ref)
                angle = omega * dt
                
                sum_I += val * math.cos(angle)
                sum_Q += val * math.sin(angle)
                
            phi = math.atan2(sum_Q, sum_I)
            
            # t_target = t_ref + (phi + PI) / omega
            dt_pi = (phi + M_PI) / omega
            self.g_rotation_state.last_peak_timestamp = t_ref + int(dt_pi)

    def task_motor(self):
        # Mimic motor_task loop body
        now = self.current_time_us
        
        time_since_peak = now - self.g_rotation_state.last_peak_timestamp
        phase = 2.0 * M_PI * float(time_since_peak) / self.g_rotation_state.estimated_period_us
        
        # Apply Offset
        phase += self.g_config.phase_offset
        
        # Normalize 0..2PI
        phase = phase % (2.0 * M_PI)
        if phase < 0: phase += 2.0 * M_PI
        
        # Determine LED Color
        # Green for 45 deg arc opposite peak (Heading) -> Peak is at phase=0?
        # In IQ code: "t_target" is where phase would be PI? 
        # C Code Line 766: "Find time where phase would be PI... last_peak_timestamp = ... "
        # So last_peak_timestamp is effectively checking PI intersection.
        # So at last_peak_timestamp, phase should be PI.
        # But calculation `phase = 2PI * dt / period` implies phase grows linearly from last_peak_timestamp.
        # If last_peak_timestamp is "PI", then at `now` == `last_peak`, `phase` calc gives 0 (since dt=0).
        # This seems inconsistent or strictly defined: `time_since_peak` is 0 at `last_peak_timestamp`.
        # So `phase` variable here starts at 0 at `last_peak_timestamp`.
        # C code `HEADING_START` is `PI - PI/8`.
        
        # LED Logic
        r, g, b = 0, 0, 0
        max_intensity = 255
        
        if phase < M_PI:
             ratio = phase / M_PI
             r = int(max_intensity * ratio)
             b = int(max_intensity * (1.0 - ratio))
        else:
             ratio = (phase - M_PI) / M_PI
             r = max_intensity
             g = int(max_intensity * ratio)
             b = int(max_intensity * ratio) # Wait, C code says g and b both ratio? 
             # C Line 415: b = (uint8_t)(max_intensity * ratio);
             # Red -> White
             
        HEADING_START = M_PI - M_PI / 8.0
        HEADING_END = M_PI + M_PI / 8.0
        
        if phase > HEADING_START and phase < HEADING_END:
            r, g, b = 0, max_intensity, 0
            
        # Motor Mixing
        TRANSLATION_BASE_STRENGTH = 100
        throttle = self.g_control_input.throttle
        leftDShot = throttle
        rightDShot = throttle
        
        if throttle >= 48:
            throttle_rescaled = int(throttle * self.g_config.throttle_multiplier)
            leftDShot = throttle_rescaled
            rightDShot = throttle_rescaled
            
            vx = self.g_control_input.vector_x
            vy = self.g_control_input.vector_y
            mag = math.sqrt(vx*vx + vy*vy)
            
            if mag > 0.1:
                target_angle = math.atan2(-vy, vx) + M_PI_2
                if target_angle < 0: target_angle += 2.0 * M_PI
                if target_angle >= 2.0 * M_PI: target_angle -= 2.0 * M_PI
                
                diff = phase - target_angle
                while diff <= -M_PI: diff += 2.0 * M_PI
                while diff > M_PI: diff -= 2.0 * M_PI
                
                # Smooth Sinusoidal Modulation
                modulation = math.cos(diff)
                strength = TRANSLATION_BASE_STRENGTH * self.g_config.translation_multiplier * mag * modulation
                
                leftDShot = throttle_rescaled + strength
                rightDShot = throttle_rescaled - strength
                    
            if leftDShot < 48: leftDShot = 48
            if leftDShot > 2047: leftDShot = 2047
            if rightDShot < 48: rightDShot = 48
            if rightDShot > 2047: rightDShot = 2047
            
        # Log outputs
        self.log_dshot_a.append(leftDShot)
        self.log_dshot_b.append(rightDShot)
        self.log_led_r.append(r)
        self.log_led_g.append(g)
        self.log_led_b.append(b)
        self.log_phase.append(phase)
        
        # Step Physics using these DShot values
        # They are applied for the NEXT millisecond
        # Note: In C this runs in parallel.
        self.update_physics(0.001, leftDShot, rightDShot)
        
    def load_data(self, filename):
        print(f"Loading data from {filename}...")
        raw_ts = []
        raw_rssi = []
        raw_throttle = []
        raw_vx = []
        raw_vy = []
        
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                # Check for new columns
                has_control = "Throttle" in header

                for row in reader:
                    if not row: continue
                    try:
                        # Index, Timestamp_US, RSSI
                        t = int(row[1])
                        r = int(row[2])
                        raw_ts.append(t)
                        raw_rssi.append(r)
                        
                        if has_control and len(row) >= 6:
                            raw_throttle.append(int(row[3]))
                            raw_vx.append(float(row[4]))
                            raw_vy.append(float(row[5]))
                        else:
                            # Default or previous logic
                            raw_throttle.append(400)
                            raw_vx.append(0.5)
                            raw_vy.append(0.5)
                            
                    except ValueError:
                        continue
                        
        except FileNotFoundError:
            print(f"Error: File {filename} not found.")
            return None, None, None, None, None
            
        print(f"Loaded {len(raw_ts)} samples.")
        return raw_ts, raw_rssi, raw_throttle, raw_vx, raw_vy

    def run_simulation(self, raw_ts, raw_rssi, raw_throttle, raw_vx, raw_vy):
        if not raw_ts: return
        
        start_time = raw_ts[0]
        end_time = raw_ts[-1]
        
        self.current_time_us = start_time
        curr_raw_idx = 0
        total_len = len(raw_ts)
        
        # 1ms Tick Loop (1000us)
        TICK_US = 1000
        
        while self.current_time_us <= end_time + TICK_US: # Run a bit past
            # 1. Ingest Data for this tick
            # Find all raw points <= current_time_us that haven't been processed
            # Actually, interpolate_rssi is called ON ARRIVAL.
            # So as we step time, we check if any new packets "arrived".
            
            while curr_raw_idx < total_len and raw_ts[curr_raw_idx] <= self.current_time_us:
                self.interpolate_rssi(self.g_interpolated_rssi_buf, raw_ts[curr_raw_idx], raw_rssi[curr_raw_idx])
                
                # Update Control Input from CSV
                self.g_control_input.throttle = raw_throttle[curr_raw_idx]
                self.g_control_input.vector_x = raw_vx[curr_raw_idx]
                self.g_control_input.vector_y = raw_vy[curr_raw_idx]
                
                curr_raw_idx += 1
                
            # 2. Run Tasks
            self.task_rotation() # C: vTaskDelay(1) -> once per tick
            self.task_motor()    # C: vTaskDelay(1) -> once per tick
            
            # 3. Log
            self.log_time.append(self.current_time_us)
            
            # Log current interpolated value for vis (take head-1)
            idx = (self.g_interpolated_rssi_buf.head - 1 + RSSI_BUF_SIZE) % RSSI_BUF_SIZE
            self.log_rssi_sample.append(self.g_interpolated_rssi_buf.rssi[idx])
            
            self.log_rate.append(self.g_rotation_state.rotation_rate)
            
            # Log Physics Truth
            self.log_phys_rate_hz.append(self.phys_state.angular_velocity_rad_s / (2 * M_PI))
            self.log_phys_angle.append(self.phys_state.angle_rad)
            self.log_phys_x.append(self.phys_state.x)
            self.log_phys_y.append(self.phys_state.y)
            
            # Advance
            self.current_time_us += TICK_US
            
        print("Simulation Complete.")

    def render_video(self, filename="simulation_video.mp4", fps=60, slowdown=100):
        print(f"Rendering video to {filename} (Slowdown: {slowdown}x)...")
        
        # Resample data to FPS
        total_time_us = self.log_time[-1] - self.log_time[0]
        total_time_s = total_time_us / 1e6
        total_frames = int(total_time_s * fps * slowdown)
        ts_step = 1e6 / (fps * slowdown)
        
        # Create figure
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect('equal')
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_title("Physics Simulation")
        
        # Elements
        # 1. Disc Body
        disc_radius = self.phys_config.disc_radius_m # 0.175
        disc_patch = Circle((0, 0), disc_radius, color='gray', alpha=0.3)
        ax.add_patch(disc_patch)
        
        # 2. Orientation Line (Black line from center to edge)
        # We will update its data
        line_orientation, = ax.plot([], [], 'k-', linewidth=2)
        
        # Trajectory
        line_traj, = ax.plot([], [], 'g-', linewidth=1, alpha=0.5)
        
        # Grid
        ax.grid(True, which='both', linestyle='--', alpha=0.3)
        
        # 3. LED Indicator (Big colored circle in center or arc?)
        # User asked for "render the physics simulation".
        # Let's put a colored circle in the center representing the LED
        led_radius = 0.05
        led_patch = Circle((0, 0), led_radius, color='black')
        ax.add_patch(led_patch)
        
        # 4. Motors (Arrows)
        # Motor A at +R (relative to angle), pushing tangentially?
        # Motors are "opposite ends, 90 deg offset from heading".
        # If Heading is 0 (X axis?), Motors at +Y and -Y?
        # Or if Heading is +Y, Motors at +X and -X?
        # Let's assume Heading is +X (Angle 0).
        # Motors at 90 deg (Y) and 270 deg (-Y).
        # Motor A at +90 deg. Force direction?
        # To spin Positive (CCW), A (at +90) should push -X (CW?? Wait).
        # CCW Rotation: Torque is +Z. 
        # Force at +Y must be pointing -X to create +Z torque? (r x F) -> (0, R, 0) x (-1, 0, 0) = (0, 0, R) -> Positive Torque. Correct.
        # So Motor A at 90 deg point Left relative to heading.
        # Motor B at -90 deg point Right relative to heading.
        
        # Re-do arrows using plot for speed/simplicity
        line_motor_a, = ax.plot([], [], 'r-', linewidth=3, label='Motor A')
        line_motor_b, = ax.plot([], [], 'b-', linewidth=3, label='Motor B')
        
        # Helper to get interpolated sample
        # We have ~1000 Hz logs.
        time_map = np.array(self.log_time)
        
        # Unwrap angles for smooth interpolation
        unwrapped_angles = np.unwrap(self.log_phys_angle)
        
        def update_simple(frame):
            t_target = self.log_time[0] + frame * ts_step
            # Find index
            idx = np.searchsorted(time_map, t_target)
            if idx >= len(self.log_time): idx = len(self.log_time) - 1
            
            # Get State
            angle = self.log_phys_angle[idx]
            
            # Update Orientation Line
            # End point
            ex = disc_radius * math.cos(angle)
            ey = disc_radius * math.sin(angle)
            line_orientation.set_data([0, ex], [0, ey])
            
            # Update LED
            r = self.log_led_r[idx] / 255.0
            g = self.log_led_g[idx] / 255.0
            b = self.log_led_b[idx] / 255.0
            led_patch.set_color((r, g, b))
            
            
            # Interpolate Angle
            angle = np.interp(t_target, time_map, unwrapped_angles)
            
            # Interpolate Position
            x = np.interp(t_target, time_map, self.log_phys_x)
            y = np.interp(t_target, time_map, self.log_phys_y)
            
            # Update View Limit (Camera Follow)
            window = 0.5
            ax.set_xlim(x - window, x + window)
            ax.set_ylim(y - window, y + window)
            
            # Trajectory
            # For efficiency, just draw path up to current time? 
            # Or simplified: just tail?
            # Let's map 'idx' roughly
            idx = np.searchsorted(time_map, t_target)
            if idx > 1:
                # Downsample trajectory logging for speed?
                # Just plot log_phys_x[:idx]
                line_traj.set_data(self.log_phys_x[:idx:10], self.log_phys_y[:idx:10])

            # Orientation
            ex = disc_radius * math.cos(angle)
            ey = disc_radius * math.sin(angle)
            line_orientation.set_data([x, x + ex], [y, y + ey])
            
            # Body
            disc_patch.center = (x, y)
            
            # LED (Interpolate Colors)
            r = np.interp(t_target, time_map, self.log_led_r) / 255.0
            g = np.interp(t_target, time_map, self.log_led_g) / 255.0
            b = np.interp(t_target, time_map, self.log_led_b) / 255.0
            led_patch.set_color((r, g, b))
            led_patch.center = (x, y)
            
            # Motor A (Left/Red)
            angle_a = angle + M_PI_2
            px_a = x + disc_radius * math.cos(angle_a)
            py_a = y + disc_radius * math.sin(angle_a)
            
            thr_a = np.interp(t_target, time_map, self.log_dshot_a)
            len_a = (thr_a / 2047.0) * 0.1
            
            angle_f_a = angle_a + M_PI_2
            fx_a = px_a + len_a * math.cos(angle_f_a)
            fy_a = py_a + len_a * math.sin(angle_f_a)
            line_motor_a.set_data([px_a, fx_a], [py_a, fy_a])

            # Motor B (Right/Blue)
            angle_b = angle - M_PI_2
            px_b = x + disc_radius * math.cos(angle_b)
            py_b = y + disc_radius * math.sin(angle_b)
             
            thr_b = np.interp(t_target, time_map, self.log_dshot_b)
            len_b = (thr_b / 2047.0) * 0.1
            # Wait, CCW torque. R x F.
            # Pos B: (0, -R). Force vector F?
            # (0, -R, 0) x (Fx, Fy, 0) = (0, 0, R*Fx) (if Fy=0).
            # We want +Z torque. So Fx must be positive.
            # Direction 0 (Right).
            # Radial B: Angle - 90.
            # Tangent B: Angle - 90 + 90 = Angle.
            # So Force B is along Angle.
            
            angle_f_b = angle_b + M_PI_2
            fx_b = px_b + len_b * math.cos(angle_f_b)
            fy_b = py_b + len_b * math.sin(angle_f_b)
            
            line_motor_b.set_data([px_b, fx_b], [py_b, fy_b])
            
            return [line_orientation, led_patch, line_motor_a, line_motor_b, disc_patch, line_traj]
        ani = animation.FuncAnimation(fig, update_simple, frames=total_frames, blit=True)
        ani.save(filename, writer='ffmpeg', fps=fps)
        print("Video saved.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python simulate_rotation.py <csv_file>")
        return
        
    filename = sys.argv[1]
    if not os.path.exists(filename):
        print("File not found")
    
    # Load Config if exists
    json_file = os.path.splitext(filename)[0] + ".json"
    config_dict = None
    if os.path.exists(json_file):
        with open(json_file, 'r') as f:
            config_dict = json.load(f)
            
    sim = ESPFirmwareSimulation(config_dict=config_dict)
    raw_ts, raw_rssi, raw_th, raw_vx, raw_vy = sim.load_data(filename)
    
    sim.run_simulation(raw_ts, raw_rssi, raw_th, raw_vx, raw_vy)
    
    # Calculate Midway Rate
    mid_idx = len(sim.log_rate) // 2
    if 0 <= mid_idx < len(sim.log_rate):
        print(f"Index {mid_idx}/{len(sim.log_rate)}")
        print(f"Midway Estimated Rate: {sim.log_rate[mid_idx]} Hz")
    
    # Plotting
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    
    # RSSI
    axes[0].plot(sim.log_time, sim.log_rssi_sample, label='Interpolated RSSI', color='blue', linewidth=0.5)
    axes[0].scatter(raw_ts, raw_rssi, label='Raw', color='red', s=2, alpha=0.5)
    axes[0].set_ylabel('RSSI')
    axes[0].legend()
    axes[0].set_title(f'Simulation: {os.path.basename(filename)}')
    
    # Rate
    axes[1].plot(sim.log_time, sim.log_rate, label='Estimated Rate (Hz)', color='green')
    axes[1].plot(sim.log_time, sim.log_phys_rate_hz, label='Physics Rate (Hz)', color='black',  linestyle='--')
    axes[1].set_ylabel('Hz')
    axes[1].legend()
    
    # Phase
    axes[2].plot(sim.log_time, sim.log_phase, label='Phase (rad)', color='purple', linewidth=0.5)
    axes[2].set_ylabel('Rad')
    axes[2].legend()
    
    # DShot / Motor
    axes[3].plot(sim.log_time, sim.log_dshot_a, label='Left DShot', color='orange')
    axes[3].plot(sim.log_time, sim.log_dshot_b, label='Right DShot', color='cyan')
    axes[3].set_ylabel('DShot')
    axes[3].legend()
    axes[3].set_xlabel('Time (us)')
    
    out_png = f"sim_aligned_{os.path.basename(filename)}.png"
    plt.tight_layout()
    plt.savefig(out_png)
    print(f"Saved plot to {out_png}")
    
    # Render Video
    base = os.path.splitext(os.path.basename(filename))[0]
    vid_name = f"sim_video_{base}.mp4"
    sim.render_video(vid_name, slowdown=100)

if __name__ == "__main__":
    main()

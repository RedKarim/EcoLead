# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2025, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------
# ---------------------------------------------------------------------
# !\ mpc_agent.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import carla
import numpy as np
import os
from mpc_controller import MPCController # importing the MPC controller
import math
from ego_model import EgoModel
import csv
from datetime import datetime
import utils
import yaml
import time


def get_waypoints(waypoints_list, N, vehicle_x, vehicle_y, vehicle_psi, current_wp_idx):
    waypoints = []
    num_waypoints = len(waypoints_list)

    # Initialize variables
    min_distance = float('inf')
    closest_idx = current_wp_idx

    # Find the closest waypoint ahead of the vehicle
    for idx in range(current_wp_idx, num_waypoints):
        wp = waypoints_list[idx]
        dx = wp['x'] - vehicle_x
        dy = wp['y'] - vehicle_y
        distance = np.hypot(dx, dy)

        # Compute angle between vehicle heading and waypoint vector
        heading_to_wp = np.arctan2(dy, dx)
        angle = vehicle_psi - heading_to_wp
        angle = np.arctan2(np.sin(angle), np.cos(angle))  # Normalize angle between -pi and pi

        # Consider waypoints that are within 90 degrees in front of the vehicle
        if abs(angle) < np.pi / 2:
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx
        else:
            # Waypoint is behind the vehicle
            continue

    # Handle case when no waypoint ahead is found
    if min_distance == float('inf'):
        print("Warning: No waypoint ahead found. Using current waypoint index.")
        closest_idx = current_wp_idx
        return [1,2,3,4], 1000

    # Select the next N waypoints
    end_idx = min(closest_idx + N, num_waypoints)
    waypoints_subset = waypoints_list[closest_idx:end_idx]

    # Transform waypoints to vehicle coordinate system
    waypoints = []
    for wp in waypoints_subset:
        x_global = wp['x']
        y_global = wp['y']

        # Shift coordinates
        shift_x = x_global - vehicle_x
        shift_y = y_global - vehicle_y

        # Rotate coordinates
        x_vehicle = shift_x * np.cos(-vehicle_psi) - shift_y * np.sin(-vehicle_psi)
        y_vehicle = shift_x * np.sin(-vehicle_psi) + shift_y * np.cos(-vehicle_psi)

        # **Filter out waypoints behind the vehicle**
        if x_vehicle >= 0:
            waypoints.append((x_vehicle, y_vehicle))

    # If fewer than N waypoints, pad with the last waypoint
    if len(waypoints) < N:
        last_wp = waypoints[-1] if waypoints else (0, 0)
        while len(waypoints) < N:
            waypoints.append(last_wp)

    # Update the current waypoint index for next iteration
    new_current_wp_idx = closest_idx

    return waypoints[:N], new_current_wp_idx


def map_acceleration_to_throttle_brake(a_desired, a_max, a_min, should_brake=False):
    """
    Maps the desired acceleration to throttle and brake commands.

    :param a_desired: Desired acceleration (m/s^2)
    :param a_max: Maximum acceleration (m/s^2)
    :param a_min: Maximum deceleration (negative value) (m/s^2)
    :param should_brake: Boolean indicating if the vehicle should brake immediately
    :return: Tuple of (throttle, brake) commands
    """
    if should_brake:
        throttle = 0.0
        brake = 1.0  # Maximum brake
    else:
        if a_desired > 0:
            print("Throttle applied")
            throttle = np.clip(a_desired / a_max, 0.0, 1)
            brake = 0.0
        elif a_desired < 0:
            print("Brake applied")
            throttle = 0.0
            brake = np.clip(-a_desired / abs(a_min), 0.0,1)
        else:
            throttle = 0.0
            brake = 0.0

    return throttle, brake

def normalize_angle(angle):
    """
    Normalizes an angle to be within [-pi, pi].

    :param angle: Angle in radians.
    :return: Normalized angle in radians.
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def predict_state_with_latency(current_state, prev_control, dt, latency, vehicle_params):
    """
    レイテンシを考慮した状態予測（バイシクルモデル）
    
    :param current_state: 現在の状態 [x, y, ψ, v, CTE, eψ]
    :param prev_control: 前回の制御入力 [δ, a]
    :param dt: タイムステップ (s)
    :param latency: 予測するレイテンシ (s)
    :param vehicle_params: 車両パラメータ辞書
    :return: 予測された状態 [x, y, ψ, v, CTE, eψ]
    """
    x, y, psi, v, cte, epsi = current_state
    delta, a = prev_control
    
    # 車両パラメータ
    Lf = vehicle_params['Lf']
    Cd = vehicle_params['Cd']
    rho_a = vehicle_params['rho_a']
    Av = vehicle_params['Av']
    Mh = vehicle_params['Mh']
    mu = vehicle_params['mu']
    g = vehicle_params['g']
    
    # 統合ステップ数
    num_steps = int(latency / dt)
    
    # 状態を順伝播
    for _ in range(num_steps):
        # 抵抗力による正味加速度
        a_drag = 0.5 * Cd * rho_a * Av * (v**2) / Mh
        a_roll = mu * g
        a_net = a - a_drag - a_roll
        
        # 運動学方程式
        x_next = x + v * np.cos(psi) * dt
        y_next = y + v * np.sin(psi) * dt
        psi_next = psi + (v / Lf) * delta * dt
        v_next = v + a_net * dt
        
        # 速度は非負
        v_next = max(v_next, 0.0)
        
        # 角度正規化
        psi_next = normalize_angle(psi_next)
        
        # CTE と eψ の更新（車両座標系での伝播）
        cte_next = cte + v * np.sin(epsi) * dt
        epsi_next = epsi + (v / Lf) * delta * dt
        epsi_next = normalize_angle(epsi_next)
        
        # 状態更新
        x, y, psi, v = x_next, y_next, psi_next, v_next
        cte, epsi = cte_next, epsi_next
    
    return np.array([x, y, psi, v, cte, epsi])


def write_data_to_csv(data_list, filename_prefix='important_data'):
    current_date = datetime.now().strftime('%Y-%m-%d')
    filename = f"{filename_prefix}_{current_date}.csv"
    fieldnames = ['Fuel Consumption', 'Current Velocity', 'Estimated Velocity','Acceleration']
    file_exists = os.path.isfile(filename)

    with open(filename, mode='a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists or os.stat(filename).st_size == 0:
            writer.writeheader()
        writer.writerows(data_list)

    print(f"Data has been appended to {filename}")


# Defining the Agent class to control a vehicle's behavior in the simulation
class MPCAgent(object):
    
    # Constructor for initializing the agent
    def __init__(self, ego_vehicle, config):
        
        mpc_config = config.get('MPCController', {})
        weights = mpc_config.get('weights', {})
        fuel_coeffs = mpc_config.get('fuel_consumption', {})
        latency_config = mpc_config.get('latency', {})

        # Initialize parameters
        self.N = mpc_config.get('N', 10)
        self.dt = mpc_config.get('dt', 0.1)
        self.ref_v = mpc_config.get('ref_v',0)  # Default to 40 km/h if not specified
        self.current_traffic_light_id = 13
        self.a_max = mpc_config.get('a_max', 3.0)
        steer_max_deg = mpc_config.get('steer_max_deg', 25)
        self.steer_max = np.deg2rad(steer_max_deg)  # Convert to radians
        self.v_max = mpc_config.get('v_max', 14.0)
        self.previous_velocity = 0
        self.last_ref_update_time = 0.0
        
        # レイテンシ補償設定
        self.enable_latency_compensation = latency_config.get('enable_compensation', False)
        self.latency_sec = latency_config.get('latency_sec', 0.8)
        self.enable_adaptive_latency = latency_config.get('enable_adaptive', False)
        
        # アダプティブレイテンシ測定用
        self.latency_measurements = []
        self.max_latency_samples = 50
        self.last_mpc_send_time = None

        
        # Cost function weights
        self.weight_cte = weights.get('cte', 1.0)
        self.weight_epsi = weights.get('epsi', 1.0)
        self.weight_v = weights.get('v', 2.0)
        self.weight_delta = weights.get('delta', 2.0)
        self.weight_a = weights.get('a', 5.0)
        self.weight_diff_delta = weights.get('diff_delta', 10.0)
        self.weight_diff_a = weights.get('diff_a', 1.0)
        self.weight_fuel = weights.get('fuel', 1000.0)

        # Fuel consumption coefficients
        self.b0 = fuel_coeffs.get('b0', 0.0)
        self.b1 = fuel_coeffs.get('b1', 0.0)
        self.b2 = fuel_coeffs.get('b2', 0.0)
        self.b3 = fuel_coeffs.get('b3', 0.0)
        self.c0 = fuel_coeffs.get('c0', 0.0)
        self.c1 = fuel_coeffs.get('c1', 0.0)
        self.c2 = fuel_coeffs.get('c2', 0.0)
        self.F_idle = fuel_coeffs.get('F_idle', 0.0)

        self.prev_delta = 0.0  # Previous steering angle (radians)
        self.prev_a = 0  
        
        self.vehicle = ego_vehicle
        # Reference to the traffic light manager for controlling lights
        self.ref_v = 6
        
        # Initialize the EGO model
        self.ego_model = EgoModel(self.dt)
        # Initializing the MPC controller
        self.mpc_controller = MPCController(self.ego_model,mpc_config,weights,fuel_coeffs)
        
        # 車両パラメータ（レイテンシ予測用、mpc_controller.pyと同期）
        self.vehicle_params = {
            'Lf': 2.0,  # MPCコントローラと同じ値
            'Cd': 0.3,  # C_D in mpc_controller.py
            'rho_a': 1.225,  # rho_a in mpc_controller.py
            'Av': 2.2,  # A_v in mpc_controller.py
            'Mh': 1500.0,  # M_h in mpc_controller.py
            'mu': 0.01,  # Rolling resistance coefficient
            'g': 9.81  # Gravitational acceleration
        }
        
        # Read waypoints from CSV. For Standalone mode, seperate calling.
        # Read waypoints from CSV. For Standalone mode, seperate calling.
        self.waypoints = utils.read_waypoints_from_csv("config/route.csv")
        self.waypoint_locations = [carla.Location(x=wp['x'], y=wp['y'], z=wp['z']) for wp in self.waypoints]
        self.current_wp_idx = 0  # Initialize current waypoint index
        
        # ログ用CSVファイル初期化
        if self.enable_latency_compensation:
            self._initialize_latency_log()

    def _initialize_latency_log(self):
        """ログファイル初期化"""
        current_date = datetime.now().strftime('%Y-%m-%d')
        self.latency_log_filename = f"latency_compensation_log_{current_date}.csv"
        fieldnames = ['timestamp', 'current_x', 'current_y', 'current_psi', 'current_v', 
                      'predicted_x', 'predicted_y', 'predicted_psi', 'predicted_v',
                      'current_cte', 'predicted_cte', 'current_epsi', 'predicted_epsi',
                      'latency_used', 'adaptive_latency']
        
        file_exists = os.path.isfile(self.latency_log_filename)
        if not file_exists or os.stat(self.latency_log_filename).st_size == 0:
            with open(self.latency_log_filename, mode='w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
        
        print(f"[MPC Agent] Latency compensation log initialized: {self.latency_log_filename}")
    
    def _log_latency_compensation(self, current_state, predicted_state, latency_used, adaptive_latency=None):
        """レイテンシ補償ログ記録"""
        log_data = {
            'timestamp': time.time(),
            'current_x': current_state[0],
            'current_y': current_state[1],
            'current_psi': current_state[2],
            'current_v': current_state[3],
            'predicted_x': predicted_state[0],
            'predicted_y': predicted_state[1],
            'predicted_psi': predicted_state[2],
            'predicted_v': predicted_state[3],
            'current_cte': current_state[4],
            'predicted_cte': predicted_state[4],
            'current_epsi': current_state[5],
            'predicted_epsi': predicted_state[5],
            'latency_used': latency_used,
            'adaptive_latency': adaptive_latency if adaptive_latency is not None else 'N/A'
        }
        
        with open(self.latency_log_filename, mode='a', newline='') as csvfile:
            fieldnames = log_data.keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow(log_data)
    
    def _update_adaptive_latency(self):
        """アダプティブレイテンシ測定（タイムスタンプベース）"""
        if self.last_mpc_send_time is not None:
            current_time = time.time()
            measured_latency = current_time - self.last_mpc_send_time
            
            self.latency_measurements.append(measured_latency)
            if len(self.latency_measurements) > self.max_latency_samples:
                self.latency_measurements.pop(0)
            
            # 移動平均
            adaptive_latency = np.mean(self.latency_measurements)
            return adaptive_latency
        return self.latency_sec
    
    def get_vehicle_state(self):
        """
        Retrieves the vehicle state information from the CARLA vehicle actor.

        :param vehicle: The CARLA vehicle actor
        :param cte: Cross-track error (optional, if computed externally)
        :param epsi: Orientation error (optional, if computed externally)
        :return: A dictionary containing vehicle state information
        """
        # Get vehicle location and rotation
        vehicle_location = self.vehicle.get_location()
        vehicle_rotation = self.vehicle.get_transform().rotation

        # Position in world coordinates
        x = vehicle_location.x
        y = vehicle_location.y

        # Orientation (psi) in radians (yaw in degrees, converted to radians)
        psi = np.deg2rad(vehicle_rotation.yaw)

        # Velocity vector and speed calculation
        self.velocity_vector = self.vehicle.get_velocity()
        self.speed = math.sqrt(self.velocity_vector.x**2 + self.velocity_vector.y**2) # speed in m/s
        #self.speed_kmh = self.speed * 3.6  # convert to km/h if needed

        # Get control inputs
        control = self.vehicle.get_control()
        throttle = control.throttle
        steer = control.steer
        brake = control.brake

        # Create the vehicle state dictionary
        vehicle_state = {
            'x': x,
            'y': y,
            'psi': psi,
            'v': self.speed,
            'speed': self.speed,
            'throttle': throttle,
            'steer': steer,
            'brake': brake,
        }

        return vehicle_state
 

    def on_tick(self,ref_v,stop_location):
        self.ref_v = ref_v # From the traffic light manager
        print("[MPC AGENT] Updated Reference Velocity:", self.ref_v)
        velocity_vector = self.vehicle.get_velocity()
        current_velocity = math.sqrt(velocity_vector.x**2 + velocity_vector.y**2)  # Speed in m/s
        vehicle_location = self.vehicle.get_location()
        vehicle_x = vehicle_location.x
        vehicle_y = vehicle_location.y

        # Acceleration
        calculated_acceleration = (current_velocity - self.previous_velocity) / self.dt
        self.previous_velocity = current_velocity

        # try:
        #     front_vehicle_status = self.traffic_manager.get_front_vehicle_status(self.vehicle)
        #     print(f"Front Vehicle Distance: {front_vehicle_status['distance']:.2f}, Speed: {front_vehicle_status['speed']:.2f} km/h")
        # except TypeError:
        #     front_vehicle_status = {'distance': 11, 'speed': 0.0}
        #     print("No front vehicle detected.")

        vehicle_transform = self.vehicle.get_transform()
        vehicle_rotation = vehicle_transform.rotation
        vehicle_psi = math.radians(vehicle_rotation.yaw)
       
        # Extract and transform waypoints
        waypoints_coords, self.current_wp_idx = get_waypoints(
            self.waypoints, 
            self.N, 
            vehicle_x, 
            vehicle_y, 
            vehicle_psi, 
            self.current_wp_idx
        )

        # Check if waypoints_coords is not empty
        # if self.current_wp_idx == 1000:
        #     print("No waypoints available for control computation.")
        #     control = carla.VehicleControl()
        #     control.steer = 0.0
        #     control.throttle = 0.0
        #     control.brake = 1.0  # Full brake to stop the vehicle
        #     self.vehicle.apply_control(control)
        #     return 0, True # Exit the function early

        if stop_location:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)
            print("[MPC Agent] Red light ahead, stopping at stop line.")
            return 0, False

        x_vals = [coord[0] for coord in waypoints_coords]
        y_vals = [coord[1] for coord in waypoints_coords]

        # Polynomial fit of the waypoints, polyfit returns highest power first.
        coeffs = np.polyfit(x_vals, y_vals, 3)
        coeffs = coeffs[::-1] 

        # Calculate the CTE
        cte = np.polyval(coeffs, 0) - 0  # y = 0 in vehicle coordinates

        # Calculate the orientation error
        epsi = - np.arctan(coeffs[1])

        # Predicted state after latency dt
        pred_x = current_velocity * self.dt  # Forward movement along x-axis
        pred_y = 0  # No lateral movement in local frame prediction
        pred_psi = normalize_angle((current_velocity * self.prev_delta / (self.ego_model.front_wb + self.ego_model.rear_wb)) * self.dt)
        pred_v = current_velocity + calculated_acceleration * self.dt
        pred_cte = cte + current_velocity * np.sin(epsi) * self.dt
        pred_epsi = normalize_angle(epsi + pred_psi)

        # 初期状態（単一ステップ予測）
        current_state = np.array([pred_x, pred_y, pred_psi, pred_v, pred_cte, pred_epsi])
        
        # レイテンシ補償適用
        if self.enable_latency_compensation:
            # アダプティブレイテンシ測定
            adaptive_latency = None
            if self.enable_adaptive_latency:
                adaptive_latency = self._update_adaptive_latency()
                latency_to_use = adaptive_latency
                print(f"[MPC Agent] Adaptive latency: {adaptive_latency:.3f}s")
            else:
                latency_to_use = self.latency_sec
                print(f"[MPC Agent] Using fixed latency: {latency_to_use}s")
            
            # レイテンシを考慮した状態予測
            prev_control = [self.prev_delta, self.prev_a]
            predicted_state = predict_state_with_latency(
                current_state, 
                prev_control, 
                self.dt, 
                latency_to_use, 
                self.vehicle_params
            )
            
            # ログ記録
            self._log_latency_compensation(current_state, predicted_state, latency_to_use, adaptive_latency)
            
            # 予測状態をMPCに渡す
            mpc_state = predicted_state
            print(f"[MPC Agent] Latency compensation applied: {latency_to_use}s ahead")
            print(f"[MPC Agent] Current state: x={current_state[0]:.2f}, v={current_state[3]:.2f}")
            print(f"[MPC Agent] Predicted state: x={predicted_state[0]:.2f}, v={predicted_state[3]:.2f}")
        else:
            mpc_state = current_state
            print("[MPC Agent] Latency compensation disabled")
        
        # MPC送信時刻記録（アダプティブレイテンシ用）
        if self.enable_adaptive_latency:
            self.last_mpc_send_time = time.time()
        
        print("Min Acceleration: ", min(calculated_acceleration, self.prev_a))
        # Solve MPC to obtain the optimal delta and acceleration
        optimal_delta, optimal_a, mpc_x, mpc_y = self.mpc_controller.solve(mpc_state, coeffs, self.prev_delta, self.prev_a, self.ref_v)

        # Update for next timestep
        self.prev_delta = optimal_delta
        print("Optimal Delta: ", optimal_delta)
        self.prev_a = optimal_a[0]

        # Map acceleration to throttle and brake
        throttle_cmd, brake_cmd = map_acceleration_to_throttle_brake(optimal_a[0], 
                                                                     self.mpc_controller.a_max, 
                                                                     -self.mpc_controller.a_max, 
                                                                     should_brake=False)
        
        # Compute the steering command
        steering_cmd = float(optimal_delta / math.radians(25))  # Normalize steering
        # Create and inject the control command
        control = carla.VehicleControl()
        control.steer = np.clip(steering_cmd, -1.0, 1.0)  # Clamp steering to [-1, 1]
        control.throttle = np.clip(throttle_cmd, 0.0, 0.6)  # Clamp throttle to [0, 1]
        control.brake = np.clip(brake_cmd, 0.0, 0.28)  # Clamp brake to [0, 1]
        print("[MPC Agent] Throttle Command: ", control.throttle)
        print("[MPC Agent] Brake Command: ", control.brake)
        self.vehicle.apply_control(control)
        return optimal_a[0], False, self.ref_v

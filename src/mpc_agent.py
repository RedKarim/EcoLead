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
# from mpc_controller import MPCController # importing the MPC controller - DISABLED for distributed MPC
import math
from ego_model import EgoModel
import csv
from datetime import datetime
import utils
import yaml
import json
import time
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# MQTT client for distributed MPC
import paho.mqtt.client as mqtt


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
        # DISTRIBUTED MPC: Initialize MQTT instead of local MPC controller
        self.setup_distributed_mpc()
        # Read waypoints from CSV. For Standalone mode, seperate calling.
        # Read waypoints from CSV. For Standalone mode, seperate calling.
        self.waypoints = utils.read_waypoints_from_csv("config/route.csv")
        self.waypoint_locations = [carla.Location(x=wp['x'], y=wp['y'], z=wp['z']) for wp in self.waypoints]
        self.current_wp_idx = 0  # Initialize current waypoint index
        
        # MQTT Performance Metrics
        self.setup_mqtt_metrics()

    def setup_mqtt_metrics(self):
        """Setup MQTT performance metrics tracking"""
        # Packet tracking
        self.sent_packets = {}  # {packet_id: timestamp}
        self.packet_counter = 0
        self.metrics_lock = threading.Lock()
        
        # Performance metrics storage
        self.latency_history = deque(maxlen=1000)  # Keep last 1000 measurements
        self.packet_loss_history = deque(maxlen=1000)
        self.time_history = deque(maxlen=1000)
        
        # Current metrics
        self.current_latency = 0.0
        self.current_packet_loss = 0.0
        self.total_sent = 0
        self.total_received = 0
        self.total_lost = 0
        
        # CSV logging
        self.metrics_csv_file = f"mqtt_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_csv_file()
        
        # Real-time plotting
        self.setup_realtime_plot()
        
        # Cleanup thread for old packets (consider lost after 5 seconds)
        self.cleanup_thread = threading.Thread(target=self.cleanup_old_packets, daemon=True)
        self.cleanup_thread.start()
        
        print("[MQTT Metrics] Performance tracking initialized")

    def init_csv_file(self):
        """Initialize CSV file for metrics logging"""
        try:
            with open(self.metrics_csv_file, 'w', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'packet_id', 'latency_ms', 'packet_loss_percent',
                    'total_sent', 'total_received', 'total_lost'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
            print(f"[MQTT Metrics] CSV file initialized: {self.metrics_csv_file}")
        except Exception as e:
            print(f"[MQTT Metrics] Error initializing CSV: {e}")

    def setup_realtime_plot(self):
        """Setup real-time matplotlib visualization"""
        try:
            # Create figure with subplots
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
            self.fig.suptitle('MQTT Performance Metrics - Real Time')
            
            # Latency plot
            self.ax1.set_title('Round Trip Latency (ms)')
            self.ax1.set_ylabel('Latency (ms)')
            self.ax1.grid(True)
            self.ax1.set_ylim(0, 200)  # Adjust based on expected latency
            
            # Packet loss plot
            self.ax2.set_title('Packet Loss (%)')
            self.ax2.set_xlabel('Time (s)')
            self.ax2.set_ylabel('Packet Loss (%)')
            self.ax2.grid(True)
            self.ax2.set_ylim(0, 100)
            
            # Initialize empty line plots
            self.latency_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='Latency')
            self.packet_loss_line, = self.ax2.plot([], [], 'r-', linewidth=2, label='Packet Loss')
            
            self.ax1.legend()
            self.ax2.legend()
            
            # Start animation
            self.anim = animation.FuncAnimation(
                self.fig, self.update_plot, interval=100, blit=False
            )
            
            # Show plot in non-blocking mode
            plt.ion()
            plt.show(block=False)
            
            print("[MQTT Metrics] Real-time plot initialized")
        except Exception as e:
            print(f"[MQTT Metrics] Error setting up plot: {e}")
            self.fig = None

    def update_plot(self, frame):
        """Update the real-time plot"""
        try:
            with self.metrics_lock:
                if len(self.time_history) > 0:
                    # Update latency plot
                    self.latency_line.set_data(list(self.time_history), list(self.latency_history))
                    self.ax1.set_xlim(min(self.time_history), max(self.time_history))
                    
                    # Update packet loss plot
                    self.packet_loss_line.set_data(list(self.time_history), list(self.packet_loss_history))
                    self.ax2.set_xlim(min(self.time_history), max(self.time_history))
                    
                    # Update titles with current values
                    self.ax1.set_title(f'Round Trip Latency: {self.current_latency:.1f} ms (Avg: {np.mean(list(self.latency_history)):.1f} ms)')
                    self.ax2.set_title(f'Packet Loss: {self.current_packet_loss:.1f}% (Total: {self.total_sent} sent, {self.total_lost} lost)')
                    
            return self.latency_line, self.packet_loss_line
        except Exception as e:
            print(f"[MQTT Metrics] Plot update error: {e}")
            return self.latency_line, self.packet_loss_line

    def cleanup_old_packets(self):
        """Background thread to clean up old packets (consider them lost)"""
        while True:
            try:
                current_time = time.time()
                timeout = 5.0  # 5 seconds timeout
                
                with self.metrics_lock:
                    expired_packets = []
                    for packet_id, timestamp in self.sent_packets.items():
                        if current_time - timestamp > timeout:
                            expired_packets.append(packet_id)
                    
                    # Remove expired packets and count as lost
                    for packet_id in expired_packets:
                        del self.sent_packets[packet_id]
                        self.total_lost += 1
                        self.update_packet_loss_metrics()
                
                time.sleep(1.0)  # Check every second
            except Exception as e:
                print(f"[MQTT Metrics] Cleanup thread error: {e}")
                time.sleep(1.0)

    def update_packet_loss_metrics(self):
        """Update packet loss percentage"""
        if self.total_sent > 0:
            self.current_packet_loss = (self.total_lost / self.total_sent) * 100.0
        else:
            self.current_packet_loss = 0.0

    def log_metrics_to_csv(self, packet_id, latency_ms):
        """Log metrics to CSV file"""
        try:
            with open(self.metrics_csv_file, 'a', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'packet_id', 'latency_ms', 'packet_loss_percent',
                    'total_sent', 'total_received', 'total_lost'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow({
                    'timestamp': datetime.now().isoformat(),
                    'packet_id': packet_id,
                    'latency_ms': latency_ms,
                    'packet_loss_percent': self.current_packet_loss,
                    'total_sent': self.total_sent,
                    'total_received': self.total_received,
                    'total_lost': self.total_lost
                })
        except Exception as e:
            print(f"[MQTT Metrics] CSV logging error: {e}")

    def setup_distributed_mpc(self, mqtt_broker='10.21.89.202', mqtt_port=1883):
        """Setup MQTT for distributed MPC communication"""
        try:
            if hasattr(mqtt, 'CallbackAPIVersion'):
                self.mqtt_client = mqtt.Client(client_id="mpc_agent", callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
            else:
                self.mqtt_client = mqtt.Client(client_id="mpc_agent")
        except Exception as e:
            print(f"[MQTT] Client creation error: {e}")
            self.mqtt_client = mqtt.Client(client_id="mpc_agent")
        
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # MPC result data
        self.latest_mpc_result = None
        self.result_lock = threading.Lock()
        self.result_received = False
        
        try:
            self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            self.mqtt_client.loop_start()
            print(f"[MQTT] Connected to distributed MPC controller at {mqtt_broker}:{mqtt_port}")
        except Exception as e:
            print(f"[MQTT] Connection failed: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("[MQTT] Successfully connected to broker")
            # Subscribe to MPC results from distributed controller
            client.subscribe("mpc/result")
            print("[MQTT] Subscribed to mpc/result")
        else:
            print(f"[MQTT] Failed to connect, return code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message received callback"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            
            if topic == "mpc/result":
                # Extract packet tracking info
                packet_id = payload.get('packet_id', None)
                receive_time = time.time()
                
                # Calculate latency if packet ID exists
                if packet_id is not None:
                    with self.metrics_lock:
                        if packet_id in self.sent_packets:
                            # Calculate round-trip latency
                            send_time = self.sent_packets[packet_id]
                            latency_ms = (receive_time - send_time) * 1000.0
                            
                            # Update metrics
                            self.total_received += 1
                            self.current_latency = latency_ms
                            
                            # Store in history for plotting
                            current_sim_time = receive_time - self.simulation_start_time if hasattr(self, 'simulation_start_time') else receive_time
                            self.latency_history.append(latency_ms)
                            self.packet_loss_history.append(self.current_packet_loss)
                            self.time_history.append(current_sim_time)
                            
                            # Remove from sent packets (received successfully)
                            del self.sent_packets[packet_id]
                            
                            # Update packet loss metrics
                            self.update_packet_loss_metrics()
                            
                            # Log to CSV
                            self.log_metrics_to_csv(packet_id, latency_ms)
                            
                            print(f"[MQTT Metrics] Packet {packet_id}: RTT = {latency_ms:.1f}ms, Loss = {self.current_packet_loss:.1f}%")
                        else:
                            print(f"[MQTT Metrics] Warning: Received unknown packet ID {packet_id}")
                
                with self.result_lock:
                    self.latest_mpc_result = payload
                    self.result_received = True
                print(f"[DEBUG] Received MPC result")
                
        except Exception as e:
            print(f"[MQTT] Message processing error: {e}")

    def send_mpc_request(self, state, coeffs, prev_delta, prev_a, ref_v):
        """Send MPC request to distributed controller with packet tracking"""
        try:
            # Generate unique packet ID
            with self.metrics_lock:
                self.packet_counter += 1
                packet_id = self.packet_counter
                send_time = time.time()
                
                # Store packet for tracking
                self.sent_packets[packet_id] = send_time
                self.total_sent += 1
                
                # Initialize simulation start time if not set
                if not hasattr(self, 'simulation_start_time'):
                    self.simulation_start_time = send_time
            
            request_msg = {
                'packet_id': packet_id,  # Add packet ID for tracking
                'timestamp': send_time,
                'state': [float(s) for s in state],
                'coeffs': [float(c) for c in coeffs],
                'prev_delta': float(prev_delta),
                'prev_a': float(prev_a),
                'ref_v': float(ref_v)
            }
            
            self.mqtt_client.publish("mpc/request", json.dumps(request_msg), qos=1)
            print(f"[DEBUG] Sent MPC request (Packet ID: {packet_id})")
        except Exception as e:
            print(f"[MQTT] Failed to send request: {e}")

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

        # Define the initial state for MPC
        current_state = np.array([pred_x, pred_y, pred_psi, pred_v, pred_cte, pred_epsi])
        print("Min Acceleration: ", min(calculated_acceleration, self.prev_a))
        
        # DISTRIBUTED MPC: Send request to Mac and wait for result
        self.send_mpc_request(current_state, coeffs, self.prev_delta, self.prev_a, self.ref_v)
        
        # Wait for MPC result with timeout
        timeout = 0.5  # 500ms timeout
        start_time = time.time()
        while not self.result_received and (time.time() - start_time) < timeout:
            time.sleep(0.001)  # 1ms sleep
        
        if self.result_received:
            with self.result_lock:
                result = self.latest_mpc_result.copy()
                self.result_received = False
            
            optimal_delta = result['optimal_delta']
            optimal_a = result['optimal_a']
            mpc_x = result['mpc_x']
            mpc_y = result['mpc_y']
            print(f"[MPC] Received distributed result: delta={optimal_delta:.3f}, a[0]={optimal_a[0]:.3f}")
        else:
            print("[MPC] Timeout waiting for distributed MPC result, using fallback")
            # Fallback to safe control
            optimal_delta = 0.0
            optimal_a = [0.0] * (self.N - 1)
            mpc_x = [0.0] * self.N
            mpc_y = [0.0] * self.N

        # Update for next timestep
        self.prev_delta = optimal_delta
        print("Optimal Delta: ", optimal_delta)
        self.prev_a = optimal_a[0]

        # Map acceleration to throttle and brake
        throttle_cmd, brake_cmd = map_acceleration_to_throttle_brake(optimal_a[0], 
                                                                     self.a_max, 
                                                                     -self.a_max, 
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

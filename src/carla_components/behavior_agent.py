# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is a modified version of a CARLA API script.
#
# Copyright (c) 2025, FZI Forschungszentrum Informatik
# Copyright (c) 2018-2020, Computer Vision Center (CVC), Universitat Autonoma de Barcelona
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notices, this list of conditions,
#    and the following disclaimers.
#
# 2. Redistributions in binary form must reproduce the above copyright notices, this list of
#    conditions, and the following disclaimers in the documentation and/or other materials
#    provided with the distribution.
#
# 3. Neither the name of the copyright holders nor the names of its contributors
#    may be used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# -- END LICENSE BLOCK ------------------------------------------------


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import math
import numpy as np
import carla
from carla_components.basic_agent import BasicAgent
from carla_components.local_planner import RoadOption
from carla_components.behavior_types import Cautious, Aggressive, Normal

from carla_components.misc import get_speed, positive, is_within_distance, compute_distance

class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(self, vehicle, behavior='normal'):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param ignore_traffic_light: boolean to ignore any traffic light
            :param behavior: type of agent to apply
        """

        super(BehaviorAgent, self).__init__(vehicle)
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0
        self._speed_limit = 50/3.6
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5
        self.previous_target_speed = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.start_time = None
        self.filtered_derivative = 0.0

        # Parameters for agent behavior
        if behavior == 'cautious':
            self._behavior = Cautious()

        elif behavior == 'normal':
            self._behavior = Normal()

        elif behavior == 'aggressive':
            self._behavior = Aggressive()

    def _update_information(self,ego_vehicle_speed=None):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
        """
        self._speed = get_speed(self._vehicle)
        # Dynamically update speed limit from ego vehicle or use the vehicle's own limit
        if ego_vehicle_speed is not None:
            self._speed_limit = ego_vehicle_speed
            print(f"[Behaviour Agent] Speed Limit is set to the ego vehicle speed {self._speed_limit}")
        else:
            self._speed_limit = self._vehicle.get_speed_limit()

        self._local_planner.set_speed(self._speed_limit)
        self._direction = self._local_planner.target_road_option
        if self._direction is None:
            self._direction = RoadOption.LANEFOLLOW

        self._look_ahead_steps = int((self._speed_limit) / 10)

        self._incoming_waypoint, self._incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self._look_ahead_steps)
        if self._incoming_direction is None:
            self._incoming_direction = RoadOption.LANEFOLLOW

    def traffic_light_manager(self):
        """
        This method is in charge of behaviors for red lights.
        """
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter("*traffic_light*")
        affected, _ = self._affected_by_traffic_light(lights_list)

        return affected

    def _tailgating(self, waypoint, vehicle_list):
        """
        This method is in charge of tailgating behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max(
            self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, low_angle_th=160)
        if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
            if (right_turn == carla.LaneChange.Right or right_turn ==
                    carla.LaneChange.Both) and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the right!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(end_waypoint.transform.location,
                                         right_wpt.transform.location)
            elif left_turn == carla.LaneChange.Left and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
                if not new_vehicle_state:
                    print("Tailgating, moving to the left!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(end_waypoint.transform.location,
                                         left_wpt.transform.location)

    def collision_and_car_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        and managing possible tailgating chances.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a vehicle nearby, False if not
            :return vehicle: nearby vehicle
            :return distance: distance to nearby vehicle
        """

        vehicle_list = self._world.get_actors().filter("*vehicle*")
        def dist(v): return v.get_location().distance(waypoint.transform.location)
        vehicle_list = [v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id]

        if self._direction == RoadOption.CHANGELANELEFT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
        elif self._direction == RoadOption.CHANGELANERIGHT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
        else:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=30)

            # Check for tailgating
            if not vehicle_state and self._direction == RoadOption.LANEFOLLOW \
                    and not waypoint.is_junction and self._speed > 10 \
                    and self._behavior.tailgate_counter == 0:
                self._tailgating(waypoint, vehicle_list)

        return vehicle_state, vehicle, distance

    def pedestrian_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any pedestrian.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a walker nearby, False if not
            :return vehicle: nearby walker
            :return distance: distance to nearby walker
        """

        walker_list = self._world.get_actors().filter("*walker.pedestrian*")
        def dist(w): return w.get_location().distance(waypoint.transform.location)
        walker_list = [w for w in walker_list if dist(w) < 10]

        if self._direction == RoadOption.CHANGELANELEFT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=-1)
        elif self._direction == RoadOption.CHANGELANERIGHT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=1)
        else:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=60)

        return walker_state, walker, distance
    
    def low_pass_filter(self,current_value, previous_value, alpha=0.8):
        if not isinstance(current_value, (int, float)) or not isinstance(previous_value, (int, float)):
            raise TypeError(f"low_pass_filter expected numeric inputs, got {type(current_value)} and {type(previous_value)}")
        return alpha * current_value + (1 - alpha) * previous_value
    # def car_following_manager(self, vehicle, distance, debug=True):
    #     """
    #     Manages car-following behavior with clear separation of different scenarios.
        
    #     :param vehicle: Leading vehicle to follow
    #     :param distance: Distance to front vehicle in meters
    #     :param debug: Debugging flag
    #     :return: carla.VehicleControl instance
    #     """
    #     # Configuration parameters (adjust these in your behavior class)
    #     SAFETY_TIME = self._behavior.safety_time  # Time gap in seconds
    #     MAX_SPEED = self._behavior.max_speed       # km/h
    #     MIN_SPEED = self._min_speed                # km/h
    #     SPEED_LIMIT = self._speed_limit            # km/h
    #     DECELERATION_FACTOR = 0.2                  # Aggressiveness of braking
        
    #     # Current state
    #     ego_speed = self._speed  # km/h
    #     lead_speed = get_speed(vehicle)  # km/h
    #     delta_v = (ego_speed - lead_speed) / 3.6  # m/s
        
    #     # Calculate time-to-collision (TTC)
    #     ttc = distance / delta_v if delta_v > 0 else float('inf')
        
    #     # Calculate desired safety distance
    #     desired_distance = (ego_speed / 3.6) * 1 # meters

    #     # Decision making ----------------------------------------------------------
    #     # Case 1: Dangerous closing speed
    #     if ttc <= SAFETY_TIME * 2 and delta_v > 0:
    #         if debug: print("[CFM] Emergency deceleration")
    #         target_speed = max(MIN_SPEED, lead_speed - (lead_speed * DECELERATION_FACTOR))

    #     # Case 2: Maintaining safe following distance
    #     elif SAFETY_TIME * 2 < ttc <= 10 :
    #         if debug: print("[CFM] Speed matching")
    #         target_speed = lead_speed

    #     # Case 3: Far ahead, resume normal spee
    #     elif ttc > 15 or distance > desired_distance * 1.5:
    #         speed_diff = ((distance - desired_distance) / SAFETY_TIME) * (self._speed_limit/self._behavior.max_speed)
    #         print(f"[CFM] Increase Speed {speed_diff:.1f}km/h")
    #         target_speed = min(MAX_SPEED, SPEED_LIMIT+speed_diff)
    #         if debug: print(f"[CFM] Target Speed: {target_speed:.1f}km/h")

    #     # Case 4: Adjusting to moving away vehicle
    #     elif delta_v <= 0:  # Lead vehicle is faster or same speed
    #         if distance < desired_distance:
    #             if debug: print("[CFM] Increasing gap")
    #             target_speed = max(MIN_SPEED, ego_speed - (ego_speed * 0.1))
    #         else:
    #             target_speed = min(MAX_SPEED, SPEED_LIMIT)

    #     # Case 5: Default behavior (smooth adjustment)
    #     else:
    #         if debug: print("[CFM] Progressive adjustment")
    #         speed_diff = (distance - desired_distance) / SAFETY_TIME * 3.6
    #         target_speed = ego_speed + speed_diff

    #     # Speed limits enforcement -------------------------------------------------
    #     target_speed = max(MIN_SPEED, target_speed)
        
    #     # Control execution --------------------------------------------------------
    #     self._local_planner.set_speed(target_speed)
    #     control = self._local_planner.run_step(debug=debug)

    #     if debug:
    #         print(f"[CFM] Target: {target_speed:.1f}km/h | "
    #             f"Lead: {lead_speed}km/h | TTC: {ttc:.1f}s | "
    #             f"Dist: {distance:.1f}m/{desired_distance:.1f}m")

    #     return control

    def car_following_manager_ramp(self, vehicle, distance, debug=True):
        """
        An improved finite-state car-following manager that:
        1) Uses time-gap-based distance.
        2) Has a comfort band around that distance.
        3) Handles emergency deceleration via TTC.
        4) Ensures 'low-speed' catch-up if the vehicle is far behind at low speeds.
        5) Optionally smooths final speed to avoid jumps.
        
        :param vehicle: The leading vehicle object.
        :param distance: Current gap (m).
        :param debug: If True, prints debug info.
        
        :return: carla.VehicleControl from local_planner.run_step()
        """
        debug_print=True
        print(f"[CFM] Car-following Manager (Ramp) activated: {distance}m, {vehicle}km/h")
        # -----------------------------
        # 1) Behavior / Config
        # -----------------------------
        T = 1 #self._behavior.safety_time / 3.2    # e.g. safety_time=3 => T=1.0
        d0 = 0                                # minimal standstill distance
        MAX_SPEED   = self._behavior.max_speed  # e.g. 50 km/h
        MIN_SPEED   = self._min_speed           # e.g. 0 km/h
        SPEED_LIMIT = self._speed_limit         # from map or platoon logic
        DECEL_FACTOR = 0.2                      # factor for emergency slowdown
        EMERGENCY_TTC = 3.0                     # seconds, triggers emergency decel
        
        # For initial state if not set
        if not hasattr(self, "previous_target_speed"):
            self.previous_target_speed = 0.0

        # -----------------------------
        # 2) Current States
        # -----------------------------
        ego_speed_kmh    = self._speed
        leader_speed_kmh = get_speed(vehicle)

        ego_speed_ms    = ego_speed_kmh / 3.6
        leader_speed_ms = leader_speed_kmh / 3.6

        delta_v = ego_speed_ms - leader_speed_ms
        epsilon = 1e-3

        if delta_v > 0:
            ttc = distance / (delta_v + epsilon)
        else:
            ttc = float('inf')

        # Time-gap-based desired distance
        desired_distance = d0 + (leader_speed_ms * T)

        if debug_print:
            print("[CFM]--------------------------------------------")
            print(f"[CFM] Dist={distance:.1f}m | DesiredDist={desired_distance:.1f}m | "
                f"Ego={ego_speed_kmh:.1f}km/h | Leader={leader_speed_kmh:.1f}km/h | TTC={ttc:.1f}s")

        # -----------------------------
        # 3) Main Decision Logic
        # -----------------------------
        # We'll define a comfort band ±10% around desired_distance
        lower_band = 0.9 * desired_distance
        upper_band = 1.1 * desired_distance

        # Case 3.1: EMERGENCY if TTC < EMERGENCY_TTC
        if ttc < EMERGENCY_TTC:
            if debug: print("[CFM] EMERGENCY => Strong deceleration")
            target_speed = max(MIN_SPEED, leader_speed_kmh * (1 - DECEL_FACTOR))

        else:
            # Distance checks
            if distance < lower_band:
                # TOO CLOSE => reduce speed
                if debug_print: print("[CFM] TOO CLOSE => decelerate or match lower speed")
                decel_speed = ego_speed_kmh * 0.95
                target_speed = min(decel_speed, leader_speed_kmh)

            elif distance > upper_band:
                # TOO FAR => accelerate
                if debug_print: print("[CFM] TOO FAR => accelerate up to leader or small overshoot")

                gap_error = distance - desired_distance  # positive
                # Speed increase based on gap_error
                speed_increase_kmh = ((gap_error / T) * 3.6)/3
                raw_target_speed = ego_speed_kmh + speed_increase_kmh

                overshoot_margin =(SPEED_LIMIT-leader_speed_kmh)
                print(f"[CFM] overshoot_margin={overshoot_margin:.2f}km/h")
                allowed_speed = leader_speed_kmh + overshoot_margin

                target_speed = min(raw_target_speed, allowed_speed)

                # (A) Handle low-speed scenario specifically
                LOW_SPEED_THRESHOLD = 7.3   # km/h
                MIN_BUMP = 0.2             # km/h
                if SPEED_LIMIT < LOW_SPEED_THRESHOLD:
                    # If we are crawling, ensure at least +2 km/h above current
                    # to "nudge" forward if gap is large
                    min_target_for_low_speed = ego_speed_kmh + MIN_BUMP
                    # target_speed = (target_speed + min_target_for_low_speed)/2
                    target_speed = min(target_speed , min_target_for_low_speed)
                    if debug:
                        print(f"[CFM] LOW-SPEED => forced bump to {target_speed:.1f}km/h")

            else:
                # COMFORT BAND => match leader speed
                if debug_print: print("[CFM] COMFORT BAND => match leader speed")
                target_speed = leader_speed_kmh

        # -----------------------------
        # 4) Final Clamps
        # -----------------------------
        target_speed = max(MIN_SPEED, target_speed)
        target_speed = min(target_speed, MAX_SPEED, SPEED_LIMIT)

        # If you want strictly never exceed leader, comment out the overshoot usage
        # target_speed = min(target_speed, leader_speed_kmh)

        # -----------------------------
        # 5) Smooth final speed to avoid big jumps
        # -----------------------------
        alpha = 0.2  # 0 => instant change, 1 => no change from old
        old_speed = self.previous_target_speed
        smoothed_speed = alpha * old_speed + (1 - alpha) * target_speed
        self.previous_target_speed = smoothed_speed

        if debug_print:
            print(f"[CFM] raw_target={target_speed:.1f} | smoothed={smoothed_speed:.1f}\n")

        # -----------------------------
        # 6) Apply Speed
        # -----------------------------
        self._local_planner.set_speed(smoothed_speed)
        control = self._local_planner.run_step(debug=debug)
        return control
    def car_following_manager(self, vehicle, current_gap, debug=True):
        """
        Implements Adaptive CACC with Distance Control (10-15m).
        
        :param preceding_vehicle: The vehicle directly ahead.
        :param leader_vehicle: The MPC-controlled leader (V2V data).
        :param current_gap: The measured gap between ego and preceding vehicle.
        
        :return: Vehicle control command.
        """
        #1) Get current time 
        current_time = self._world.get_snapshot().timestamp.elapsed_seconds
        # If start_time has not been set, record it now
        if self.start_time is None:
            self.start_time = current_time
        # 2) Add the 8-second delay check
        dt = 0.1
        # Vehicle speeds
        v_ego = self._speed/3.6
        v_prev = get_speed(vehicle)/3.6  # Preceding vehicle speed (local sensors)
        v_leader = self._speed_limit/3.6  # MPC leader speed (V2V)
        # Calculate how many seconds have passed since we first started
        sim_time_elapsed = current_time - self.start_time
        if sim_time_elapsed < 10:
            if v_ego < 13:
                #v_target = max(v_target, 12 * (sim_time_elapsed / 5.0))  # Linear ramp to 10 m/s in 5s
                # print(f"[CFM] Initial acceleration: Preceding car: {v_prev:.1f}m/s, Ego: {v_ego:.1f}m/s")
                return self.car_following_manager_ramp(vehicle,current_gap)


        # Desired gap range (10-15m)
        min_gap, max_gap = 6, 15.0  
        s_0 = 1  # Minimum standstill gap
        T_gap = 1 # Safe time gap (s)
        if v_ego < 5:
            s_desired = max(min_gap, s_0 + v_ego * T_gap * 0.6)  # More flexibility at low speeds
        else:
            s_desired = max(min_gap, min(s_0 + v_ego * T_gap, max_gap))  # Clamp gap
        # print(f"[CFM] Desired gap: {s_desired:.1f}m | Current gap: {current_gap:.1f}m")
        # Compute gap error
        gap_error = current_gap - s_desired
        # print(f"[CFM] Gap error: {gap_error:.1f}m")

        # Adaptive CACC Gains (Based on gap error magnitude)
        K1_min, K1_max = 0.2, 0.6  
        K2_min, K2_max = 0.3, 0.7  
        abs_gap_error = abs(current_gap - s_desired)  # Compute absolute error for gains
        K1 = K1_min + (K1_max - K1_min) * (1 - np.exp(-abs_gap_error / max_gap))
        K2 = K2_min + (K2_max - K2_min) * (1 - np.exp(-abs_gap_error / max_gap))
        # print(f"[CFM] K1: {K1:.2f} | K2: {K2:.2f}")
        # Compute CACC target speed
        v_target = v_prev + K1 * gap_error + K2 * (v_leader - v_prev)
        # print(f"[CFM] Target speed: {v_target:.1f}m/s | Ego speed: {v_ego:.1f}m/s | Leader speed: {v_leader:.1f}m/s | Preceding speed: {v_prev:.1f}m/s")
        # Allow a slight overshoot before stabilizing
        v_target = max(0, min(v_target, (self._speed_limit )/3.6+ 2))  # Allow minor overshoot
        # print(f"[CFM] Target speed Clamp: {v_target:.1f}m/s ")
        # **Apply PID control with Dynamic Gains**
        # Gain scheduling for CACC PID
        if v_ego < 5:
            Kp, Ki, Kd = 0.35, 0.02, 0.5   # Aggressive Kp/Kd for low-speed precision
        elif 5 <= v_ego < 8:  
            Kp, Ki, Kd = 0.30, 0.04, 0.65  # Boost Kd to dampen mid-speed oscillations
        elif 8 <= v_ego < 12:
            Kp, Ki, Kd = 0.25, 0.05, 0.50  # Reduce Kd for smoother high-speed tracking
        else:
            Kp, Ki, Kd = 0.20, 0.05, 0.30  # Conservative gains for very high speeds

        speed_error = v_target - v_ego
        # print(f"[CFM] Speed error: {speed_error:.1f}m/s")
        # PID with Anti-Windup  
        # Calculate raw derivative
        raw_derivative = (speed_error - self.prev_error) / dt

        # Apply low-pass filter to derivative
        alpha = 0.1
        self.filtered_derivative = alpha * raw_derivative + (1 - alpha) * getattr(self, 'filtered_derivative', raw_derivative)

        self.prev_error = speed_error

        # Update speed using PID controller
        v_ego += Kp * speed_error + Ki * self.integral + Kd * self.filtered_derivative
        # print(f"[CFM] Updated speed: {v_ego:.1f}m/s")
        v_ego = max(0, min(v_ego, self._speed_limit/3.6))  # Speed clamping
        # print(f"[CFM] Updated speed: {v_ego:.1f}m/s")
        # hysteresis_threshold = 0.2  # Ignore small changes below 0.2 m/s
        # if v_ego < 8 and v_ego > 6:
        #     if abs(v_ego - self.previous_target_speed ) > hysteresis_threshold:
        #         self.previous_target_speed = v_ego
        #     else:
        #         v_ego = self.previous_target_speed 

        # Store the previous velocity for the next iteration
        self.previous_target_speed  = v_ego
        self._local_planner.set_speed(v_ego*3.6)
        control = self._local_planner.run_step(debug=debug)

        return control
    # def car_following_manager(self, leader_vehicle, distance_to_leader, debug=True):
    #     """
    #     a 'cacc-like' pd controller that uses parameters from the behavior class
    #     (cautious, normal, aggressive).

    #     :param leader_vehicle: the vehicle in front.
    #     :param distance_to_leader: gap (m) to the leader.
    #     :param dt: time step (s).
    #     :param debug: if true, prints debug info.
    #     :param behavior: an instance of cautious, normal, or aggressive
    #     :return: carla.vehiclecontrol from self._local_planner.run_step()
    #     """


    #     # --------------------------------------------------------
    #     # 2) extract relevant behavior parameters
    #     # --------------------------------------------------------
    #     # we'll interpret:
    #     #  - behavior.safety_time as the desired time headway, t
    #     #  - behavior.braking_distance as d0, the minimum gap at standstill
    #     #  - behavior.max_speed as the final clamp on speed (km/h)

    #     t     = self._behavior.safety_time/2
    #     d0    = self._behavior.braking_distance
    #     v_max = self._behavior.max_speed  # speed clamp in km/h
    #     dt   = 0.1  # time step in seconds

    #     # example pd gains for gap & speed error
    #     kp_gap = 1.2
    #     kd_spd = 0.5

    #     # we'll keep a simple acceleration limit
    #     max_accel = 8 # m/s^2
    #     max_decel = 4.0  # m/s^2

    #     # --------------------------------------------------------
    #     # 3) get current speeds (leader & ego)
    #     # --------------------------------------------------------
    #     v_leader_km_h = get_speed(leader_vehicle)  # in km/h
    #     v_ego_km_h    = self._speed                # in km/h

    #     # convert to m/s
    #     v_leader = v_leader_km_h / 3.6
    #     v_ego    = v_ego_km_h    / 3.6

    #     # ensure gap isn't negative
    #     gap = max(distance_to_leader, 0.1)

    #     # --------------------------------------------------------
    #     # 4) compute desired gap & errors
    #     # --------------------------------------------------------
    #     desired_gap = d0 + t * v_leader  # time-gap formula
    #     gap_error = gap - desired_gap
    #     speed_error = (v_leader - v_ego)

    #     # --------------------------------------------------------
    #     # 5) pd control for acceleration
    #     # --------------------------------------------------------
    #     a_raw = (kp_gap * gap_error) + (kd_spd * speed_error)

    #     # --------------------------------------------------------
    #     # 6) clip acceleration
    #     # --------------------------------------------------------
    #     if a_raw > max_accel:
    #         a_raw = max_accel
    #     elif a_raw < -max_decel:
    #         a_raw = -max_decel

    #     # --------------------------------------------------------
    #     # 7) integrate acceleration -> new speed (m/s)
    #     # --------------------------------------------------------
    #     v_new = v_ego + a_raw * dt
    #     if v_new < 0:
    #         v_new = 0
    #     v_new_km_h = v_new * 3.6

    #     # --------------------------------------------------------
    #     # 8) apply behavior-based speed clamps
    #     # --------------------------------------------------------
    #     #  a) never exceed behavior.max_speed
    #     v_new_km_h = min(v_new_km_h, v_max)

    #     #  b) if you want strict platooning, also clamp to leader speed
    #     #     (comment out if you allow overshoot)
    #     # v_new_km_h = min(v_new_km_h, v_leader_km_h)

    #     # --------------------------------------------------------
    #     # 9) pass the speed to your local_planner (pid inside)
    #     # --------------------------------------------------------
    #     self._local_planner.set_speed(v_new_km_h)
    #     control = self._local_planner.run_step(debug=debug)

    #     # debug info
    #     if debug:
    #         print(f"[cacc-pd-behavior] "
    #             f"gap={gap:.1f}/{desired_gap:.1f}, gaperr={gap_error:.1f}, "
    #             f"spderr={speed_error:.1f}, vego={v_ego_km_h:.1f}, vleader={v_leader_km_h:.1f}, "
    #             f"vtarget={v_new_km_h:.1f}")

    #     return control

    def run_step(self,vehicle,distance,distance_to_packleader,leader_velocity,leader,debug=False):
        """
        Execute one step of navigation.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        # self._update_information()
        control = None
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1
        
        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)
        ego_vehcile_speed = get_speed(self._vehicle)
        print("[BEHAVIOUR AGENT] ego velocity is:",ego_vehcile_speed)
        print(f"[BEHAVIOUR AGENT] Leader {leader_velocity}")
        if leader:
            print("[BEHAVIOUR AGENT] Leader is in the run step")
            self._local_planner.set_speed(leader_velocity*3.6)
            control = self._local_planner.run_step(debug=debug)
            return control
        # 1: Red lights and stops behavior
        if self.traffic_light_manager():
            # print("[BEHAVIOUR AGENT] Traffic Light_manager emergency stop")
            return self.emergency_stop()

        # 2.2: Car following behaviors
        # print(f"My Distance:{distance}")
        vehicle_state, col_vehicle, col_distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)
        # print(f"[BEHAVIOUR AGENT] Collision Distance:{col_distance}")
        # print(f"Vehicle State:{vehicle_state}")
        if distance > 0 and col_distance >= -1:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            # print(f"Distance: {distance},braking_distance: {self._behavior.braking_distance}")
            # distance = distance - max(
            #     vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
            #         self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            if col_distance < self._behavior.braking_distance and col_distance>0:
                print("[BEHAVIOUR AGENT] Emergency Stop")
                return self.emergency_stop()
            else:
                # print("Car Followingteyim")
                control = self.car_following_manager(vehicle, distance,debug=True)

        # 4: Normal behavior
        else:
            print("Normal Behaviour in Run Step")
            target_speed = min([
                self._behavior.max_speed,
                self._speed_limit - self._behavior.speed_lim_dist])
            print(f"Target_speed: {target_speed}")
            self._local_planner.set_speed(target_speed)
        control = self._local_planner.run_step(debug=debug)

        return control

    def emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control
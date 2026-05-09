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
# !\ traffic_light_manager.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import carla
from platoon_manager import PlatoonManager
import math

class TrafficLightManager:
    def __init__(self, client, traffic_lights_config, waypoint_locations):
        self.client = client
        self.world = client.get_world()
        self.traffic_lights_config = traffic_lights_config
        self.traffic_lights = {}
        self.waypoints = [wp.transform.location if isinstance(wp, carla.Waypoint) else wp for wp in waypoint_locations]
        self.fixed_delta_seconds = self.world.get_settings().fixed_delta_seconds
        self.v_min = 1
        self.v_max = 50/3.6
        self.ref_v = self.v_max
        self.init_delay = 5 # Initial delay before starting the reference velocity calculation
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds  # Record the start time
        self.platoon_manager = None  # Will be set externally
        self.split_counter = 1 # Counter to track the number of splits
        self.corridor_id = 1  # Default corridor ID
        self.csv_files = {}
        self.start_time = None
        self.processed_tl_ids = {}  # Dictionary keyed by platoon_id
        self.corridor_change = False
        self.pams = []

        # Deactivate other traffic lights
        self.deactivate_other_traffic_lights()

        # Initialize traffic lights with their initial states and durations
        self.initialize_traffic_lights()

    def deactivate_other_traffic_lights(self):
        """Deactivate all traffic lights except those specified in the configuration."""
        all_traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        for tl in all_traffic_lights:
            if tl.id not in self.traffic_lights_config:
                tl.set_state(carla.TrafficLightState.Off)
                tl.freeze(True)
                print(f"Deactivated Traffic Light ID {tl.id} set to Off and frozen.")

    def initialize_traffic_lights(self):
        """Initialize specified traffic lights with their initial state and phase durations."""
        all_traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        current_tick = self.world.get_snapshot().frame  # Get the current tick at initialization
        custom_order = [13, 11, 20]  # Define the desired order

        for tl in all_traffic_lights:
            if tl.id in self.traffic_lights_config:
                # Read the initial state and phase durations from the configuration
                config = self.traffic_lights_config[tl.id]
                initial_state = config.get('initial_state', carla.TrafficLightState.Red)
                green_duration = config.get('green_time', 10.0)  # Duration in seconds
                red_duration = config.get('red_time', 10.0)  # Duration in seconds

                green_ticks = int(green_duration / self.fixed_delta_seconds)
                red_ticks = int(red_duration / self.fixed_delta_seconds)

                # Set the traffic light's state in CARLA
                tl.set_state(initial_state)
                tl.freeze(True)

                # Initialize `last_change_tick` to the current tick
                self.traffic_lights[tl.id] = {
                    'actor': tl,
                    'current_state': initial_state,
                    'green_ticks': green_ticks,
                    'red_ticks': red_ticks,
                    'green_time': green_duration,
                    'red_time': red_duration,
                    'last_change_tick': current_tick,  # Set to current tick
                    'remaining_time': green_duration if initial_state == carla.TrafficLightState.Green else red_duration,
                }

                print(f"Traffic Light {tl.id} initialized with state {initial_state.name} and {self.traffic_lights[tl.id]['remaining_time']:.2f} seconds remaining.")
        # Reorder traffic lights based on the custom order
        self.traffic_lights = {key: self.traffic_lights[key] for key in custom_order if key in self.traffic_lights}

    def calculate_route_distance(self, start_location, end_location):
        # Find closest waypoints
        start_index = min(range(len(self.waypoints)),
                  key=lambda i: start_location.distance(self.waypoints[i]))
        end_index = min(range(len(self.waypoints)),
                        key=lambda i: end_location.distance(self.waypoints[i]))

        # Handle slicing for circular route
        if start_index == end_index:
            return 0.0
        elif start_index < end_index:
            route_waypoints = self.waypoints[start_index:end_index]
        else:
            # Wrap around: from start_index to the end of the list, plus from 0 to end_index
            route_waypoints = self.waypoints[start_index:] + self.waypoints[:end_index]
        
        # Calculate total distance by summing segments
        total_distance = 0.0
        if len(route_waypoints) > 1:
            previous_location = route_waypoints[0]
            for i in range(1, len(route_waypoints)):
                current_location = route_waypoints[i]
                total_distance += previous_location.distance(current_location)
                previous_location = current_location

        return total_distance

    def update_traffic_lights(self, vehicle_location, current_tick):
        """Manually manage state transitions for each traffic light."""
        self.current_tick = current_tick

        if not hasattr(self, 'processed_ids'):
            self.processed_ids = set()

        # Iterate through traffic lights to update distances and states
        for tl_id, tl_data in list(self.traffic_lights.items()):
            stop_waypoint = tl_data['actor'].get_stop_waypoints()[1]  # Right lane stop waypoint
            stop_location = stop_waypoint.transform.location
            distance_to_light = self.calculate_route_distance(vehicle_location, stop_location)

            tl_data['distance'] = distance_to_light

            elapsed_ticks = self.current_tick - tl_data['last_change_tick']
            max_ticks = tl_data['green_ticks'] if tl_data['current_state'] == carla.TrafficLightState.Green else tl_data['red_ticks']
            remaining_ticks = max_ticks - elapsed_ticks
            tl_data['remaining_time'] = max(0, remaining_ticks * self.fixed_delta_seconds)

            # Process traffic lights that the platoon has passed (within 2 meters)
            if distance_to_light < 2:
                if tl_id not in self.processed_ids:
                    # Move this traffic light to the end of the dictionary queue
                    tl_data_to_move = self.traffic_lights.pop(tl_id)
                    self.traffic_lights[tl_id] = tl_data_to_move
                    self.processed_ids.add(tl_id)
                    print(f"Traffic Light {tl_id} passed and moved to end of queue.")

            # Phase transition logic
            if elapsed_ticks >= max_ticks:
                if tl_data['current_state'] == carla.TrafficLightState.Green:
                    new_state = carla.TrafficLightState.Red
                    next_duration = tl_data['red_ticks']
                else:
                    new_state = carla.TrafficLightState.Green
                    next_duration = tl_data['green_ticks']

                tl = tl_data['actor']
                tl.set_state(new_state)
                tl_data['current_state'] = new_state
                tl_data['last_change_tick'] = self.current_tick
                tl_data['remaining_time'] = next_duration * self.fixed_delta_seconds

    def stop(self):
        for tl_data in self.traffic_lights.values():
            tl_data['actor'].freeze(False)
        print("Traffic Light Manager stopped.")

    def get_traffic_lights(self):
        return self.traffic_lights
    
    def set_platoon_manager(self, platoon_manager: PlatoonManager):
        self.platoon_manager = platoon_manager
        self.refresh_pams(pam=platoon_manager.pam)

    def refresh_pams(self, pam):
        if pam and pam.platoon_id not in [p.platoon_id for p in self.pams]:
            self.pams.append(pam)
        elif pam:
            for existing_pam in self.pams:
                if existing_pam.platoon_id == pam.platoon_id:
                    existing_pam.platoon_speed = pam.platoon_speed
                    existing_pam.leader_id = pam.leader_id
                    existing_pam.leaving_vehicles = pam.leaving_vehicles
                    existing_pam.status = pam.status
                    existing_pam.split_decision_cntr = pam.split_decision_cntr
                    existing_pam.eta_to_light = pam.eta_to_light
                    existing_pam.platoon_length = pam.platoon_length
                    existing_pam.vehicle_ids = pam.vehicle_ids
                    existing_pam.platoon_position = pam.platoon_position
                    existing_pam.platoon_size = pam.platoon_size
                    existing_pam.corridor_id = pam.corridor_id

    def find_feasible_velocity_range(self, distance_full, green_start, green_end):
        if green_end <= 0:
            return None

        vf_start = max(self.v_min, distance_full / max(green_end, 0.3))
        if green_start > 0:
            vf_end = min(self.v_max, distance_full / max(0.3, green_start)) 
        else:
            vf_end = self.v_max

        if vf_start <= vf_end:
            return (vf_start, vf_end)
        return None

    def check_leader_arrival_time(self, distance, chosen_velocity, green_start, green_end, vehicle_velocity):
        print("[Arrival TIME] Vehicle distance to traffic light:", distance)
        if distance < 0:
            return chosen_velocity, 0

        if chosen_velocity < 0.001:
            arrival_time = float('inf')
        else:
            arrival_time = distance / chosen_velocity
            actual_arrival_time = distance / max(vehicle_velocity, 1.0)
            delta_time = actual_arrival_time - green_start

        if arrival_time < green_start or actual_arrival_time < green_start:
            needed_time = green_start
            if needed_time <= 0:
                return chosen_velocity, delta_time
            needed_v = distance / needed_time
            if needed_v < chosen_velocity:
                if self.corridor_change:
                    return needed_v, delta_time    
                return needed_v * 1.05, delta_time

        if arrival_time > green_end and actual_arrival_time > green_end:
            return None, delta_time

        return chosen_velocity, delta_time
    
    def check_last_arrival_time(self, distance, chosen_velocity, green_end, vehicle_velocity, delta_time_leader):
        if distance < 0:
            return chosen_velocity
        if chosen_velocity < 0.001:
            arrival_time = float('inf')
        else:
            arrival_time = distance / chosen_velocity
            actual_arrival_time = distance / max(vehicle_velocity, 1.0)
            delta_time = actual_arrival_time - arrival_time + delta_time_leader
            
        if arrival_time > green_end or actual_arrival_time > (green_end + delta_time):
            return None
        return chosen_velocity
    
    def compute_subplatoon_length(self, pcms_subgroup):
        if not pcms_subgroup:
            return 0.0
        sorted_sub = sorted(pcms_subgroup, key=lambda p: p.position_in_platoon)
        length = 0.0
        for pcm in sorted_sub[1:]:
            length += pcm.distance_to_front
        return length
    
    def check_trailing_vehicle_arrival(self, distance_to_stop_line, leader_velocity, green_end, vf_end, delta_time_leader, v_saturation):
        if self.platoon_manager.pam.platoon_speed < v_saturation and leader_velocity < vf_end:
            delta_time_leader = 1
        if distance_to_stop_line < 0:
            return True

        if leader_velocity < 0.001:
            arrival_time = float('inf')
        else:
            arrival_time = (distance_to_stop_line / leader_velocity) + delta_time_leader

        return arrival_time < green_end

    def split_for_first_green_window(self, pam, pcms, distance_to_light, green_start, green_end, feasible_range, v_saturation):
        (vf_start, vf_end) = feasible_range
        sorted_pcms = sorted(pcms, key=lambda x: x.position_in_platoon)
        leader_pcm = next((p for p in sorted_pcms if p.vehicle_id == pam.leader_id), None)
        if not leader_pcm:
            return {"mode": "NONE", "velocity": None, "front_group": [], "rear_group": [], "sub_platoon": None}

        front_group = [leader_pcm]
        rear_candidates = [p for p in sorted_pcms if p != leader_pcm]

        if not self.corridor_change:
            updated_v_leader, delta_time_leader = self.check_leader_arrival_time(
                distance=distance_to_light - pam.platoon_length,
                chosen_velocity=pam.platoon_speed,
                green_start=green_start,
                green_end=green_end,
                vehicle_velocity=leader_pcm.target_speed
            )
        else:
            updated_v_leader = pam.platoon_speed
            delta_time_leader = 0
            
        if not updated_v_leader:
            return {"mode": "NONE", "velocity": None, "front_group": [], "rear_group": [], "sub_platoon": None}

        chosen_v_for_front = updated_v_leader
        feasible_front_group = list(front_group)

        for next_vehicle in rear_candidates:
            candidate_front = feasible_front_group + [next_vehicle]
            sub_len = self.compute_subplatoon_length(candidate_front)
            dist_sub = (distance_to_light - self.platoon_manager.pam.platoon_length) + sub_len
            bool_sat, trailing_v_saturation = self._check_saturation_flow_for_trailing_vehicles(sub_len, green_start, green_end, chosen_v_for_front, dist_sub, v_saturation)
            can_pass = self.check_trailing_vehicle_arrival(dist_sub, chosen_v_for_front, green_end, vf_end, delta_time_leader, trailing_v_saturation)
            if not can_pass:
                break
            feasible_front_group = candidate_front

        rear_group = [v for v in sorted_pcms if v not in feasible_front_group]
        if not rear_group:
            return {"mode": "ENTIRE", "velocity": chosen_v_for_front, "front_group": feasible_front_group, "rear_group": [], "sub_platoon": None}

        pam.split_decision_cntr += 1
        if pam.split_decision_cntr >= 5:
            self.split_counter += 1
            sub_platoon = self.platoon_manager.split_platoon(
                platoon=feasible_front_group,
                new_group=rear_group,
                new_platoon_id=self.split_counter,
                tl_id=self.corridor_id,
                eta_to_light=max(0.1, distance_to_light / chosen_v_for_front)
            )
            pam.split_decision_cntr = 0
            return {"mode": "SPLIT", "velocity": chosen_v_for_front, "front_group": feasible_front_group, "rear_group": rear_group, "sub_platoon": sub_platoon}
        else:
            return {"mode": "WAIT", "velocity": chosen_v_for_front, "front_group": feasible_front_group, "rear_group": rear_group, "sub_platoon": []}
    
    def _compute_feasible_green_windows(self):
        self.corridor_id, tl_data = min(self.traffic_lights.items(), key=lambda item: item[1]['distance'])
        distance_to_stop = tl_data['distance']
        remaining_time = tl_data['remaining_time']
        red_time = tl_data['red_time']
        green_time = tl_data['green_time']
        total_cycle = red_time + green_time
        feasible_windows = []
        current_phase = tl_data['current_state']

        for i in range(4):
            if current_phase == carla.TrafficLightState.Red:
                green_start = remaining_time + i * total_cycle
                green_end = green_start + green_time
            else:
                if i == 0:
                    green_start = 0
                    green_end = remaining_time
                else:
                    green_start = remaining_time + (i - 1) * total_cycle + red_time
                    green_end = remaining_time + i * total_cycle
            
            dist = distance_to_stop
            v_start = max(self.v_min, dist / max(green_end, 0.3))
            v_end = min(self.v_max, dist / max(green_start, 0.3))
            
            if (v_start <= v_end) and (v_start < 14.0): 
                feasible_windows.append((v_start, v_end, green_start, green_end))
        
        if not feasible_windows and self.corridor_change:
            feasible_windows.append((self.platoon_manager.pam.platoon_speed, self.platoon_manager.pam.platoon_speed, green_start, green_end))
        return feasible_windows
    
    def _check_entire_platoon_feasibility(self, feasible_range, distance_to_light, green_start, green_end, related_pcms, pam, v_saturation):
        (v_lower_bound, v_upper_bound) = feasible_range
        leader_pcm = related_pcms[0]
        last_pcm = related_pcms[-1]
        if not leader_pcm:
            return {"status": "NONE", "velocity": None}
        
        leader_distance = distance_to_light - pam.platoon_length
        if not self.corridor_change:
            updated_v_leader, delta_time_leader = self.check_leader_arrival_time(leader_distance, v_upper_bound, green_start, green_end, leader_pcm.target_speed)
        else:
            updated_v_leader = pam.platoon_speed
            delta_time_leader = 0
            
        if updated_v_leader is None:
            return {"status": "NONE", "velocity": None}

        updated_v_last = self.check_last_arrival_time(distance_to_light, updated_v_leader, green_end, last_pcm.target_speed, delta_time_leader)
        if updated_v_last is None:
            return {"status": "SPLIT", "velocity": updated_v_leader * 0.5}

        candidate_vel = 0.5 * (updated_v_leader + updated_v_last)
        if candidate_vel >= v_lower_bound:
            return {"status": "ENTIRE", "velocity": updated_v_leader if not self.corridor_change else updated_v_last}
        else:
            return {"status": "SPLIT", "velocity": updated_v_leader}

    def _check_saturation_flow(self, platoon_length, green_start, green_end, vf_start, distance_to_light):
        time_window = green_end - green_start
        if green_start == 0:
            return True, distance_to_light / green_end
        required_speed = platoon_length / time_window
        return (required_speed <= self.v_max - 1), required_speed
    
    def _check_saturation_flow_for_trailing_vehicles(self, platoon_length, green_start, green_end, vf_start, distance_to_light, v_saturation):
        time_window = green_end - green_start
        if green_start == 0:
            return True, distance_to_light / green_end
        required_speed = platoon_length / time_window
        return (required_speed <= self.v_max - 1 and required_speed >= v_saturation), required_speed

    def _finalize_split_decision(self, result, traffic_light_key, eta_to_light):
        mode = result["mode"]
        vel = result["velocity"]
        if mode == "NONE":
            self.ref_v = self.v_min
            return self.ref_v, result, traffic_light_key, eta_to_light
        if mode in ["ENTIRE", "SPLIT", "WAIT"]:
            self.ref_v = result["velocity"]
            return self.ref_v, result, traffic_light_key, eta_to_light
        self.ref_v = self.v_min
        return self.ref_v, result, traffic_light_key, eta_to_light

    def calculate_reference_velocity(self):
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        if self.start_time is None: self.start_time = current_time
        sim_time_elapsed = current_time - self.start_time

        if sim_time_elapsed < self.init_delay:
            return self.ref_v, {"mode": "WAITING", "velocity": self.ref_v, "front_group": [], "rear_group": [], "sub_platoon": None}, None, 1000
        
        pam = self.platoon_manager.pam
        related_pcms = [pcm for pcm in self.platoon_manager.pcms if pcm.vehicle_id in pam.vehicle_ids]
        if not related_pcms:
            return self.v_min, {"mode": "NONE", "velocity": None, "front_group": [], "rear_group": [], "sub_platoon": None}, None, None

        self.corridor_id, tl_data = min(self.traffic_lights.items(), key=lambda item: item[1]['distance'])
        distance_to_light = tl_data['distance']

        if distance_to_light - pam.platoon_length >= 300:
            return pam.platoon_speed, {"mode": "WAITING FOR SPAT", "velocity": pam.platoon_speed, "front_group": [pam.vehicle_ids], "rear_group": [], "sub_platoon": None}, self.corridor_id, 100
        
        leader_pcm = related_pcms[0]
        self.corridor_change = (distance_to_light - pam.platoon_length < 0 or (distance_to_light - pam.platoon_length) < leader_pcm.target_speed)

        if distance_to_light < pam.platoon_speed:
            return pam.platoon_speed, {"mode": "PASSING", "velocity": pam.platoon_speed, "front_group": [pam.vehicle_ids], "rear_group": [], "sub_platoon": None}, self.corridor_id, max(0.1, tl_data["distance"] / max(pam.platoon_speed, 1.0))

        feasible_windows = self._compute_feasible_green_windows()
        if not feasible_windows:
            self.ref_v = self.v_min
            return self.ref_v, {"mode": "NONE", "velocity": None, "front_group": [], "rear_group": [], "sub_platoon": None}, self.corridor_id, None
        
        platoon_id = self.platoon_manager.platoon_id
        (vf_start, vf_end, green_start, green_end) = feasible_windows[0]

        if platoon_id is not None and platoon_id > 1:
            for other_pam in self.pams:
                if other_pam.platoon_id != platoon_id and other_pam.corridor_id == pam.corridor_id and (other_pam.eta_to_light < pam.eta_to_light):
                    for window in feasible_windows:
                        (vfc, vfe, gsc, gec) = window
                        if vfe < other_pam.platoon_speed * 1.1:
                            (vf_start, vf_end, green_start, green_end) = window
                            break
                    break
        
        feasible_range = (vf_start, vf_end)
        bool_sat, v_saturation = self._check_saturation_flow(pam.platoon_length, green_start, green_end, vf_start, distance_to_light)
        
        if feasible_range and bool_sat and (pam.platoon_speed > v_saturation or self.corridor_change):
            entire_check = self._check_entire_platoon_feasibility(feasible_range, distance_to_light, green_start, green_end, related_pcms, pam, v_saturation)
            if entire_check["status"] == "ENTIRE":
                self.ref_v = entire_check["velocity"]
                return self.ref_v, {"mode": "ENTIRE", "velocity": self.ref_v, "front_group": related_pcms, "rear_group": [], "sub_platoon": None}, self.corridor_id, tl_data["distance"] / self.ref_v
            else:
                result = self.split_for_first_green_window(pam, related_pcms, distance_to_light, green_start, green_end, feasible_range, v_saturation)
                safe_vel = result["velocity"] if result["velocity"] is not None else max(self.v_min, 1.0)
                return self._finalize_split_decision(result, self.corridor_id, max(0.1, tl_data["distance"] / safe_vel))

        result = self.split_for_first_green_window(pam, related_pcms, distance_to_light, green_start, green_end, feasible_range, v_saturation)
        safe_vel = result["velocity"] if result["velocity"] is not None else max(self.v_min, 1.0)
        return self._finalize_split_decision(result, self.corridor_id, max(0.1, tl_data["distance"] / safe_vel))

    def is_red_light_ahead(self, vehicle_location):
        tl_id, tl_data = min(self.traffic_lights.items(), key=lambda item: item[1]['distance'])
        if tl_data['current_state'] == carla.TrafficLightState.Red and tl_data['distance'] < 12.0:
                return True, tl_id
        return False, None

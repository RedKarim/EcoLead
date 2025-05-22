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
                # csv_file = open(f'TL_{tl.id}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
                # writer = csv.writer(csv_file)
                # writer.writerow(['Time','Vf Start','Vf End','Platoon ID'])
                # self.csv_files[f'TL_{tl.id}'] = csv_file
        # Reorder traffic lights based on the custom order
        self.traffic_lights = {key: self.traffic_lights[key] for key in custom_order if key in self.traffic_lights}

    def calculate_route_distance(self, start_location, end_location):
        # NOTE: Ciricular Route with 
        start_index = min(range(len(self.waypoints)),
                  key=lambda i: start_location.distance(self.waypoints[i]))
        end_index = min(range(len(self.waypoints)),
                        key=lambda i: end_location.distance(self.waypoints[i]))
        print(f"Start Index: {start_index}, End Index: {end_index}")

        # Handle slicing for circular routep
        if start_index == end_index:
            total_distance = 0.0
        if start_index < end_index:
            route_waypoints = self.waypoints[start_index:end_index]
        else:
            # Wrap around: from start_index to the end, plus from 0 to end_index
            # NOTE: Ciricular Route with 726 waypoints, should be automated
            route_waypoints = self.waypoints[start_index:2504] + self.waypoints[:end_index]
        
        # Calculate total distance
        total_distance = 0.0
        previous_location = route_waypoints[0]
        for i in range(1, len(route_waypoints)):
            current_location = route_waypoints[i]
            segment_distance = previous_location.distance(current_location)
            total_distance += segment_distance
            previous_location = current_location

        # print(f"Total distance from vehicle to traffic light: {total_distance}")
        return total_distance

    def update_traffic_lights(self, vehicle_location, current_tick):
        """Manually manage state transitions for each traffic light."""
        self.current_tick = current_tick

        # Initialize a set to keep track of traffic light IDs that have already been moved to the end
        if not hasattr(self, 'processed_ids'):
            self.processed_ids = set()

        # Iterate through traffic lights to find the closest one
        for tl_id, tl_data in list(self.traffic_lights.items()):
            # Calculate distance to the traffic light stop waypoint using route distance
            stop_waypoint = tl_data['actor'].get_stop_waypoints()[1]  # Right lane stop waypoint
            stop_location = stop_waypoint.transform.location
            distance_to_light = self.calculate_route_distance(vehicle_location, stop_location)

            # Store the calculated distance
            tl_data['distance'] = distance_to_light

            # Calculate elapsed ticks
            elapsed_ticks = self.current_tick - tl_data['last_change_tick']
            max_ticks = tl_data['green_ticks'] if tl_data['current_state'] == carla.TrafficLightState.Green else tl_data['red_ticks']
            remaining_ticks = max_ticks - elapsed_ticks
            tl_data['remaining_time'] = max(0, remaining_ticks * self.fixed_delta_seconds)  # Ensure non-negative time

            # Process traffic lights within 2 meters
            if distance_to_light < 2:
                print(f"Vehicle is near Traffic Light {tl_id} (distance < 2 meters).")
                if tl_id not in self.processed_ids:
                    # Move this traffic light to the end of the queue
                    tl_data_to_move = self.traffic_lights.pop(tl_id)
                    self.traffic_lights[tl_id] = tl_data_to_move
                    self.processed_ids.add(tl_id)
                    print(f"Traffic Light {tl_id} processed and moved to the end.")

            # Check if it's time to change the traffic light state
            if elapsed_ticks >= max_ticks:
                # Toggle state between Green and Red
                if tl_data['current_state'] == carla.TrafficLightState.Green:
                    new_state = carla.TrafficLightState.Red
                    next_duration = tl_data['red_ticks']
                else:
                    new_state = carla.TrafficLightState.Green
                    next_duration = tl_data['green_ticks']

                # Update the traffic light
                tl = tl_data['actor']
                tl.set_state(new_state)
                tl_data['current_state'] = new_state
                tl_data['last_change_tick'] = self.current_tick
                tl_data['remaining_time'] = next_duration * self.fixed_delta_seconds  # Reset remaining time


    def stop(self):
        """Unfreeze all managed traffic lights upon stopping."""
        for tl_data in self.traffic_lights.values():
            tl_data['actor'].freeze(False)
        print("Traffic Light Manager stopped.")

    def get_traffic_lights(self):
        """Returns a dictionary of controlled traffic light actors."""
        return self.traffic_lights
    
    def set_platoon_manager(self, platoon_manager: PlatoonManager):
        """Set the platoon manager reference to access PCMs and PAM."""
        print("Setting Platoon Manager...")
        self.platoon_manager = platoon_manager
        # Store each new pam if not already present
        self.refresh_pams(pam=platoon_manager.pam)

    def refresh_pams(self, pam):
        """
        Update or add new pams without duplicates.
        """
        # If pam is new and not in self.pams, append it
        if pam and pam.platoon_id not in [p.platoon_id for p in self.pams]:
            self.pams.append(pam)
        else:
            # Optionally update existing pam data here
            for existing_pam in self.pams:
                # print(f"Existing PAM: {existing_pam}")
                # print(f"NEW PAM: {pam}")
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

    # ------------------------------------------------------------------------
    # 1 Helper: find_feasible_velocity_range
    # ------------------------------------------------------------------------
    def find_feasible_velocity_range(self, distance_full, green_start, green_end):
        """
        Returns (vf_start, vf_end) if the subplatoon can clear the intersection
        between green_start..green_end, else None.

        - Must finish crossing by green_end => v >= distance_full / green_end
        - Must not arrive before green_start => v <= distance_full / green_start (if green_start>0)
        """
        if green_end <= 0:
            return None

        vf_start = max(self.v_min, distance_full / max(green_end,0.3))
        if green_start > 0:
            vf_end = min(self.v_max, distance_full / max(0.3,green_start)) 
        else:
            # If green_start <= 0 => already green
            vf_end = self.v_max

        if vf_start <= vf_end:
            return (vf_start, vf_end)
        return None
    # ------------------------------------------------------------------------
    # 2 Helper: check_leader_arrival_time
    # ------------------------------------------------------------------------
    def check_leader_arrival_time(self, distance, chosen_velocity, green_start, green_end, vehicle_velocity):
        """
        Ensures a vehicle (leader or last car) does not:
        - Arrive before green_start (avoid red violation),
        - Arrive after green_end (avoid missing the green).

        If arriving too early, we try to slow down to arrive exactly at green_start.
        If that breaks feasibility, return None.
        If too late, also return None.
        Otherwise, return the feasible velocity.
        """
        print("[Arrival TIME] Vehicle distance to traffic light:", distance)
        if distance < 0:
            print("[SPLIT LOGIC] Vehicle is already past the traffic light.")
            return chosen_velocity,0

        if chosen_velocity < 0.001:
            arrival_time = float('inf')
        else:
            arrival_time = distance / chosen_velocity
            actual_arrival_time = distance / max(vehicle_velocity,1)
            delta_time = actual_arrival_time - green_start # delta real + delta ideal

        print(f"[Arrival TIME] Vehicle ideal arrival time: {arrival_time:.2f} s.")
        print(f"[Arrival TIME] Vehicle actual arrival time: {actual_arrival_time:.2f} s.")
        print(f"[Arrival TIME] Delta Time: {delta_time:.2f} s.")

        # Arrive too early => attempt to slow down
        if arrival_time < green_start or actual_arrival_time < green_start:
            needed_time = green_start
            print(f"[Arrival TIME] Needed time to match green start: {needed_time:.2f} s.")
            if needed_time <= 0:
                # The green is basically starting now or already on
                return chosen_velocity, delta_time
            needed_v = distance / needed_time
            print(f"[Arrival TIME] Computed needed velocity: {needed_v:.4f} m/s.")
            print(f"[Arrival TIME] Current chosen velocity: {chosen_velocity:.4f} m/s.")
            print(f"[Arrival TIME] Current velocity: {vehicle_velocity:.4f} m/s.")

            if needed_v < chosen_velocity:
                print(f"[Arrival TIME] Return the needed: {needed_v:.4f} m/s.")
                if self.corridor_change:
                    print(f"[Arrival TIME] Corrirdor Change: {needed_v:.4f} m/s.")
                    return needed_v, delta_time    
                return needed_v*1.05, delta_time
            # else:
            #     print(f"[Arrival TIME] None: cannot adjust arrival from {chosen_velocity:.4f} to {needed_v:.4f}.")
            #     return chosen_velocity  # can't fix arrival or NOTE: slow down!

        # Arrive too late => out of window
        if arrival_time > green_end and actual_arrival_time > green_end:
            print(f"[Arrival TIME] Ideal Arrival time {arrival_time:.2f}s and actual artival time {actual_arrival_time:.2f} exceeds green_end {green_end}.")
            return None, delta_time

        # Otherwise, arrival is within the window
        print(f"[Arrival TIME] Arrival time is within the green window at velocity {chosen_velocity:.4f} m/s.")
        return chosen_velocity, delta_time
    
    def check_last_arrival_time(self, distance, chosen_velocity, green_end, vehicle_velocity, delta_time_leader):
        """
        Ensures a vehicle (leader or last car) does not:
        - Arrive before green_start (avoid red violation),
        - Arrive after green_end (avoid missing the green).

        If arriving too early, we try to slow down to arrive exactly at green_start.
        If that breaks feasibility, return None.
        If too late, also return None.
        Otherwise, return the feasible velocity.
        """
        print("[Arrival TIME] Vehicle distance to traffic light:", distance)
        if distance < 0:
            print("[SPLIT LOGIC] Vehicle is already past the traffic light.")
            return chosen_velocity,0

        if chosen_velocity < 0.001:
            arrival_time = float('inf')
        else:
            arrival_time = distance / chosen_velocity
            actual_arrival_time = distance / max(vehicle_velocity,1)
            delta_time = actual_arrival_time - arrival_time + delta_time_leader
            
        print(f"[Arrival TIME LAST CAR] Chosen Velocity: {chosen_velocity:.2f} s.")
        print(f"[Arrival TIME LAST CAR] Vehicle ideal arrival time: {arrival_time:.2f} s.")
        print(f"[Arrival TIME LAST CAR] Vehicle actual arrival time: {actual_arrival_time:.2f} s.")
        print(f"[Arrival TIME LAST CAR] Delta Time: {delta_time:.2f} s.")
        needed_v = chosen_velocity
        # Arrive too late => out of window SPLIT!!
        if arrival_time > green_end or actual_arrival_time > (green_end+delta_time):
            print(f"[Arrival TIME LAST CAR] None: arrival time {arrival_time:.2f}s exceeds green_end {green_end}.")
            # return None
            needed_v = None
        if self.corridor_change and chosen_velocity < (distance / max(1,green_end)):
                needed_v = distance / green_end
                print(f"[Arrival TIME LAST CAR] Corridor Change: {needed_v:.4f} m/s.")
                needed_v = needed_v
        # Otherwise, arrival is within the window
        print(f"[Arrival TIME LAST CAR] Arrival time is within the green window at velocity {chosen_velocity:.4f} m/s.")
        return needed_v
    
    # ------------------------------------------------------------------------
    # 3) Helper: compute_subplatoon_length
    # ------------------------------------------------------------------------
    def compute_subplatoon_length(self, pcms_subgroup):
        """
        Approximate the 'length' from the leader to the last car in pcms_subgroup.
        For now, we sum up 'distance_to_front' for each 'following' car.
        """
        if not pcms_subgroup:
            return 0.0

        # Sort by position_in_platoon so we accumulate forward
        sorted_sub = sorted(pcms_subgroup, key=lambda p: p.position_in_platoon)
        # The leader is first => no distance_to_front from leader to itself
        length = 0.0
        for pcm in sorted_sub[1:]:
            length += pcm.distance_to_front
        return length
    
    def check_trailing_vehicle_arrival(self, distance_to_stop_line, leader_velocity,green_end,  vf_end, delta_time_leader,v_saturation):
        """
        Checks if a trailing vehicle, located 'distance_to_stop_line' away from 
        the intersection, can arrive before green_end when traveling at 
        'leader_velocity'.

        :param distance_to_stop_line: float 
            The distance from the vehicle to the traffic-light stop line (meters).
        :param leader_velocity: float
            The velocity (m/s) at which the leader (and thus the subplatoon) is traveling.
        :param green_end: float
            The cutoff time (seconds from now) by which this vehicle must arrive 
            to avoid missing the green.

        :return: bool 
            True if this trailing vehicle can pass before green_end at 'leader_velocity'.
            False otherwise.
        """
        if self.platoon_manager.pam.platoon_speed < v_saturation and leader_velocity < vf_end:
            # This is the case where we brake but still think we can pass but we have to split hard.
            print(f"[ARRIVAL CHECK] Vehicle is slower than saturation and leader is slower than vf_end")
            delta_time_leader = 1
        if distance_to_stop_line < 0:
            # Already past the intersection => trivially okay
            print(f"[ARRIVAL CHECK] Vehicle is already at/past stop line => can pass.")
            return True

        if leader_velocity < 0.001:
            # Effectively zero => won't arrive in time
            arrival_time = float('inf')
        else:
            arrival_time = (distance_to_stop_line / leader_velocity) + delta_time_leader

        print(f"[ARRIVAL CHECK] distance={distance_to_stop_line:.2f} m, "
            f"leader_vel={leader_velocity:.2f} m/s => arrival_time={arrival_time:.2f} s (green_end={green_end:.2f} s).")

        if arrival_time <green_end:
            print("[ARRIVAL CHECK] Trailing vehicle can make it before green ends.")
            return True
        else:
            print("[ARRIVAL CHECK] Trailing vehicle misses the green => False.")
            return False
    # ------------------------------------------------------------------------
    # 4) Core Logic: Attempt entire platoon, else split into 2 subplatoons
    # ------------------------------------------------------------------------
    def split_for_first_green_window(self, pam, pcms, distance_to_light, green_start, green_end, feasible_range, v_saturation):
        """
        This method ensures:
        1) At least the leader is in the front group.
        2) We keep adding vehicles behind the leader if still feasible in [green_start..green_end].
        3) The remainder forms the rear group, which waits for the second green.

        Return dict describing the result:
        {
            "mode": "SPLIT" or "ENTIRE" or "NONE",
            "velocity": <float or None>,
            "front_group": [...],
            "rear_group": [...],
            "sub_platoon": <the new platoon object if we actually split>
        }
        """
        print("--------------------SPLITTING----------------------")
        print("[SPLIT LOGIC] Entire platoon not feasible or leader violates red => partial split...")

        (vf_start, vf_end) = feasible_range

        # Sort pcms so we can build from the leader forward
        sorted_pcms = sorted(pcms, key=lambda x: x.position_in_platoon)
        leader_pcm = next((p for p in sorted_pcms if p.vehicle_id == pam.leader_id), None)
        if not leader_pcm:
            print("No leader PCM => can't even do partial split => NONE.")
            return {"mode": "NONE", "velocity": None, "front_group": [], "rear_group": [],"sub_platoon": None}

        # The front group will always contain the leader
        front_group = [leader_pcm]
        rear_candidates = [p for p in sorted_pcms if p != leader_pcm]

        # First, check if the leader alone can pass
        print("--------------------[SPLIT LOGIC] Checking feasibility for leader alone.-----------------------")
        if not self.corridor_change: # Buraya baska bisi bulunmalali, Green coktan baslamissa?
            updated_v_leader,delta_time_leader = self.check_leader_arrival_time(
                distance=distance_to_light - pam.platoon_length,
                chosen_velocity=pam.platoon_speed,
                green_start=green_start,
                green_end=green_end,
                vehicle_velocity=leader_pcm.target_speed
            )
        else:
            print(f"[PLATOON Feasibility] Leader has max 1 second .{pam.platoon_speed}")
            updated_v_leader = pam.platoon_speed
            delta_time_leader = 0
        if not updated_v_leader:
            # Leader alone can't pass =>
            print("[SPLIT LOGIC] Leader alone can't pass => NONE.")
            return {
                "mode": "NONE",
                "velocity": None,
                "front_group": [],
                "rear_group": [],
                "sub_platoon": None
            }

        chosen_v_for_front = updated_v_leader
        feasible_front_group = list(front_group)

        # Try to add vehicles behind the leader, as long as it's feasible
        for next_vehicle in rear_candidates:
            candidate_front = feasible_front_group + [next_vehicle]
            sub_len = self.compute_subplatoon_length(candidate_front)
            dist_sub = (distance_to_light - self.platoon_manager.pam.platoon_length) + sub_len
            print(f"[SPLIT LOGIC] Subplatoon length: {sub_len:.2f} m, Distance to light: {dist_sub:.2f} m")
            # r = self.find_feasible_velocity_range(dist_sub, green_start, green_end)
            # print(f"[SPLIT LOGIC] Feasible Range for subplatoon +1 vehicle: {r}")
            bool_sat, trailing_v_saturation = self._check_saturation_flow_for_trailing_vehicles(sub_len, green_start, green_end, chosen_v_for_front, dist_sub,v_saturation)
            # We'll pick the lower bound so we can still decelerate if early
            can_pass = self.check_trailing_vehicle_arrival(
                distance_to_stop_line=dist_sub,
                leader_velocity=chosen_v_for_front,
                green_end=green_end,
                vf_end=vf_end,
                delta_time_leader=delta_time_leader,
                v_saturation=trailing_v_saturation
            )
            if not can_pass:
                # Can't add the next vehicle
                break

            # Otherwise, it's feasible
            feasible_front_group = candidate_front

        # If we ended up adding everyone => entire platoon
        rear_group = [v for v in sorted_pcms if v not in feasible_front_group]
        if not rear_group:
            print("[SPLIT LOGIC] Actually the entire platoon can pass after all.")
            return {
                "mode": "ENTIRE",
                "velocity": chosen_v_for_front,
                "front_group": feasible_front_group,
                "rear_group": [],
                "sub_platoon": None
            }

        # Otherwise, we have a front subplatoon plus a rear group
        print("[SPLIT LOGIC] Splitting into 2 subplatoons:")
        print("  FRONT:", [p.vehicle_id for p in feasible_front_group])
        print("  REAR:", [p.vehicle_id for p in rear_group])

        # Actually call your platoon_manager to finalize the split
        # if pam.leaving_vehicles == [p.vehicle_id for p in rear_group] and len([p.vehicle_id for p in rear_group])>0:
        #     pam.split_decision_cntr += 1
        # else: pam.split_decision_cntr = 0
        pam.split_decision_cntr += 1

        print(f"[SPLIT LOGIC] Split decision counter: {pam.split_decision_cntr}")
        if pam.split_decision_cntr >= 5:
            self.split_counter += 1
            sub_platoon = self.platoon_manager.split_platoon(
                platoon=feasible_front_group,
                new_group=rear_group,
                new_platoon_id=self.split_counter,
                tl_id=self.corridor_id,
                eta_to_light= max(0.1,distance_to_light / chosen_v_for_front)
            )
            pam.split_decision_cntr = 0

            return {
                "mode": "SPLIT",
                "velocity": chosen_v_for_front,
                "front_group": feasible_front_group,
                "rear_group": rear_group,
                "sub_platoon": sub_platoon
            }
        else:
            print("[SPLIT LOGIC] Waiting for plitting decision counter to reach 10.")
            return {
                "mode": "WAIT",
                "velocity": chosen_v_for_front,
                "front_group": feasible_front_group,
                "rear_group": rear_group,
                "sub_platoon": []}
    
    def _compute_feasible_green_windows(self):
        """
        Adapted from your old 'green_windows' method.
        Gathers feasible velocity ranges for the next N (3) cycles, 
        based on traffic light data in self.traffic_lights.
        """
        print("[CALC REF VEL] Feasible windows for the next 3 cycles.")
        # first_key = next(iter(self.traffic_lights))
        # tl_data = self.traffic_lights[first_key]
        self.corridor_id, tl_data = min(
            self.traffic_lights.items(), key=lambda item: item[1]['distance'])

        distance_to_stop = tl_data['distance']
        remaining_time   = tl_data['remaining_time']
        red_time         = tl_data['red_time']
        green_time       = tl_data['green_time']
        total_cycle      = red_time + green_time

        feasible_windows = []

        current_phase = tl_data['current_state']
        # If we're red, the next green starts in 'remaining_time'
        # If we're green, we have 'remaining_time' seconds of green left
        for i in range(4):
            if current_phase == carla.TrafficLightState.Red:
                # Red light logic
                green_start = remaining_time + i * total_cycle
                green_end = green_start + green_time
            else:
                # Green light logic
                if i == 0:
                    green_start = 0
                    green_end = remaining_time
                else:
                    green_start = remaining_time + (i - 1) * total_cycle + red_time
                    green_end = remaining_time + i * total_cycle
            # if not self.corridor_change:
            #     dist = distance_to_stop-self.platoon_manager.pam.platoon_length
            # else:
            #     dist = distance_to_stop
            dist = distance_to_stop
             # Calculate velocity ranges for this green window
            v_start = max(self.v_min, dist/ max(green_end, 1))
            v_end = min(self.v_max, dist/ max(green_start, 1))
            # v_start = max(self.platoon_manager.pam.platoon_speed, distance_to_stop/ max(green_end, 1))
            # v_end = min(self.platoon_manager.pam.platoon_speed, distance_to_stop/ max(green_start, 1))
            print(f"[Cycle {i}]V_start: {v_start},V_end: {v_end}")
            #print(f"[Cycle {i}] Green start: {green_start:.2f}s, Green end: {green_end:.2f}s")
            # Check if the range is valid
            if (v_start <= v_end) and (v_start < 13.0):
                feasible_windows.append((v_start, v_end, green_start, green_end))
                print(f"[Cycle {i}] Green start: {green_start:.2f}s, Green end: {green_end:.2f}s")
                print(f"[Cycle {i}] Velocity range: {v_end:.2f} m/s to {v_start:.2f} m/s")
            # elif self.platoon_manager.pam.platoon_speed > 13 and self.corridor_change:
            #     print(f"[Cycle {i}]Corridor Change : {v_end} m/s")
            #     feasible_windows.append((v_end, v_end, green_start, green_end))
            else:
                print(f"[Cycle {i}] No feasible velocity range, but v_min als Ref Velocity.")
                # feasible_windows.append((self.v_min, self.v_min, green_start, green_end))
        if not feasible_windows and self.corridor_change:
            print(f"[Cycle {i}]Corridor Change : {v_end} m/s")
            feasible_windows.append((self.platoon_manager.pam.platoon_speed, self.platoon_manager.pam.platoon_speed, green_start, green_end))
        return feasible_windows
    
    def _check_entire_platoon_feasibility(self, 
                                          feasible_range, 
                                          distance_to_light,
                                          green_start, 
                                          green_end, 
                                          related_pcms, 
                                          pam,
                                          v_saturation):
        """
        1) Check if the platoon LEADER can pass at the UPPER bound of feasible_range 
           (so it doesn't violate red).
        2) Check if the LAST CAR can pass at the LOWER bound.
        3) Return a dictionary with 'status' and 'velocity' for final usage.

        status can be:
          - "ENTIRE" => entire platoon feasible
          - "SPLIT"  => leader feasible but last car not => partial
          - "NONE"   => leader not feasible => no pass (or partial)
        """
        (v_lower_bound, v_upper_bound) = feasible_range
        print(f"[PLATOON Feasibility] feasible_range={feasible_range}")

        # 1) Leader check with the upper bound
        leader_pcm = related_pcms[0]
        last_pcm = related_pcms[-1]
        # print(f"[PLATOON Feasibility] Last={last_pcm}")
        if not leader_pcm:
            # No leader => can't do entire
            return {"status": "NONE", "velocity": None}
        leader_distance = distance_to_light - pam.platoon_length
        print("-------------Leader CHECK----------------")
        if not self.corridor_change: # Buraya baska bisi bulunmalali, Green coktan baslamissa?
            updated_v_leader,delta_time_leader = self.check_leader_arrival_time(
                distance=leader_distance,
                chosen_velocity=v_upper_bound,
                green_start=green_start,
                green_end=green_end,
                vehicle_velocity=leader_pcm.target_speed
            )
        else:
            print(f"[PLATOON Feasibility] Leader has max 1 second .{v_saturation}")
            updated_v_leader = pam.platoon_speed
            delta_time_leader = 0
        # If leader not feasible => "NONE" or partial
        if updated_v_leader is None:
            return {"status": "NONE", "velocity": None}
        print("-------------LAST CAR CHECK----------------")
        # 2) Last car check with the lower bound
        last_vehicle_distance = distance_to_light
        updated_v_last = self.check_last_arrival_time(
            distance=last_vehicle_distance,
            chosen_velocity=updated_v_leader,
            green_end=green_end,
            vehicle_velocity=last_pcm.target_speed,
            delta_time_leader=delta_time_leader,
        )

        # If last car fails, we do partial split
        if updated_v_last is None:
            return {"status": "SPLIT", "velocity": updated_v_leader*0.5}

        # ---------------------------------------------
        # (3) Combine with v_saturation
        # ---------------------------------------------
        # We want a single velocity that doesn't break red 
        # for either leader or last car, AND doesn't exceed 
        # the saturation needed. Usually we pick the 'min'.

        candidate_vel = 0.5 * (updated_v_leader + updated_v_last)
        print(f"[ENTIRE FEAS] Combining (Leader={updated_v_leader:.2f}, "
            f"Last={updated_v_last:.2f}, Saturation={v_saturation:.2f})")

        # Finally, ensure the chosen velocity is still within [v_lower_bound, v_upper_bound]
        if candidate_vel >= v_lower_bound:
            # If it's below v_lower_bound, we can't sustain that slow without red violation
            # or we can't physically drive that slow safely. Decide if it's SPLIT or NONE:
            print(f"[ENTIRE FEAS] candidate_vel={candidate_vel:.2f} - v_lower_bound={v_lower_bound:.2f} => GOOD TO GO.")
            return {"status": "ENTIRE", "velocity": updated_v_leader if not self.corridor_change else updated_v_last}

        else:
            print(f"[ENTIRE FEAS] candidate_vel={candidate_vel:.2f} < v_lower_bound={v_lower_bound:.2f} => SPLIT.")
            return {"status": "SPLIT", "velocity": updated_v_leader}

        
    
    def _check_saturation_flow(self, platoon_length, green_start, green_end, vf_start ,distance_to_light):
        """
        Return True if the entire platoon can cross the intersection 
        within green_start..green_end at or below self.v_max.

        For example:
        - green_start=50s, green_end=60s => 10s window
        - distance_to_light + platoon_length=100m
        - required_speed = 100m / 10s = 10 m/s
        If required_speed <= v_max => saturation = True, else False
        """
        time_window = green_end - green_start
        if green_start == 0:
            vf_start = distance_to_light / green_end
            # Green already started
            return True, vf_start

        # We'll consider that the last car must clear the stop line 
        # => distance_to_light + platoon_length
        required_speed = platoon_length / time_window
        print(f"[SAT CHECK] time_window={time_window:.2f} s, "
            f"platoon_length={platoon_length:.2f} m, "
            f"required_speed={required_speed:.2f} m/s (v_max={self.v_max:.2f}).")

        return (required_speed <= self.v_max-1), required_speed # This speed required once the leader at upper bound of feasible range
    
    def _check_saturation_flow_for_trailing_vehicles(self, platoon_length, green_start, green_end, vf_start,distance_to_light,v_saturation):
        """
        Return True if the entire platoon can cross the intersection 
        within green_start..green_end at or below self.v_max.

        For example:
        - green_start=50s, green_end=60s => 10s window
        - distance_to_light + platoon_length=100m
        - required_speed = 100m / 10s = 10 m/s
        If required_speed <= v_max => saturation = True, else False
        """
        time_window = green_end - green_start
        if green_start == 0:
            vf_start = distance_to_light / green_end
            # Green already started
            return True, vf_start

        # We'll consider that the last car must clear the stop line 
        # => distance_to_light + platoon_length
        required_speed = platoon_length / time_window
        print(f"[SAT CHECK Trailing] time_window={time_window:.2f} s, "
            f"platoon_length={platoon_length:.2f} m, "
            f"required_speed={required_speed:.2f} m/s.")

        return (required_speed <= self.v_max-1 and required_speed>=v_saturation), required_speed # This speed required once the leader at upper bound of feasible range
    # -------------------------------------------------------------
    # C) New Helper #2: Finalize the result of split or entire pass
    # -------------------------------------------------------------
    def _finalize_split_decision(self, result, traffic_light_key,eta_to_light):
        """
        Decide how to set self.ref_v and return the final dictionary.
        Also track the number of times we decided to split.
        """
        mode = result["mode"]
        vel=result["velocity"]
        print(f"[Finalize Split Decision] Ref Vel= {vel:.4f} m/s, Mode= {mode}.")
        if mode == "NONE":
            print("[Finalize Split Decision] No subplatoon can pass => fallback to v_min.")
            self.ref_v = self.v_min
            return self.ref_v, result, traffic_light_key,eta_to_light

        if mode == "ENTIRE":
            print("[Finalize Split Decision] Entire Platoon can pass")
            self.ref_v = result["velocity"]
            return self.ref_v, result, traffic_light_key,eta_to_light

        if mode == "SPLIT":
            # increment the split counter
            print(f"[Finalize Split Decision] SPLIT DECISION COUNT = {self.split_counter}")
            self.ref_v = result["velocity"]
            return self.ref_v, result, traffic_light_key,eta_to_light
        
        if mode == "WAIT":
            self.ref_v = result["velocity"]
            return self.ref_v, result, traffic_light_key,eta_to_light
        # Should not happen, fallback
        self.ref_v = self.v_min
        return self.ref_v, result, traffic_light_key,eta_to_light

    # -------------------------------------------------------------
    # E) MAIN: calculate_reference_velocity
    # -------------------------------------------------------------
    def calculate_reference_velocity(self):
        """
        Use the next (or current) green window, then decide:
         - ENTIRE platoon can pass,
         - PARTIAL-split,
         - or NONE can pass
        """
        # 1) Get current time 
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        # If start_time has not been set, record it now
        if self.start_time is None:
            self.start_time = current_time

        # Calculate how many seconds have passed since we first started
        sim_time_elapsed = current_time - self.start_time
        # 2) Add the 8-second delay check
        if sim_time_elapsed < self.init_delay:
            print("[CALC REF VEL] We are within the initial 5s offset; skipping logic.")
            return self.ref_v, {
                "mode": "WAITING",  # or "PASSING", or anything that means "not splitting yet"
                "velocity": self.ref_v,
                "front_group": [self.platoon_manager.pam.vehicle_ids],
                "rear_group": [],
                "sub_platoon": None
            }, None, 1000
        
        pam = self.platoon_manager.pam
        all_pcms = self.platoon_manager.pcms
        related_pcms = [pcm for pcm in all_pcms if pcm.vehicle_id in pam.vehicle_ids]
        leader_pcm = related_pcms[0]
        last_pcm = related_pcms[-1]

        # 2) Retrieve traffic light data
        # first_key = next(iter(self.traffic_lights))
        # tl_data = self.traffic_lights[first_key]
        # distance_to_light = tl_data['distance']
        # self.corridor_id = first_key
        # Find the traffic light with the minimum distance
        self.corridor_id, tl_data = min(
            self.traffic_lights.items(), key=lambda item: item[1]['distance'])
        distance_to_light = tl_data['distance']
        if distance_to_light - pam.platoon_length >= 300:
            print("[CALC REF VEL] Traffic Light is too far away.")
            return pam.platoon_speed, {
                "mode": "WAITING FOR SPAT",
                "velocity": pam.platoon_speed,
                "front_group": [pam.vehicle_ids],
                "rear_group": [],
                "sub_platoon": None
            }, self.corridor_id, 100
        
        if distance_to_light - pam.platoon_length < 0 or (distance_to_light-pam.platoon_length)<leader_pcm.target_speed:
            self.corridor_change = True
            print(f"[CALC REF VEL] Corridor Change Detected: {self.corridor_id}.")
        else: self.corridor_change = False

        print(f"[CALC REF VEL] Calculating reference velocity for TL: {self.corridor_id}.")
        print(f"[CALC REF VEL] Distance to Light of Entire Platoon: {distance_to_light:.5f} m.")

        # If too close, just keep the reference velocity
        if distance_to_light < pam.platoon_speed:
            print("[CALC REF VEL] Last Vehicle is too close to the traffic light.")
            return pam.platoon_speed, {
                "mode": "PASSING",
                "velocity": pam.platoon_speed,
                "front_group": [pam.vehicle_ids],
                "rear_group": [],
                "sub_platoon": None
            }, self.corridor_id, max(0.1,tl_data["distance"]/pam.platoon_speed)

        # 3) Compute feasible green windows
        feasible_windows = self._compute_feasible_green_windows()
        if not feasible_windows:
            print("[CALC REF VEL] No feasible green windows => fallback to v_min.")
            return self.ref_v, {
                "mode": "NONE",
                "velocity": None,
                "front_group": [],
                "rear_group": [],
                "sub_platoon": None
            }, self.corridor_id, None
        

            # Decide which window to use
        platoon_id = self.platoon_manager.platoon_id
        print(f"[CALC REF VEL] Platoon ID: {platoon_id}")
        if platoon_id is not None and platoon_id > 1 and len(feasible_windows) > 1:
            # Find other platoons in the same corridor
            for other_pam in self.pams:
                # print(f"[CALC REF VEL] ETA to Light: {other_pam}")
                if other_pam.platoon_id != platoon_id and other_pam.corridor_id == pam.corridor_id and (other_pam.eta_to_light < pam.eta_to_light):
                    print(f"[CALC REF VEL] Checking platoon {other_pam.platoon_id} in the same corridor.")
                    # Calculate distance between the two platoons, leader to leader
                    distance_to_other = math.sqrt(
                        (pam.platoon_position[0] - other_pam.platoon_position[0]) ** 2 +
                        (pam.platoon_position[1] - other_pam.platoon_position[1]) ** 2
                    )
                    # What will happen if they are too close? Let's align them quasi merge!
                    for window in feasible_windows:
                        (vf_start_candidate, vf_end_candidate, green_start_candidate, green_end_candidate) = window
                        print(f"[CALC REF VEL] Checking Time window: {green_start_candidate:.2f} - {green_end_candidate:.2f}")
                        print(f"[CALC REF VEL] Checking Velocity window: {vf_start_candidate:.2f} - {vf_end_candidate:.2f}")
                        if vf_end_candidate < other_pam.platoon_speed:
                            print(f"[CALC REF VEL] Found suitable window for other platoon.")
                            (vf_start, vf_end, green_start, green_end) = window
                            print(f"[CALC REF VEL] Green Window for platoon id {platoon_id} => start: {green_start:.2f}, end: {green_end:.2f}")
                            break
                        # elif distance_to_other < 20 and other_pam.platoon_length < 20 :
                        #     print(f"[CALC REF VEL] Other Platoon is too close, assigning default.")
                        #     (vf_start, vf_end, green_start, green_end) = feasible_windows[0]
                        #     break
                    break
                else:
                    print("[CALC REF VEL] Other Platoon behind in the same corridor, assigning default.")
                    (vf_start, vf_end, green_start, green_end) = feasible_windows[0]
        else:
            # Ensure fallback feasible window is valid
            print("[CALC REF VEL] No other platoon in the same corridor or the other platoon under 10 sec of traffic light, assigning default.")
            (vf_start, vf_end, green_start, green_end) = feasible_windows[0]
            print(f"[CALC REF VEL] Green Window for platoon id {platoon_id} => start: {green_start:.2f}, end: {green_end:.2f}, vf_start: {vf_start:.2f}, vf_end: {vf_end:.2f}")

        # print(f"Green Window => start: {green_start:.2f}, end: {green_end:.2f}")
        feasible_range = (vf_start, vf_end)

        #  saturation check
        bool_sat, v_saturation = self._check_saturation_flow(pam.platoon_length, green_start, green_end, vf_start, distance_to_light)
        print(f"[CALC REF VEL] Saturation check {bool_sat} with v_saturation={v_saturation:.2f} m/s.")
        # 4) Check entire platoon feasibility 
        if feasible_range and bool_sat and (pam.platoon_speed > v_saturation or self.corridor_change):
            print(f"[CALC REF VEL] Entire Platoon feasibility check.")
            entire_check = self._check_entire_platoon_feasibility(
                feasible_range,
                distance_to_light,
                green_start,
                green_end,
                related_pcms,
                pam,
                v_saturation
            )
            if entire_check["status"] == "ENTIRE":
                # Entire can pass
                
                self.ref_v = entire_check["velocity"]
                print(f"[CALC REF VEL] Entire platoon => ENTIRE, Ref Vel= {self.ref_v:.4f} m/s.")
                return self.ref_v, {
                    "mode": "ENTIRE",
                    "velocity": self.ref_v,
                    "front_group": related_pcms,
                    "rear_group": [],
                    "sub_platoon": None
                }, self.corridor_id, tl_data["distance"]/self.ref_v

            elif entire_check["status"] == "SPLIT":
                print("[CALC REF VEL] Last car not feasible => partial split attempt.")
                result = self.split_for_first_green_window(
                    pam=pam,
                    pcms=related_pcms,
                    distance_to_light=distance_to_light,
                    green_start=green_start,
                    green_end=green_end,
                    feasible_range=feasible_range,
                    v_saturation=v_saturation
                )
                eta_to_light = max(0.1,tl_data["distance"]/result["velocity"])
                print(f"[CALC REF VEL] Partial split => Ref Vel= {result['velocity']:.4f} m/s.")
                return self._finalize_split_decision(result, self.corridor_id,eta_to_light)

            else:
                print("[CALC REF VEL] Leader not feasible => partial split attempt.")
                (vf_start, vf_end, green_start, green_end) = feasible_windows[1]
                feasible_range = (vf_start, vf_end)
                result = self.split_for_first_green_window(
                    pam=pam,
                    pcms=related_pcms,
                    distance_to_light=distance_to_light,
                    green_start=green_start,
                    green_end=green_end,
                    feasible_range=feasible_range,
                    v_saturation=v_saturation
                )
                eta_to_light = max(0.1,tl_data["distance"]/result["velocity"])
                return self._finalize_split_decision(result, self.corridor_id,eta_to_light)

        # 5) If feasible_range is None => partial or none
        print("[CALC REF VEL] feasible_range or saturatin check fails => partial-split attempt.")
        result = self.split_for_first_green_window(
            pam=pam,
            pcms=related_pcms,
            distance_to_light=distance_to_light,
            green_start=green_start,
            green_end=green_end,
            feasible_range=feasible_range,
            v_saturation=v_saturation
        )
        eta_to_light = max(0.1,tl_data["distance"]/result["velocity"])
        return self._finalize_split_decision(result, self.corridor_id,eta_to_light)

    def is_red_light_ahead(self, vehicle_location):
        # Check traffic lights for red state and proximity
        tl_id, tl_data = min(self.traffic_lights.items(), key=lambda item: item[1]['distance'])
        print(f"[TL MANAGER] Closest Traffic Light: {tl_id},{tl_data}.")
        if tl_data['current_state'] == carla.TrafficLightState.Red and tl_data['distance'] < 12.0:
                return True, tl_id
        return False, None
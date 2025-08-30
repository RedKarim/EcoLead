# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2025, FZI Forschungszentrum Informatik
# Copyright (c) 2025, MAHDYAR KARIMI
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
# !\ traffic_manager.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
# Modified by: MAHDYAR KARIMI, 2025-08-31
# ---------------------------------------------------------------------

import csv
import random
import math
from datetime import datetime
from platoon_manager import PlatoonManager
from carla_components.behavior_agent import BehaviorAgent
from carla_components.local_planner import RoadOption

class VehicleTrafficManager:
    def __init__(self, client, world, waypoints, scenario, behaviour, ego_vehicle, traffic_light_manager, num_behind, num_vehicles=20, spacing= 7.5, front_vehicle_autopilot=False):
        self.client = client
        self.world = world
        self.num_vehicles = num_vehicles
        self.waypoints = waypoints
        self.traffic_vehicles = []
        self.front_vehicle = None
        self.behind_vehicles = []
        self.traffic_manager = self.client.get_trafficmanager(8000)
        self.vehicle_bps = self.world.get_blueprint_library().filter('vehicle.*mini*')
        self.csv_files = {}
        self.map = world.get_map()
        self.behaviour = behaviour
        self.spacing = spacing
        self.scenario = scenario
        self.autopilot = front_vehicle_autopilot
        self.ego_vehicle = ego_vehicle
        self.num_behind = num_behind
        self.front_vehicle_speed = 0.0
        self.front_vehicle_transform = None
        self.platoon_managers = []  # Initialize list to manage multiple platoons
        self.traffic_light_manager = traffic_light_manager
        self.setup_traffic_manager()
        self.start_time = None

    def setup_traffic_manager(self):
        self.traffic_manager.set_synchronous_mode(True)
        # self.traffic_manager.global_percentage_speed_difference(5.0)
        self.traffic_manager.set_random_device_seed(42)
        self.traffic_manager.set_hybrid_physics_mode(False)
        self.traffic_manager.set_hybrid_physics_radius(500.0)

    def configure_vehicle(self, vehicle, autopilot=True):
        vehicle.set_autopilot(autopilot, self.traffic_manager.get_port())
        # Set random behavior parameters
        speed_percentage = random.uniform(-30, 10)  # Adjust as needed
        self.traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_percentage)
        self.traffic_manager.distance_to_leading_vehicle(vehicle, random.uniform(2.0, 8.0))

        # Disable lane change behavior
        self.traffic_manager.auto_lane_change(vehicle, False)
        self.traffic_manager.random_left_lanechange_percentage(vehicle, 0)
        self.traffic_manager.random_right_lanechange_percentage(vehicle, 0)


    def get_front_vehicle_status(self, ego_vehicle):
        """Retrieve the status of the front vehicle, including distance and velocity."""
        if self.front_vehicle and self.front_vehicle.is_alive:
            front_vehicle_location = self.front_vehicle.get_location()
            front_vehicle_velocity = self.front_vehicle.get_velocity()
            speed = math.sqrt(front_vehicle_velocity.x**2 + front_vehicle_velocity.y**2 + front_vehicle_velocity.z**2)
            distance = ego_vehicle.get_location().distance(front_vehicle_location)
              # Log to CSV
            csv_file = self.csv_files.get('front_vehicle')
            if csv_file:
                writer = csv.writer(csv_file)
                timestamp = self.world.get_snapshot().timestamp.elapsed_seconds
                writer.writerow([timestamp, speed, distance])
            return {
                'location': front_vehicle_location,
                'speed': speed,
                'distance': distance
            }
        return None
    
    def spawn_scenario(self):
        """Spawn vehicles for a specific scenario."""
        if self.scenario == 'packleader':
            self.spawn_pack()
            # Initialize platoon for packleader
            vehicles_list = [self.ego_vehicle] + [v['vehicle'] for v in self.behind_vehicles]
            platoon_manager = PlatoonManager(vehicles=vehicles_list, leader_id=self.ego_vehicle.id, traffic_manager=self, behaviour_agents=self.behind_vehicles,platoon_id=1)
            self.platoon_managers.append(platoon_manager)
            self.traffic_light_manager.set_platoon_manager(platoon_manager)

        elif self.scenario == 'following':
            self.spawn_following_scenario()
            # Initialize platoon for following
            vehicles_list = [self.ego_vehicle]
            if self.front_vehicle:
                vehicles_list.append(self.front_vehicle)
            platoon_manager = PlatoonManager(vehicles=vehicles_list, leader_id=self.ego_vehicle.id, traffic_manager=self)
            self.platoon_managers.append(platoon_manager)

        elif self.scenario == 'idm_packleader':
            self.spawn_idm_pack()
            # Initialize platoon for IDM packleader (similar to MPC packleader)
            vehicles_list = [self.ego_vehicle] + [v['vehicle'] for v in self.behind_vehicles]
            platoon_manager = PlatoonManager(vehicles=vehicles_list, leader_id=self.ego_vehicle.id, traffic_manager=self, behaviour_agents=self.behind_vehicles, platoon_id=1)
            self.platoon_managers.append(platoon_manager)
            self.traffic_light_manager.set_platoon_manager(platoon_manager)
        else:
            print(f"Unknown scenario: {self.scenario}")

    def spawn_idm_scenario(self):
        """Spawn IDM scenario with following vehicles."""
        self.spawn_scenario()  # Use existing spawn logic

    def spawn_idm_pack(self):
        """Spawn vehicles behind the ego vehicle for IDM pack leader scenario."""
        # Use the same logic as spawn_pack
        self.spawn_pack()

    def spawn_pack(self):
        """Spawn vehicles behind the ego vehicle as pack leader scenario."""
        previous_vehicle = self.ego_vehicle
         # Fixed number of vehicles behind. Behind 1 is the car, which follows the ego vehicle
        for i in range(self.num_behind): # Start from the last vehicle (behind_5) and go backwards
            # if i==0:
            #     index = len(self.waypoints) - 3
            # else:
            index = len(self.waypoints) - ((i+1) * (self.spacing))

            # Wrap around to create circular waypoint continuity
            # spawn_index = index % len(self.waypoints)
            spawn_index = index
            spawn_transform = self.waypoints[index]
            spawn_transform.location.z += 2  # Adjust height to prevent collisions
            vehicle_bp = random.choice(self.vehicle_bps)
            role_name = f'behind_{i+1}'
            vehicle_bp.set_attribute('role_name', role_name)

            try:
                vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
                if vehicle:
                    print("Creating Basic Agents with global plan for behind vehicles.")
                     # Initialize Behaviour for the vehicle
                    agent = BehaviorAgent(vehicle, behavior=self.behaviour)

                    print(f"Spawned behind vehicle {i+1} at index {spawn_index}")
                    # Create a circular global plan
                    global_plan = (
                        [(self.map.get_waypoint(wp.location), RoadOption.LANEFOLLOW) for wp in self.waypoints[spawn_index:]] +
                        [(self.map.get_waypoint(wp.location), RoadOption.LANEFOLLOW) for wp in self.waypoints[:spawn_index]]
                    )
                    agent.set_global_plan(global_plan)

                    self.behind_vehicles.append({'id':vehicle.id,'vehicle': vehicle, 'agent': agent, 'role_name': role_name, 'following':previous_vehicle})

                    # Update the previous vehicle to the current one for the next iteration
                    previous_vehicle = vehicle
                   # Open or create a CSV file for logging
                    csv_file = open(f'vehicle_{vehicle.id}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
                    writer = csv.writer(csv_file)
                    writer.writerow(['Time', 'Velocity','Platoon ID','Reference Velocity','Distance to Front Vehicle'])
                    self.csv_files[f'velocity_{vehicle.id}'] = csv_file
                else:
                    print(f"Failed to spawn behind vehicle {i+1}.")
            except RuntimeError as e:
                print(f"Failed to spawn behind vehicle {i+1}: {e}")
                
    def update_pack(self,ego_agent,current_tick):
        """Update status and control for behind vehicles."""
        route_end = False
        optimal_a=0
        for platoon_manager in self.platoon_managers:
            leader = False
            mpc_agent = False
            print("---------Update PACK-------------")
            self.traffic_light_manager.set_platoon_manager(platoon_manager)
            print(f"[Traffic Manager] Updating Platoon ID: {platoon_manager.platoon_id}")
            distances = [100] # Leader has no front vehicles
            # if the platoon is not the first one(MPC Agent), get the reference velocity from the traffic light manager
            if platoon_manager.platoon_id > 1:
                # print(f"[Traffic Manager] Calculating Reference Velocity for Non MPC Agent")
                ego_vehicle = platoon_manager.behind_vehicles[0]['vehicle']
                # print(f"[Traffic Manager] Ego Vehicle:{ego_vehicle}")
                ego_velocity = ego_vehicle.get_velocity()
                ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2) # m/s
                # print(f"[Traffic Manager] Ego Vehicle Speed: {ego_speed:.2f} ,m/s")
                last_vehicle = platoon_manager.behind_vehicles[-1]['vehicle']
                last_vehicle_location = last_vehicle.get_location()
                # print(f"[Traffic Manager] Last Vehicle Location: {last_vehicle_location}")
                self.traffic_light_manager.update_traffic_lights(last_vehicle_location, current_tick)
                ref_v, platoon_status,tl_id,eta_to_light = self.traffic_light_manager.calculate_reference_velocity()
                ego_speed = ref_v
            else:
                mpc_agent = True
                # print(f"[Traffic Manager] Calculating Reference Velocity for MPC Agent")
                if len(platoon_manager.pam.vehicle_ids)>1:
                    last_vehicle = platoon_manager.behind_vehicles[-1]['vehicle']
                    last_vehicle_location = last_vehicle.get_location()
                else: last_vehicle_location = self.ego_vehicle.get_location()
                # print(f"[Traffic Manager] Last Vehicle Location: {last_vehicle_location}")
                self.traffic_light_manager.update_traffic_lights(last_vehicle_location, current_tick)
                ego_velocity = self.ego_vehicle.get_velocity()
                ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2) # m/s
                ref_v, platoon_status,tl_id,eta_to_light  = self.traffic_light_manager.calculate_reference_velocity()
                if route_end:
                    ref_v = 0
                    ego_speed = 0
            # print(f"[Traffic Manager] Ego Vehicle Speed: {ego_speed:.2f} ,m/s")
            for i, vehicle_data in enumerate(platoon_manager.behind_vehicles):
                vehicle = vehicle_data['vehicle']
                agent = vehicle_data['agent']
                following_vehicle = vehicle_data['following']
                id = vehicle_data['id']
                # print(f"[Traffic Manager] Vehicle ID: {id}")
                if not vehicle.is_alive:
                    # print(f"Vehicle behind_{i+1} is destroyed.")
                    continue
                # Calculate distance to the following vehicle
                if platoon_manager.leader_id == id:
                    # If the vehicle is the leader, set the distance to a large value
                    # print(f"[Traffic Manager] Leader: {platoon_manager.leader_id}, Following: {following_vehicle.id}, self id: {id}")
                    distance_to_packleader = 100
                    distance = following_vehicle.get_location().distance(vehicle.get_location())
                    leader = True
                    following_vehicle_speed = ego_speed
                else: 
                    distance = following_vehicle.get_location().distance(vehicle.get_location())
                    following_vehicle_velocity = following_vehicle.get_velocity()
                    following_vehicle_speed = math.sqrt(following_vehicle_velocity.x**2 + following_vehicle_velocity.y**2)
                    distance_to_packleader = platoon_manager.traffic_manager.ego_vehicle.get_location().distance(vehicle.get_location())
                    distances.append(distance)
                velocity = vehicle.get_velocity()
                absolute_velocity = math.sqrt(velocity.x**2 + velocity.y**2)
                timestamp = self.world.get_snapshot().timestamp.elapsed_seconds
                print(f"[Traffic Manager] Behind_{i+1}: Distance to following vehicle: {distance:.2f}, Vehicle Velocity: {absolute_velocity:.2f}, Following Vehicle Velocity: {following_vehicle_speed:.2f} , Ref V: {ref_v} m/s")
                # Write to CSV
                csv_file = self.csv_files.get(f'velocity_{id}')
                if csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow([timestamp, absolute_velocity,platoon_manager.platoon_id,ref_v,distance])
                # Update agent speed limit to match ego vehicle's speed
                # print(f"[Traffic Manager] Updating Agent Information wit Reference Speed: {ref_v} m/s,{ref_v*3.6} km/h")
                agent._update_information(ego_vehicle_speed=ref_v*3.6)
                # agent.follow_speed_limits(ego_speed*3.6)
                # print(f"[Traffic Manager] Follower: {vehicle_data['role_name']}")
                # print(f"[Traffic Manager] Leader: {following_vehicle.attributes['role_name']}")
                control = agent.run_step(following_vehicle,distance,distance_to_packleader,following_vehicle_speed,leader,debug=True)
                leader = False
                vehicle.apply_control(control)
            if mpc_agent:
                optimal_a, route_end,ref_v_mpc = ego_agent.on_tick(ref_v,False)
                mpc_agent = False
            platoon_manager.update_platoon(platoon_status,tl_id,ref_v,eta_to_light,distances=distances)
            if route_end:
                route_end = False
                self.platoon_managers.pop(0)
            if platoon_status["mode"]=="SPLIT":
                self.platoon_managers.append(platoon_status["sub_platoon"])
        return optimal_a,ref_v,ref_v_mpc

    def update_idm_pack(self, ego_agent, current_tick):
        """Update status and control for behind vehicles in IDM mode."""
        route_end = False
        optimal_a = 0
        ref_v_mpc = 0
        
        for platoon_manager in self.platoon_managers:
            leader = False
            idm_agent = False
            print("---------Update IDM PACK-------------")
            self.traffic_light_manager.set_platoon_manager(platoon_manager)
            print(f"[Traffic Manager IDM] Updating Platoon ID: {platoon_manager.platoon_id}")
            distances = [100]  # Leader has no front vehicles
            
            # For IDM mode, the ego vehicle uses IDM agent, followers use CACC/PID
            if platoon_manager.platoon_id == 1:  # Main platoon with IDM leader
                idm_agent = True
                
                # Update traffic lights for ego vehicle (IDM agent)
                ego_location = self.ego_vehicle.get_location()
                print(f"[Traffic Manager IDM] Updating traffic lights for ego vehicle at location ({ego_location.x:.1f}, {ego_location.y:.1f})")
                self.traffic_light_manager.update_traffic_lights(ego_location, current_tick)
                
                ego_velocity = self.ego_vehicle.get_velocity()
                ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2)  # m/s
                
                # IDM agent handles its own traffic light logic
                optimal_a, route_end, idm_target_speed = ego_agent.on_tick(self.traffic_light_manager)
                ref_v = ego_speed  # Use current ego speed as reference for followers
                ref_v_mpc = idm_target_speed  # Use IDM target speed for MPC reference
                
                # Platoon status for IDM (compatible with platoon_manager.update_platoon)
                platoon_status = {
                    "mode": "STABLE", 
                    "velocity": ref_v,
                    "front_group": [platoon_manager.pam.vehicle_ids],
                    "rear_group": [],
                    "sub_platoon": None
                }
                tl_id = 13  # Default traffic light
                eta_to_light = 100  # Default ETA
            
            # Update following vehicles (each with individual traffic light calculations)
            for i, vehicle_data in enumerate(platoon_manager.behind_vehicles):
                vehicle = vehicle_data['vehicle']
                agent = vehicle_data['agent']
                following_vehicle = vehicle_data['following']
                id = vehicle_data['id']
                
                # Update traffic lights for this specific follower vehicle
                follower_location = vehicle.get_location()
                print(f"[Traffic Manager IDM] Updating traffic lights for follower {id} at location ({follower_location.x:.1f}, {follower_location.y:.1f})")
                self.traffic_light_manager.update_traffic_lights(follower_location, current_tick)
                
                if not vehicle.is_alive:
                    continue
                
                # Calculate distance to the following vehicle
                if platoon_manager.leader_id == id:
                    # If the vehicle is the leader (ego), skip control (IDM agent handles it)
                    distance_to_packleader = 100
                    distance = following_vehicle.get_location().distance(vehicle.get_location())
                    leader = True
                    following_vehicle_speed = ego_speed
                    continue  # Skip control application for ego vehicle
                else:
                    distance = following_vehicle.get_location().distance(vehicle.get_location())
                    following_vehicle_velocity = following_vehicle.get_velocity()
                    following_vehicle_speed = math.sqrt(following_vehicle_velocity.x**2 + following_vehicle_velocity.y**2)
                    distance_to_packleader = platoon_manager.traffic_manager.ego_vehicle.get_location().distance(vehicle.get_location())
                    distances.append(distance)
                
                velocity = vehicle.get_velocity()
                absolute_velocity = math.sqrt(velocity.x**2 + velocity.y**2)
                timestamp = self.world.get_snapshot().timestamp.elapsed_seconds
                print(f"[Traffic Manager IDM] Behind_{i+1}: Distance to following vehicle: {distance:.2f}, Vehicle Velocity: {absolute_velocity:.2f}, Following Vehicle Velocity: {following_vehicle_speed:.2f}, Ref V: {ref_v} m/s")
                
                # Write to CSV
                csv_file = self.csv_files.get(f'velocity_{id}')
                if csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow([timestamp, absolute_velocity, platoon_manager.platoon_id, ref_v, distance])
                
                # Update agent speed limit to match ego vehicle's speed
                agent._update_information(ego_vehicle_speed=ref_v*3.6)
                
                # Pass traffic light manager to behavior agent
                agent._traffic_light_manager = self.traffic_light_manager
                
                # Apply CACC/PID control for following vehicles
                control = agent.run_step(following_vehicle, distance, distance_to_packleader, following_vehicle_speed, leader, debug=True)
                leader = False
                vehicle.apply_control(control)
            
            # Update platoon status
            platoon_manager.update_platoon(platoon_status, tl_id, ref_v, eta_to_light, distances=distances)
            
            # Handle route completion (same as MPC mode)
            if route_end:
                print("[Traffic Manager IDM] Route completed, removing platoon manager")
                route_end = False
                self.platoon_managers.pop(0)
                # If no more platoons, the simulation should end
                if not self.platoon_managers:
                    print("[Traffic Manager IDM] All platoons completed, simulation ending")
                    break  # Exit the platoon loop
            
        return optimal_a, ref_v, ref_v_mpc
    
    def spawn_following_scenario(self):
        """Spawn a front vehicle and random traffic for the following scenario."""
        self.csv_files['front_vehicle'] = open(f'front_vehicle_status_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
        writer = csv.writer(self.csv_files['front_vehicle'])
        writer.writerow(['Time', 'Velocity', 'Distance'])

        # Spawn front vehicle
        spawn_transform = self.waypoints[self.spacing]
        spawn_transform.location.z += 2
        vehicle_bp = random.choice(self.vehicle_bps)
        vehicle_bp.set_attribute('role_name', 'front_vehicle')

        try:
            front_vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
            if front_vehicle:
                self.front_vehicle = front_vehicle
                print(f"Spawned front vehicle at {spawn_transform}.")
                if not self.autopilot:
                # Use BasicAgent for the front vehicle
                    self.front_vehicle_agent = BehaviorAgent(front_vehicle, behavior=self.behaviour)

                    # Convert the waypoints to a global plan and set it for the agent
                    global_plan = [(self.map.get_waypoint(wp.location), RoadOption.LANEFOLLOW) for wp in self.waypoints[self.spacing:]]
                    self.front_vehicle_agent.set_global_plan(global_plan)
                    self.front_vehicle_agent.set_target_speed(50)
                    print("Created Behaviour Agent for front vehicle.")
                else:
                    print("Creating Autopilot for front vehicle")
                    self.configure_vehicle(front_vehicle)
            else:
                print("Failed to spawn front vehicle.")
        except RuntimeError as e:
            print(f"Error spawning front vehicle: {e}")

        # Spawn random traffic
        num_random_vehicles = min(self.num_vehicles - 2, 10)
        for _ in range(num_random_vehicles):
            spawn_index = random.randint(0, len(self.waypoints) - 1)
            spawn_transform = self.waypoints[spawn_index]
            spawn_transform.location.z += 2

            vehicle_bp = random.choice(self.vehicle_bps)

            try:
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
                if vehicle:
                    self.traffic_vehicles.append(vehicle)
                    self.configure_vehicle(vehicle)
                    print(f"Spawned random vehicle at index {spawn_index} with autopilot.")
            except RuntimeError as e:
                print(f"Error spawning random vehicle: {e}")

    def update_front_vehicle(self):
        """Update and control the front vehicle"""
        if hasattr(self, 'front_vehicle_agent') and self.front_vehicle_agent:
            self.front_vehicle_agent._update_information(15)
            control = self.front_vehicle_agent.run_step(self.front_vehicle, distance=100, debug=True)
            self.front_vehicle.apply_control(control)

            # Assign front vehicle's velocity and location
            front_vehicle_velocity = self.front_vehicle.get_velocity()
            self.front_vehicle_speed = math.sqrt(front_vehicle_velocity.x**2 + front_vehicle_velocity.y**2 + front_vehicle_velocity.z**2)
            self.front_vehicle_transform= self.front_vehicle.get_transform()
            print(self.front_vehicle_transform)
        else:
            print("No Behaviour Agent found for the front vehicle.")

        for platoon_manager in self.platoon_managers:
            platoon_manager.update_platoon()

    def cleanup(self):
        """Clean up all vehicles."""
        for vehicle_data in self.behind_vehicles:
            vehicle = vehicle_data['vehicle']
            if vehicle.is_alive:
                vehicle.destroy()
        self.behind_vehicles.clear()

        if self.front_vehicle and self.front_vehicle.is_alive:
            self.front_vehicle.destroy()
            self.front_vehicle = None

        for vehicle in self.traffic_vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        self.traffic_vehicles.clear()
          # Close all CSV files
        for csv_file in self.csv_files.values():
            csv_file.close()
        self.csv_files.clear()

        for platoon_manager in self.platoon_managers:
            platoon_manager.cleanup()

        print("All vehicles and files cleaned up.")

    def get_behind_vehicles(self):
        """Return the list of behind vehicles."""
        return self.behind_vehicles

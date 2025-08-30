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
# !\ main.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
# Modified by: MAHDYAR KARIMI, 2025-08-31
# ---------------------------------------------------------------------

import carla
import pygame
import math
import yaml
import sys
from mpc_agent import MPCAgent
from camera_manager import CameraManager
from display_manager import DisplayManager
from traffic_light_manager import TrafficLightManager
from traffic_manager import VehicleTrafficManager
import utils
from pid_agent import PIDAgent
from idm_agent import IDMAgent


def main():
    # コマンドライン引数をチェック
    if len(sys.argv) != 2:
        print("Usage: python3 main.py [MPC|IDM]")
        print("  MPC: Run with MPC controller and platoon scenario")
        print("  IDM: Run with IDM controller, ego vehicle only")
        sys.exit(1)
    
    mode = sys.argv[1].upper()
    if mode not in ['MPC', 'IDM']:
        print("Error: Mode must be either 'MPC' or 'IDM'")
        sys.exit(1)
    
    print(f"Running simulation in {mode} mode")
    
    pygame.init()
    clock = pygame.time.Clock()
    display_size = (800, 600)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Set synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1  # 10 FPS
    settings.substepping = True
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 15
    world.apply_settings(settings)

    vehicle = None
    camera_manager = None
    display_manager = None
    traffic_light_manager = None

    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*model3*')[0]

        # Set the role_name attribute to identify the vehicle
        vehicle_bp.set_attribute('role_name', 'ego_vehicle')
        # Read waypoints from CSV
        csv_file = 'config/route.csv'  # Replace with your CSV file path
        transforms = utils.read_csv_waypoints(csv_file)
        
        # Convert Transforms to Waypoints
        waypoints = []
        for transform in transforms:
            waypoint = world.get_map().get_waypoint(transform.location)
            waypoints.append(waypoint)
        # Spawn the ego vehicle at the first waypoint's location
        ego_vehicle_spawn_point = transforms[0]
        print(f"Spawning ego vehicle at: {ego_vehicle_spawn_point}")
        ego_vehicle = world.spawn_actor(vehicle_bp, ego_vehicle_spawn_point)
        physics_control = ego_vehicle.get_physics_control()
        print("Physics Control,",physics_control)

        # Initialize Traffic Light Manager FIRST
        traffic_lights_settings = {
            13: {
                'initial_state': carla.TrafficLightState.Red,
                'green_time': 10.0,
                'red_time': 20.0,
            },
            11: {
                'initial_state': carla.TrafficLightState.Green,
                'green_time': 10.0,
                'red_time': 10.0,
            },
            20: {
                'initial_state': carla.TrafficLightState.Red,
                'green_time': 12.0,
                'red_time': 20,
            },
        }

        # Initialize Traffic Light Manager
        traffic_light_manager = TrafficLightManager(client, traffic_lights_settings, waypoints)

        # モードに基づいてシナリオとコントローラーを設定
        if mode == 'MPC':
            scenario = 'packleader'  # MPC mode: full platoon scenario
            ego_vehicle_controller = 'mpc'
            behavior = 'normal'
            vehicle_traffic_manager = VehicleTrafficManager(
                client, world, transforms, scenario, behavior, ego_vehicle, 
                traffic_light_manager, num_behind=7, num_vehicles=3, spacing=36, 
                front_vehicle_autopilot=False
            )
            # Spawn platoon scenario
            vehicle_traffic_manager.spawn_scenario()
        else:  # IDM mode
            scenario = 'idm_packleader'  # IDM mode: ego vehicle with followers
            ego_vehicle_controller = 'idm'
            behavior = 'normal'
            vehicle_traffic_manager = VehicleTrafficManager(
                client, world, transforms, scenario, behavior, ego_vehicle, 
                traffic_light_manager, num_behind=7, num_vehicles=3, spacing=36, 
                front_vehicle_autopilot=False
            )
            # Spawn IDM scenario with following vehicles
            vehicle_traffic_manager.spawn_idm_scenario()

        # Initialize camera manager
        camera_manager = CameraManager(ego_vehicle, display_size)

        # Initialize display manager
        display_manager = DisplayManager(display_size)

        # Get traffic light actors
        traffic_light_actors = traffic_light_manager.get_traffic_lights()
        # Load configuration
        with open("config/config.yaml", 'r') as file:
            config = yaml.safe_load(file)
         # Initialize the ego vehicle Controller agent
        if ego_vehicle_controller == 'pid':
            ego_agent = PIDAgent(ego_vehicle, transforms, behavior)
        elif ego_vehicle_controller == 'idm':
            ego_agent = IDMAgent(ego_vehicle, transforms)
        else:  # mpc
            ego_agent = MPCAgent(ego_vehicle, config)

        total_distance = 0
        prev_location = transforms[0].location
        prev_speed = 0

        paused = False
        while True:
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        # Toggle paused state
                        paused = not paused
                        print("Simulation paused" if paused else "Simulation resumed")
                    
            # Skip simulation updates if paused
            if paused:
                continue
            world.tick()
            print("---------------MAIN---------------------")
            velocity = ego_vehicle.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2)  # unit m/s 
            print(f"Current Speed: {speed:.2f} m/s")
            current_tick = world.get_snapshot().frame  # Get the current tick
            # traffic_light_manager.update_traffic_lights(ego_vehicle, current_tick)
            
            # Update controller
            if ego_vehicle_controller == 'pid':
                control = ego_agent.run_step(traffic_light_manager)  # Use ref_v as target speed
                ego_vehicle.apply_control(control)
                # Testing
                optimal_a = 1.0
                ref_v = speed
                ref_v_mpc = speed
            elif ego_vehicle_controller == 'idm':
                # IDMモード：信号機状態を更新してIDMエージェントで制御 + フォロワー車両管理
                if scenario == 'idm_packleader':
                    # IDM ego + following vehicles scenario
                    optimal_a, ref_v, ref_v_mpc = vehicle_traffic_manager.update_idm_pack(ego_agent, current_tick)
                    
                    # Check if all platoons are completed
                    if not vehicle_traffic_manager.platoon_managers:
                        print("[Main] All platoons completed in IDM mode, ending simulation")
                        break  # Exit the main simulation loop
                else:
                    # IDM ego only scenario
                    ego_location = ego_vehicle.get_location()
                    traffic_light_manager.update_traffic_lights(ego_location, current_tick)
                    optimal_a, route_end, target_speed = ego_agent.on_tick(traffic_light_manager)
                    ref_v = speed  # 現在の速度を参照速度として使用
                    ref_v_mpc = target_speed
                    
                    # Check if route is completed for single IDM vehicle
                    if route_end:
                        print("[Main] IDM route completed, ending simulation")
                        break  # Exit the main simulation loop
            else:  # MPC mode
                if scenario == 'packleader':
                # Update all platoons through their respective PlatoonManager instances
                    optimal_a, ref_v,ref_v_mpc =vehicle_traffic_manager.update_pack(ego_agent,current_tick)
                    
                    # Check if all platoons are completed (same logic as IDM)
                    if not vehicle_traffic_manager.platoon_managers:
                        print("[Main] All platoons completed in MPC mode, ending simulation")
                        break  # Exit the main simulation loop
                else:
                    print("No Scenario just ego vehicle with MPC...")
                    ego_location = ego_vehicle.get_location()
                    traffic_light_manager.update_traffic_lights(ego_location, current_tick)
                    ref_v, _, _ = traffic_light_manager.calculate_reference_velocity(speed)
                    optimal_a = ego_agent.on_tick(ref_v)
            
            ## ----------------- End of Controller Update ----------------- ##
            # Calculation of travel distance
            vehicle_state = ego_agent.get_vehicle_state()
            curr_location = ego_vehicle.get_location()
            delta_distance = curr_location.distance(prev_location)
            total_distance += delta_distance
            prev_location = curr_location
            
            acceleration_carla = ego_vehicle.get_acceleration()
            # Ignore small fluctuations in speed
            if abs(speed - prev_speed) < 1e-3:
                calculated_acceleration = 0.0  # Treat as no acceleration
            else:
                calculated_acceleration = (speed - prev_speed) / 0.1
            print(f"Calculating Acceleration in Main: {calculated_acceleration:.2f} m/s^2")

            # Get camera image
            camera_surface = camera_manager.get_camera_surface()
            # Update previous speed for the next tick
            prev_speed = speed
            fuel_consumption, a_net = utils.calculate_fuel_consumption(speed, calculated_acceleration)
            timestamp = world.get_snapshot().timestamp.elapsed_seconds
                        # Get traffic light states for display
            traffic_light_states = {}
            for tl_id, tl_data in traffic_light_actors.items():
                tl_actor = tl_data['actor']  # Access the actual CARLA traffic light actor
                state = tl_actor.get_state()
                state_str = 'Red' if state == carla.TrafficLightState.Red else 'Green'
                traffic_light_states[tl_id] = state_str
            important_data = {
                'Timestamp': timestamp,
                'Fuel Consumption': fuel_consumption,
                'Current Velocity': speed,
                'Acceleration': acceleration_carla.y,
                'Calculated Acceleration': calculated_acceleration,
                'MPC Net Acceleration': a_net,
                'Reference Velocity': ref_v_mpc,
                'Travel Distance': total_distance,
                'Optimal a': optimal_a,
                'Light State 13':traffic_light_states.get(13),
                'Light State 11':traffic_light_states.get(11),
                'Light State 20':traffic_light_states.get(20)
            }
            utils.write_data_to_csv([important_data])
            # Render display
            display_manager.render(camera_surface, vehicle_state, traffic_light_states)
            clock.tick(10)

    finally:
        # Clean up
        if traffic_light_manager is not None:
            traffic_light_manager.stop()
        if camera_manager is not None:
            camera_manager.destroy()
        if ego_vehicle is not None:
            ego_vehicle.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if display_manager is not None:
            display_manager.destroy()
        if vehicle_traffic_manager is not None:
            vehicle_traffic_manager.cleanup()
        pygame.quit()
        print('Simulation ended and actors destroyed.')

if __name__ == '__main__':
    main()

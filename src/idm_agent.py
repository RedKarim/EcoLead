# Copyright (c) 2025, MAHDYAR KARIMI
# Author: MAHDYAR KARIMI, 2025-08-31 

"""
IDM Agent for CARLA simulation
Implements Intelligent Driver Model with traffic light awareness
"""

import carla
import math
import numpy as np
from IDM import IDM
from carla_components.local_planner import LocalPlanner, RoadOption


class IDMAgent:
    """
    Intelligent Driver Model agent for ego vehicle control.
    Combines IDM longitudinal control with CARLA LocalPlanner for lateral control.
    """
    
    # IDM parameters (aggressive settings for urban driving)
    DESIRED_TIME_GAP = 1.3      # 希望時間間隔 (s)
    DESIRED_VELOCITY = 17       # 希望速度 (m/s)
    MIN_SPACING = 0.5           # 最小車間距離 (m)
    MAX_ACCELERATION = 2.5      # 最大加速度 (m/s²)
    COMFORTABLE_DECEL = 2.5     # 快適減速度 (m/s²)
    VEHICLE_LENGTH = 1.5        # 車長 (m) - IDMと合わせる
    
    # Control parameters
    ACCELERATION_LIMITS = (-5.0, 4.0)      # 加速度制限 (m/s²)
    EMERGENCY_BRAKE_DISTANCE = 5.0         # 緊急ブレーキ距離 (m) - より近距離で発動
    EMERGENCY_DECELERATION = -4.0          # 緊急減速度 (m/s²) - より強い減速
    TRAFFIC_LIGHT_MIN_DISTANCE = 0.5       # 信号機検出最小距離 (m) - 停止線にさらに近く
    WAYPOINT_REACHED_THRESHOLD = 5.0       # ウェイポイント到達判定距離 (m)
    ROUTE_COMPLETION_THRESHOLD = 5         # ルート完了判定残りウェイポイント数
    
    def __init__(self, vehicle, waypoints):
        """
        Initialize IDM agent.
        
        Args:
            vehicle: CARLA vehicle object
            waypoints: List of waypoint transforms for route
        """
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.prev_acceleration = 0.0
        self.current_waypoint_index = 0
        
        # Control state for display manager
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.current_steer = 0.0
        
        # No longer needed - using traffic_light_manager distances
        
        # Initialize local planner for lateral control
        self._local_planner = LocalPlanner(vehicle)
        self._local_planner.follow_speed_limits(False)  # IDMが速度制御
        self._setup_route()
        
        print(f"[IDM Agent] Initialized with {len(waypoints)} waypoints")
    
    def _setup_route(self):
        """Setup route for local planner."""
        world = self.vehicle.get_world()
        global_plan = []
        
        for waypoint_transform in self.waypoints:
            waypoint = world.get_map().get_waypoint(waypoint_transform.location)
            global_plan.append((waypoint, RoadOption.LANEFOLLOW))
        
        self._local_planner.set_global_plan(global_plan)
        print(f"[IDM Agent] Route set with {len(global_plan)} waypoints")
    

    
    def _update_waypoint_progress(self, current_location):
        """Update waypoint progress based on current location."""
        if self.current_waypoint_index >= len(self.waypoints):
            return
        
        while self.current_waypoint_index < len(self.waypoints):
            target_location = self.waypoints[self.current_waypoint_index].location
            distance = current_location.distance(target_location)
            
            if distance < self.WAYPOINT_REACHED_THRESHOLD:
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    print(f"[IDM Agent] Reached waypoint {self.current_waypoint_index - 1}")
            else:
                break
        
    def get_vehicle_state(self):
        """Get vehicle state for display manager."""
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2)
        
        return {
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z,
            'yaw': math.radians(transform.rotation.yaw),
            'speed': speed,
            'acceleration': self.prev_acceleration,
            'throttle': self.current_throttle,
            'brake': self.current_brake,
            'steer': self.current_steer
        }
    
    def _check_traffic_light_braking(self, traffic_light_manager):
        """
        Check if immediate emergency braking is needed for red/yellow traffic lights.
        Uses the same distance calculation method as traffic_light_manager.
        
        Args:
            traffic_light_manager: Traffic light manager instance
            
        Returns:
            bool: True if immediate braking is needed
        """
        current_location = self.vehicle.get_transform().location
        
        for tl_id, tl_data in traffic_light_manager.traffic_lights.items():
            tl_state = tl_data['actor'].get_state()
            
            # Check if traffic light is red or yellow and close
            if tl_state in [carla.TrafficLightState.Red, carla.TrafficLightState.Yellow]:
                # Use direct distance instead of route distance for now
                tl_location = tl_data['actor'].get_transform().location
                distance_to_light = current_location.distance(tl_location)
                
                if 0 <= distance_to_light <= self.TRAFFIC_LIGHT_MIN_DISTANCE:
                    print(f"[IDM Agent] EMERGENCY BRAKE for TL {tl_id} (state={tl_state.name}), "
                          f"direct_distance={distance_to_light:.1f}m")
                    return True
        
        return False
    
    def _get_nearest_traffic_light_info(self, traffic_light_manager):
        """
        Get info about nearest red/yellow traffic light for IDM calculation.
        
        Args:
            traffic_light_manager: Traffic light manager instance
            
        Returns:
            tuple: (distance_to_light, None) or (None, None) if no red/yellow lights
        """
        current_location = self.vehicle.get_transform().location
        nearest_distance = None
        
        print(f"[IDM Agent DEBUG] Checking traffic lights from manager:")
        
        for tl_id, tl_data in traffic_light_manager.traffic_lights.items():
            tl_state = tl_data['actor'].get_state()
            route_distance = tl_data.get('distance', float('inf'))
            
            # Use direct distance instead of route distance for now
            tl_location = tl_data['actor'].get_transform().location
            direct_distance = current_location.distance(tl_location)
            
            print(f"  TL {tl_id}: state={tl_state.name}, route_distance={route_distance:.1f}m, direct_distance={direct_distance:.1f}m")
            
            # Only consider red/yellow lights that are ahead (using direct distance)
            if tl_state in [carla.TrafficLightState.Red, carla.TrafficLightState.Yellow] and direct_distance > 0:
                if nearest_distance is None or direct_distance < nearest_distance:
                    nearest_distance = direct_distance
                    print(f"    -> NEW NEAREST: TL {tl_id} at {direct_distance:.1f}m ahead (direct)")
            else:
                print(f"    -> SKIPPED: TL {tl_id} is green or behind")
        
        if nearest_distance is not None:
            print(f"[IDM Agent] Nearest red/yellow light at direct distance: {nearest_distance:.1f}m")
            return nearest_distance, None  # Return direct distance for IDM
        else:
            print(f"[IDM Agent] Free driving - no red/yellow lights ahead")
            return None, None
    
    def _calculate_longitudinal_control(self, current_speed, traffic_light_info):
        """
        Calculate IDM-based longitudinal acceleration.
        
        Args:
            current_speed: Current vehicle speed (m/s)
            traffic_light_info: (distance_to_light, None) or (None, None)
            
        Returns:
            tuple: (acceleration, target_speed)
        """
        distance_to_light, _ = traffic_light_info
        
        if distance_to_light is not None:
            print(f"[IDM Agent] Traffic light ahead at distance: {distance_to_light:.1f}m")
            
            # Use IDM with relative distance calculation (as intended by IDM function)
            # IDM(Xh, Vh, Xp, Vp) where:
            # - Xh = 0 (ego vehicle at local origin)
            # - Vh = current_speed  
            # - Xp = distance_to_light + vehicle_length (compensate for IDM's built-in vehicle length subtraction)
            # - Vp = 0 (stopped at traffic light)
            
            # Try using distance directly - let IDM handle the vehicle length subtraction
            # If this causes issues, we may need to add back some compensation
            adjusted_distance = distance_to_light  # Use distance directly
            acceleration = IDM(0, current_speed, adjusted_distance, 0.0)
            target_speed = self.DESIRED_VELOCITY
            
            print(f"[IDM Agent] IDM calculation: distance_to_light={distance_to_light:.1f}m, "
                  f"adjusted_distance={adjusted_distance:.1f}m, current_speed={current_speed:.1f}m/s, "
                  f"acceleration={acceleration:.2f}m/s²")
        else:
            # Free driving mode - no red/yellow traffic lights ahead
            print("[IDM Agent] Free driving - no red/yellow lights ahead")
            # Use IDM with distant virtual leader to maintain desired speed
            virtual_leader_distance = 1000  # Far ahead
            virtual_leader_speed = self.DESIRED_VELOCITY  # Moving at desired speed
            
            acceleration = IDM(0, current_speed, virtual_leader_distance, virtual_leader_speed)
            target_speed = self.DESIRED_VELOCITY
        
        return acceleration, target_speed
    
    def _apply_control_mapping(self, acceleration, target_speed):
        """
        Convert IDM acceleration to CARLA control commands.
        
        Args:
            acceleration: IDM calculated acceleration
            target_speed: Target speed for local planner
            
        Returns:
            carla.VehicleControl: Control command
        """
        # Set target speed for local planner (lateral control)
        self._local_planner.set_speed(target_speed * 3.6)  # m/s to km/h
        control = self._local_planner.run_step(debug=False)
        
        # Override throttle/brake based on IDM acceleration
        if acceleration > 0.1:
            control.throttle = np.clip(acceleration / 3.0, 0.0, 0.8)
            control.brake = 0.0
        elif acceleration < -0.1:
            control.throttle = 0.0
            control.brake = np.clip(-acceleration / 4.0, 0.0, 0.6)
        # Small accelerations maintain current control
        
        return control
    
    def _check_route_completion(self, current_location):
        """Check if route is completed."""
        route_end = False
        
        # Check local planner completion
        if self._local_planner.done():
            print("[IDM Agent] Route completed by local planner")
            route_end = True
        
        # Update waypoint progress
        self._update_waypoint_progress(current_location)
        
        # Check remaining waypoints
        remaining = len(self.waypoints) - self.current_waypoint_index
        if remaining <= self.ROUTE_COMPLETION_THRESHOLD:
            print(f"[IDM Agent] Route near completion: {remaining} waypoints remaining")
            route_end = True
        
        return route_end
    
    def on_tick(self, traffic_light_manager):
        """
        Main control loop: IDM longitudinal + LocalPlanner lateral control.
        Uses same traffic light logic as MPC mode.
        
        Args:
            traffic_light_manager: Traffic light manager instance
            
        Returns:
            tuple: (acceleration, route_end, target_speed)
        """
        # Get current vehicle state
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2)
        current_location = transform.location
        
        print(f"[IDM Agent] Location: ({current_location.x:.1f}, {current_location.y:.1f}), "
              f"Speed: {current_speed:.1f} m/s")
        
        # Log distance to traffic light for debugging
        for tl_id, tl_data in traffic_light_manager.traffic_lights.items():
            distance = tl_data.get('distance', 'N/A')
            state = tl_data['actor'].get_state()
            
            # Also calculate direct distance for comparison
            tl_location = tl_data['actor'].get_transform().location
            direct_distance = current_location.distance(tl_location)
            
            print(f"[IDM Agent] TL {tl_id}: route_distance={distance:.1f}m, direct_distance={direct_distance:.1f}m, state={state.name}")
        
        # Check for immediate braking (same as MPC mode)
        if self._check_traffic_light_braking(traffic_light_manager):
            # Emergency braking like MPC mode
            acceleration = self.EMERGENCY_DECELERATION
            target_speed = 0.0
            print("[IDM Agent] Emergency braking for traffic light")
        else:
            # Get info about nearest red/yellow traffic light for IDM calculation
            traffic_light_info = self._get_nearest_traffic_light_info(traffic_light_manager)
            
            # Calculate IDM longitudinal control
            acceleration, target_speed = self._calculate_longitudinal_control(
                current_speed, traffic_light_info)
        
        # Apply acceleration limits
        acceleration = np.clip(acceleration, *self.ACCELERATION_LIMITS)
        self.prev_acceleration = acceleration
        
        print(f"[IDM Agent] Accel: {acceleration:.2f} m/s², Target: {target_speed:.1f} m/s")
        
        # Apply control mapping
        control = self._apply_control_mapping(acceleration, target_speed)
        
        # Save control state for display manager
        self.current_throttle = control.throttle
        self.current_brake = control.brake
        self.current_steer = control.steer
        
        # Apply control to vehicle
        self.vehicle.apply_control(control)
        
        # Check route completion
        route_end = self._check_route_completion(current_location)
        
        return acceleration, route_end, target_speed

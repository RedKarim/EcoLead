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
# !\ pid_agent.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
# Modified by: MAHDYAR KARIMI, 2025-08-31
#
# ---------------------------------------------------------------------

from carla_components.behavior_agent import BehaviorAgent
from carla_components.local_planner import RoadOption
import math
import numpy as np

class PIDAgent(BehaviorAgent):
    """
    PIDAgent implements an agent that uses PID control for navigation.
    """

    def __init__(self, vehicle, waypoints, behavior='normal'):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param waypoints: list of waypoints for the global plan
            :param behavior: type of agent to apply
        """
        super(PIDAgent, self).__init__(vehicle, behavior)
        # Initialize global plan with provided waypoints
        self._initialize_global_plan(waypoints)
        self.ref_v = None

    def _initialize_global_plan(self, waypoints):
        """
        Initialize the global plan for the PID agent using provided waypoints.

            :param waypoints: list of waypoints for the global plan
        """
        global_plan = [(self._map.get_waypoint(wp.location), RoadOption.LANEFOLLOW) for wp in waypoints]
        self.set_global_plan(global_plan)

    def calculate_reference_velocity(self, traffic_light_manager):
        """
        Calculate the reference velocity based on the traffic light state.

            :param traffic_light_manager: The traffic light manager instance
            :return: The reference velocity
        """
        # Get the closest traffic light
        first_key = next(iter(traffic_light_manager.traffic_lights))
        current_traffic_light_id = first_key
        self.ref_v = traffic_light_manager.calculate_reference_velocity(current_traffic_light_id)
        return self.ref_v

    def run_step(self, traffic_light_manager, debug=False):
        """
        Execute one step of navigation using PID control and follow traffic rules.

            :param traffic_light_manager: The traffic light manager instance
            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        self.calculate_reference_velocity(traffic_light_manager)
        self._update_information()
        control = None

        # 1: Red lights and stops behavior
        vehicle_location = self._vehicle.get_location()
        is_red_ahead, tl_id = traffic_light_manager.is_red_light_ahead(vehicle_location)
        if is_red_ahead:
            print(f"[PID Agent] Red light ahead (TL {tl_id}), emergency stop!")
            return self.emergency_stop()

        # 2: Car following behaviors
        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)
        if vehicle_state:
            if distance < self._behavior.braking_distance:
                return self.emergency_stop()
            else:
                control = self.car_following_manager(vehicle, distance)

        # 3: Normal behavior
        if control is None:
            # self.ref_v = 8.5  # default speed
            self._local_planner.set_speed(self.ref_v*3.6)
            control = self._local_planner.run_step(debug=True)

        return control

    def get_vehicle_state(self):
        """
        Retrieves the vehicle state information from the CARLA vehicle actor.

            :param vehicle: The CARLA vehicle actor
            :return: A dictionary containing vehicle state information
        """
        # Get vehicle location and rotation
        vehicle_location = self._vehicle.get_location()
        vehicle_rotation = self._vehicle.get_transform().rotation

        # Position in world coordinates
        x = vehicle_location.x
        y = vehicle_location.y

        # Orientation (psi) in radians (yaw in degrees, converted to radians)
        psi = np.deg2rad(vehicle_rotation.yaw)

        # Velocity vector and speed calculation
        velocity_vector = self._vehicle.get_velocity()
        speed = math.sqrt(velocity_vector.x**2 + velocity_vector.y**2)  # speed in m/s

        # Get control inputs
        control = self._vehicle.get_control()
        throttle = control.throttle
        steer = control.steer
        brake = control.brake

        # Create the vehicle state dictionary
        vehicle_state = {
            'x': x,
            'y': y,
            'psi': psi,
            'v': speed,
            'speed': speed,
            'throttle': throttle,
            'steer': steer,
            'brake': brake,
        }

        return vehicle_state

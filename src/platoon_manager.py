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
# !\ platoon_manager.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

from platoon_messages import PCM, PAM

class PlatoonManager:
    def __init__(self, vehicles, leader_id, traffic_manager, behaviour_agents,platoon_id):
        self.traffic_manager = traffic_manager  # Reference to VehicleTrafficManager
        self.vehicles = vehicles  # list of vehicles
        self.leader_id = leader_id
        self.platoon_id = platoon_id
        self.behind_vehicles = behaviour_agents  # Initialize if needed
        self.pcms = []
        self.pam = None
        self.initialize_platoon()
    
    def initialize_platoon(self):
        # Initialize PCM and PAM
        self.pcms = [PCM(vehicle_id=vehicle.id, desired_acceleration=0.0, desired_spacing=10.0,
                        platoon_id=self.platoon_id, position_in_platoon=idx, target_speed=0.0, distance_to_front=10)
                     for idx, vehicle in enumerate(self.vehicles)]
        self.pam = PAM(
            leader_id=self.leader_id,
            platoon_id=self.platoon_id,
            platoon_size=len(self.vehicles),
            platoon_speed=0.0,
            vehicle_ids=[v.id for v in self.vehicles],
            platoon_position=(0,0),
            eta_to_light=0,
            platoon_length =sum([v.distance_to_front for v in self.pcms]),
            status="stable",
            leaving_vehicles=[],
            split_decision_cntr=0,
            corridor_id=13
        )
        print(self.pcms)
        print(self.pam)

    def update_platoon(self, platoon_status,tl_id,ref_v,eta_to_light,distances=None):
        # distances is a list of each behind vehicle's gap
        platoon_length = sum(distances[1:]) if distances else 0.0 # leaving out the leader
        leaving_vehicles = [p.vehicle_id for p in platoon_status['rear_group']]
        status = platoon_status['mode']
        # print(f"[PLATOON MANAGER] Distances: {distances}")
        leader_vehicle = None
        pam = self.pam
        pcm_list = []

        # Find and store the leader vehicle
        for idx, vehicle in enumerate(self.vehicles):
            if vehicle.id == self.leader_id:
                leader_vehicle = vehicle
                speed = leader_vehicle.get_velocity()
                current_speed = (speed.x**2 + speed.y**2 + speed.z**2)**0.5
                break
        
        if leader_vehicle:
            # Create PAM for leader
            pam = PAM(
                leader_id=self.leader_id,
                platoon_id=self.platoon_id,
                platoon_size=len(self.vehicles),
                platoon_speed=ref_v,
                vehicle_ids=[v.id for v in self.vehicles],
                platoon_position=(leader_vehicle.get_location().x, leader_vehicle.get_location().y),
                eta_to_light=eta_to_light,  # Could be computed from traffic light manager
                platoon_length=platoon_length,
                status=status,
                leaving_vehicles=leaving_vehicles,
                split_decision_cntr=pam.split_decision_cntr,
                corridor_id=tl_id)
            print(f"[PLATOON MANAGER/Update Platoon] Publishing PAM: {pam}")
            self.pam = pam
        # Create PCM for each vehicle
        for position_in_platoon, vehicle in enumerate(self.vehicles):
            speed = vehicle.get_velocity()
            current_speed = (speed.x**2 + speed.y**2 + speed.z**2)**0.5
            pcm = PCM(
                vehicle_id=vehicle.id,
                desired_acceleration=0.0,  # Could be determined by your logic
                desired_spacing=10.0,      # Example spacing
                platoon_id=self.platoon_id,
                position_in_platoon=position_in_platoon,
                target_speed=current_speed,
                distance_to_front=distances[position_in_platoon] if distances else 0.0
            )
            # print(f"Publishing PCM: {pcm}")
            pcm_list.append(pcm)
        self.pcms = pcm_list
        # print(f"[PLATOON MANAGER/Update Platoon] Publishing PCMs: {pcm_list}")

    def split_platoon(self, platoon, new_group, new_platoon_id,tl_id,eta_to_light):
        print("-------------[PLATOON MANAGER]----------------")
        print("[PLATOON MANAGER] Splitting Platoon")
        sub_platoon_vehicles=[vehicle for vehicle in self.vehicles if vehicle.id in [v.vehicle_id for v in new_group]]
        sub_platoon_pcms = [pcm for pcm in self.pcms if pcm.vehicle_id in [v.vehicle_id for v in new_group]]
        sub_platoon_behind_vehicles = [vehicle for vehicle in self.behind_vehicles if vehicle['id'] in [v.vehicle_id for v in new_group]]
        # Update the current platoon to only include the front group
        self.vehicles = [vehicle for vehicle in self.vehicles if vehicle.id in [v.vehicle_id for v in platoon]]
        self.pcms = [pcm for pcm in self.pcms if pcm.vehicle_id in [v.vehicle_id for v in platoon]]
        self.behind_vehicles = [vehicle for vehicle in self.behind_vehicles if vehicle['id'] in [v.vehicle_id for v in platoon]]
        # Create a new PlatoonManager for the rear group
        if new_group:
            rear_leader_id = new_group[0].vehicle_id
            new_platoon_manager = PlatoonManager(
                vehicles=sub_platoon_vehicles,
                leader_id=rear_leader_id,
                traffic_manager=self.traffic_manager,
                behaviour_agents=sub_platoon_behind_vehicles,
                platoon_id=new_platoon_id
            )
            new_platoon_manager.ego_vehicle = sub_platoon_behind_vehicles[0]
            platoon_position=new_platoon_manager.ego_vehicle["vehicle"].get_location()
            print(f"[PLATOON MANAGER] Platoon Created with Leader Ego: {rear_leader_id}")
            # Initialize PCM and PAM for the new platoon
            new_platoon_manager.pcms = sub_platoon_pcms
            for p,pcm in enumerate(new_platoon_manager.pcms):
                pcm.platoon_id = new_platoon_manager.platoon_id
                pcm.position_in_platoon = p
                pcm.distance_to_front = 100 if rear_leader_id == pcm.vehicle_id else sub_platoon_pcms[p].distance_to_front

            new_platoon_manager.pam = PAM(
                leader_id=rear_leader_id,
                platoon_id=new_platoon_manager.platoon_id,
                platoon_size=len(new_platoon_manager.vehicles),
                platoon_speed=self.pam.platoon_speed*0.9,
                vehicle_ids=[v.id for v in new_platoon_manager.vehicles],
                platoon_position=(platoon_position.x,platoon_position.y),
                eta_to_light=self.pam.eta_to_light + eta_to_light*2,
                platoon_length=sum([0 if rear_leader_id == pcm.vehicle_id else pcm.distance_to_front for pcm in new_platoon_manager.pcms]),
                status="leaving",
                leaving_vehicles=[],
                split_decision_cntr=0,
                corridor_id=tl_id
            )
            print(f"[PLATOON MANAGER] Subplatoon PAM:{new_platoon_manager.pam}")
            print("-------------------------------------------------------")
            print(f"[PLATOON MANAGER] Subplatoon PCM:{new_platoon_manager.pcms}")
            # Add the new platoon to the list of platoons
            return new_platoon_manager
        return None
    def cleanup(self):
        """Destroy all vehicles managed by this platoon."""
        for vehicle in self.vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        self.vehicles.clear()
        print("PlatoonManager: All vehicles destroyed and cleared.")

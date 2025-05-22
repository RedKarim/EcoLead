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
# !\ route.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------
import carla
import pandas as pd

# Load the route data
file_path = "Your Path Here/route.csv"  # Replace with your actual file path
route_df = pd.read_csv(file_path)

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Print the map name (town)
town_name = world.get_map().name
print(f"Current CARLA Town: {town_name}")

# Get the CARLA debug helper
debug = world.debug

# Get all traffic lights in the CARLA world
all_traffic_lights = world.get_actors().filter('traffic.traffic_light')

# Define static traffic light IDs
target_tl_ids = {13, 11, 20}  
traffic_lights = {}

# Store Traffic Light locations
for tl in all_traffic_lights:
    if tl.id in target_tl_ids:  # Match by Actor ID
        traffic_lights[f'TL{tl.id}'] = tl.get_transform().location  # Store location

# Define corridors and their corresponding colors
corridors = [
    ('TL13', 'TL11', carla.Color(255, 0, 0)),  # Red corridor
    ('TL11', 'TL20', carla.Color(0, 255, 0)),  # Green corridor
    ('TL20', 'TL13', carla.Color(0, 0, 255))   # Blue corridor
]

# Draw corridors with predefined colors
for start_key, end_key, color in corridors:
    start_loc = traffic_lights.get(start_key)
    end_loc = traffic_lights.get(end_key)
    if start_loc and end_loc:
        debug.draw_line(
            start_loc,
            end_loc,
            thickness=0.5,
            color=color,
            life_time=60.0
        )

# Function to determine which corridor a route point belongs to
def get_closest_corridor(x, y, corridors, traffic_lights):
    min_dist = float('inf')
    best_color = carla.Color(255, 255, 255)  # Default white if no match
    point_loc = carla.Location(x, y, 0)

    for start_key, end_key, color in corridors:
        start_loc = traffic_lights.get(start_key)
        end_loc = traffic_lights.get(end_key)
        if start_loc and end_loc:
            # Compute distance from the point to the corridor (midpoint)
            mid_x = (start_loc.x + end_loc.x) / 2
            mid_y = (start_loc.y + end_loc.y) / 2
            mid_loc = carla.Location(mid_x, mid_y, 0)

            dist = point_loc.distance(mid_loc)
            if dist < min_dist:
                min_dist = dist
                best_color = color  # Assign color of the closest corridor

    return best_color

# Draw the route points with the corresponding corridor color
previous_location = None
for _, row in route_df.iterrows():
    current_location = carla.Location(row['x'], row['y'], row['z'] + 0.5)

    # Get the corresponding corridor color for this route point
    route_color = get_closest_corridor(row['x'], row['y'], corridors, traffic_lights)

    if previous_location:
        debug.draw_line(
            previous_location,
            current_location,
            thickness=0.05,
            color=route_color,  # Use the same color as the corridor
            life_time=60.0
        )
    previous_location = current_location


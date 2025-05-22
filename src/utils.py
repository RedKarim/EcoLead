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
# !\ utils.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import os
from datetime import datetime
import numpy as np
import csv
import carla

def calculate_fuel_consumption(v, u):
    """
    Calculate the fuel consumption considering physical parameters and net acceleration.

    Parameters:
    v (float): Vehicle speed (m/s)
    u (float): Commanded acceleration from MPC (m/s^2)

    Returns:
    float: Fuel consumption (L/s)
    """
    # Physical parameters
    M_h = 1500.0   # mass of the vehicle in kg (example)
    C_D = 0.3      # Drag coefficient (example)
    A_v = 2.2      # Frontal area in m^2 (example)
    mu = 0.01      # Rolling resistance coefficient (example)
    g = 9.81       # Gravity (m/s^2)
    rho_a = 1.225  # Air density (kg/m^3)
    theta = 0.0    # No slope

    # Fuel consumption polynomial coefficients
    # These coefficients are example values and should be tuned per vehicle/engine data
    b0, b1, b2, b3 = 0.1569, 0.02450, -0.0007415, 0.00005975
    c0, c1, c2 = 0.07224, 0.09681, 0.001075
    idle_fuel_rate = 0.1 / 3600  # L/s when idling (example)
    # Compute net acceleration (a_net)
    
    a_drag = (0.5 * C_D * rho_a * A_v * v**2) / M_h
    a_roll = mu * g
    a_net = u - a_drag - a_roll  # slope-free scenario
    #print('Drag and Rolling ressistance:',a_drag,a_roll)

    # Cruising fuel consumption rate
    f_cruise = b0 + b1*v + b2*(v**2) + b3*(v**3)

    # Acceleration fuel consumption rate
    f_accel = a_net * (c0 + c1*v + c2*(v**2))

    # Determine idle/braking condition ζ
    # ζ = 1 if v ≈ 0 or u < 0, else 0
    zeta = 1 if (np.isclose(v, 0, atol=1e-3) or u < 0) else 0

    fuel_consumption = (1 - zeta) * (f_cruise + f_accel) + zeta * idle_fuel_rate

    return fuel_consumption/1000, a_net

def write_data_to_csv(data_list, filename_prefix='MPC_results'):
            current_date = datetime.now().strftime('%Y-%m-%d')
            filename = f"{filename_prefix}_{current_date}.csv"
            fieldnames = ['Timestamp','Fuel Consumption', 'Current Velocity','Acceleration','Calculated Acceleration','MPC Net Acceleration','Reference Velocity','Travel Distance','Optimal a','Light State 13','Light State 11','Light State 20']
            file_exists = os.path.isfile(filename)

            with open(filename, mode='a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                if not file_exists or os.stat(filename).st_size == 0:
                    writer.writeheader()
                writer.writerows(data_list)

def read_csv_waypoints(csv_file):
    """
    Reads waypoints from a CSV file and returns a list of carla.Transform objects.
    :param csv_file: Path to the CSV file containing waypoints.
    :return: List of carla.Transform objects.
    """
    transforms = []
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            pitch = float(row['pitch'])
            yaw = float(row['yaw'])
            roll = float(row['roll'])
            location = carla.Location(x, y, z)
            rotation = carla.Rotation(pitch, yaw, roll)
            transform = carla.Transform(location, rotation)
            transforms.append(transform)
    return transforms

def read_waypoints_from_csv(file_path):
    waypoints = []
    with open(file_path, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Extract x, y, z, pitch, yaw, roll
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            pitch = float(row['pitch'])
            yaw = float(row['yaw'])
            roll = float(row['roll'])
            waypoints.append({'x': x, 'y': y, 'z': z, 'pitch': pitch, 'yaw': yaw, 'roll': roll})
    return waypoints

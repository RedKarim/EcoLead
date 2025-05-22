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
# !\ plot_csv.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import itertools

def plot_cumulative_fuel_consumption(csv_filenames):
    """
    Plots cumulative fuel consumption vs. travel distance from multiple CSV files.

    Parameters:
    - csv_filenames: list of tuples [(csv_filename, label), ...].
    """
    plt.figure(figsize=(10, 6))
    
    # Define styles for plotting
    lines = ["-", "--", "-.", ":"]
    markers = ["o", "s", "^", "D", "v", ">", "<", "p", "*", "h"]
    colors = plt.cm.get_cmap('tab10').colors  # Colormap with 10 colors
    linecycler = itertools.cycle(lines)
    markercycler = itertools.cycle(markers)
    colorcycler = itertools.cycle(colors)
    
    for csv_filename, label in csv_filenames:
        try:
            # Read the CSV file
            data = pd.read_csv(csv_filename)
    
            # Check that required columns are present
            required_columns = ['Fuel Consumption', 'Travel Distance']
            if not all(col in data.columns for col in required_columns):
                print(f"Error: CSV file '{csv_filename}' does not contain required columns.")
                continue
    
            # Extract columns
            fuel_consumption_per_timestep = data['Fuel Consumption'].values* 0.1  # liters per time step
            travel_distance = data['Travel Distance'].values  # meters
    
            # Calculate cumulative fuel consumption (liters)
            cumulative_fuel_consumption = np.cumsum(fuel_consumption_per_timestep)
    
            # Get next style elements
            linestyle = next(linecycler)
            marker = next(markercycler)
            color = next(colorcycler)
            # Simulation parameters

            dt = 0.1  # Time step in seconds



            # Calculate time array

            time = np.arange(0, len(data) * dt, dt)



            # Extract velocity and acceleration

            velocity = data['Current Velocity'].values  # Assuming 'Velocity' is in km/h

            acceleration = data['Acceleration'].values  # Assuming 'Acceleration' is in km/h^2
            # Plot velocity vs. time

            plt.figure(figsize=(10, 6))

            plt.plot(time, velocity, label='Velocity (km/h)', color='blue')

            plt.xlabel('Time (s)')

            plt.ylabel('Velocity (km/h)')

            plt.title('Velocity vs. Time')

            plt.legend()

            plt.grid(True)

            plt.show()



            # Plot acceleration vs. time

            plt.figure(figsize=(10, 6))

            plt.plot(time, acceleration, label='Acceleration (km/h²)', color='red')

            plt.xlabel('Time (s)')

            plt.ylabel('Acceleration (km/h²)')

            plt.title('Acceleration vs. Time')

            plt.legend()

            plt.grid(True)

            plt.show()
             # Plot acceleration vs. time

            plt.figure(figsize=(10, 6))

            plt.plot(time, data['Fuel Consumption'].values, label='Fuel Consumption', color='red')

            plt.xlabel('Time (s)')

            plt.ylabel('Fuel Consump (km/h²)')

            plt.title('Fuel Consup vs. Time')

            plt.legend()

            plt.grid(True)

            plt.show()


    
            # Plot cumulative fuel consumption vs. travel distance
            plt.plot(time, cumulative_fuel_consumption, label=label,
                     linestyle=linestyle, marker=marker, color=color)
    
        except FileNotFoundError:
            print(f"Error: File '{csv_filename}' not found.")
        except Exception as e:
            print(f"An error occurred while processing '{csv_filename}': {e}")
    
    plt.xlabel('Travel time (s)')
    plt.ylabel('Cumulative Fuel Consumption (Liters)')
    plt.title('Cumulative Fuel Consumption vs. Time')
    plt.legend()
    plt.grid(True)
    plt.show()
csv_files = [
    ('Autopilot_results_2024-12-02.csv', 'Auto')
    # Add more files and labels as needed
]

plot_cumulative_fuel_consumption(csv_files)
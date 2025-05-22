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
# !\ display_manager.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import pygame

class DisplayManager:
    def __init__(self, display_size=(800, 600)):
        self.display_width, self.display_height = display_size
        self.display = pygame.display.set_mode((self.display_width, self.display_height))
        pygame.display.set_caption('Ego Vehicle Camera View')
        self.font = pygame.font.SysFont('Arial', 24)

    def render(self, camera_surface, vehicle_state, traffic_light_states=None):
        # Clear the display
        self.display.fill((0, 0, 0))

        # Render the camera image
        if camera_surface is not None:
            self.display.blit(camera_surface, (0, 0))

        # Render the vehicle data as horizontal bars
        self.render_vehicle_data(vehicle_state)

        # Render traffic light states
        if traffic_light_states:
            self.render_traffic_lights(traffic_light_states)

        # Update the display
        pygame.display.flip()

    def render_vehicle_data(self, vehicle_state):
        # Define bar dimensions and positions
        bar_width = 200  # Width of the bar
        bar_height = 20  # Height of the bar
        bar_x = 10       # X position of the bar
        bar_spacing = 30 # Spacing between bars
        bar_y_start = 10 # Starting Y position for the first bar

        # Throttle Bar
        throttle_value = vehicle_state['throttle']
        throttle_bar_length = int(bar_width * throttle_value)
        throttle_bar_rect = pygame.Rect(bar_x, bar_y_start, throttle_bar_length, bar_height)
        pygame.draw.rect(self.display, (0, 255, 0), throttle_bar_rect)
        # Border for Throttle Bar
        throttle_border_rect = pygame.Rect(bar_x, bar_y_start, bar_width, bar_height)
        pygame.draw.rect(self.display, (255, 255, 255), throttle_border_rect, 2)
        # Throttle Label
        throttle_text = self.font.render('Throttle', True, (0, 0, 0))
        self.display.blit(throttle_text, (bar_x + bar_width + 10, bar_y_start))

        # Steer Bar
        steer_value = (vehicle_state['steer'] + 1) / 2  # Normalize steer from [-1,1] to [0,1]
        steer_bar_length = int(bar_width * steer_value)
        steer_bar_rect = pygame.Rect(bar_x, bar_y_start + bar_spacing, steer_bar_length, bar_height)
        pygame.draw.rect(self.display, (0, 0, 255), steer_bar_rect)
        # Border for Steer Bar
        steer_border_rect = pygame.Rect(bar_x, bar_y_start + bar_spacing, bar_width, bar_height)
        pygame.draw.rect(self.display, (255, 255, 255), steer_border_rect, 2)
        # Steer Label
        steer_text = self.font.render('Steer', True, (0, 0, 0))
        self.display.blit(steer_text, (bar_x + bar_width + 10, bar_y_start + bar_spacing))

        # Brake Bar
        brake_value = vehicle_state['brake']
        brake_bar_length = int(bar_width * brake_value)
        brake_bar_rect = pygame.Rect(bar_x, bar_y_start + 2 * bar_spacing, brake_bar_length, bar_height)
        pygame.draw.rect(self.display, (255, 0, 0), brake_bar_rect)
        # Border for Brake Bar
        brake_border_rect = pygame.Rect(bar_x, bar_y_start + 2 * bar_spacing, bar_width, bar_height)
        pygame.draw.rect(self.display, (255, 255, 255), brake_border_rect, 2)
        # Brake Label
        brake_text = self.font.render('Brake', True, (0, 0, 0))
        self.display.blit(brake_text, (bar_x + bar_width + 10, bar_y_start + 2 * bar_spacing))

        # Render the speed as text
        speed_text = self.font.render(f"Speed: {vehicle_state['speed']:.2f} m/s", True, (0, 0, 0))
        self.display.blit(speed_text, (bar_x, bar_y_start + 3 * bar_spacing))

    def render_traffic_lights(self, traffic_light_states):
        # Positioning variables
        tl_x = 10  # Starting x position
        tl_y_start = 200  # Starting y position
        tl_spacing = 50  # Vertical spacing between traffic lights

        # Sequence of traffic light IDs
        sequence = [13, 11, 20]

        for index, tl_id in enumerate(sequence):
            tl_state = traffic_light_states.get(tl_id, 'Unknown')
            # Determine color based on state
            if tl_state == 'Red':
                color = (255, 0, 0)  # Red color
            elif tl_state == 'Green':
                color = (0, 255, 0)  # Green color
            else:
                color = (128, 128, 128)  # Gray color for unknown state

            # Draw a circle representing the traffic light
            pygame.draw.circle(self.display, color, (tl_x + 20, tl_y_start + index * tl_spacing), 15)

            # Render the traffic light ID
            id_text = self.font.render(f"ID: {tl_id}", True, (0, 0, 0))
            self.display.blit(id_text, (tl_x + 50, tl_y_start - 10 + index * tl_spacing))

            # Render the state text
            #state_text = self.font.render(f"State: {tl_state}", True, (255, 255, 255))
            #self.display.blit((tl_x + 50, tl_y_start + 15 + index * tl_spacing))

    def destroy(self):
        pygame.quit()

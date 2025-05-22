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
# !\ camera_manager.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import carla
import pygame
import numpy as np

class CameraManager:
    def __init__(self, vehicle, display_size=(1200, 800), fov=90):
        self.vehicle = vehicle
        self.display_width, self.display_height = display_size
        self.fov = fov
        self.camera = None
        self.camera_surface = None

        self.setup_camera()

    def setup_camera(self):
        world = self.vehicle.get_world()
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', f'{self.display_width}')
        camera_bp.set_attribute('image_size_y', f'{self.display_height}')
        camera_bp.set_attribute('fov', f'{self.fov}')

        # Adjust the camera position relative to the vehicle
        camera_transform = carla.Transform(carla.Location(x=-20,y=0, z=60), carla.Rotation(pitch=-90))  # Adjust for BEV
        self.camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)

        # Register the callback for the camera
        self.camera.listen(self.process_image)

    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # Convert from BGR to RGB
        self.camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def get_camera_surface(self):
        return self.camera_surface

    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()

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
# !\ ego_model.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import casadi as ca
# Casadi based kinematic Model originally from https://github.com/dotchen/WorldOnRails/blob/release/rails/models/ego_model.py
class EgoModel:
    def __init__(self, dt=1. / 4,L=2.5, Cd=0.3, A=2.2, rho=1.225, Cr=0.015, m=1200):
        self.dt = dt

        # Kinematic bicycle model parameters, tuned from World on Rails
        self.front_wb = -0.090769015
        self.rear_wb = 1.4178275
        self.steer_gain = 0.36848336

    # Parameters for drag force and rolling resistance
        self.Cd = Cd          # Aerodynamic drag coefficient
        self.A = A            # Frontal area (m^2)
        self.rho = rho        # Air density (kg/m^3)
        self.Cr = Cr          # Rolling resistance coefficient
        self.m = m            # Vehicle mass (kg)

    def forward(self, locs, yaws, spds, acts):
        steer = acts[0]
        accel = acts[1] + 0.2  # Acceleration can be positive or negative

        # Calculate steering dynamics
        wheel = self.steer_gain * steer
        beta = ca.atan(self.rear_wb / (self.front_wb + self.rear_wb) * ca.tan(wheel))

        # Calculate resistive forces
        F_drag = 0.5 * self.rho * self.Cd * self.A * spds**2  # Aerodynamic drag
        F_rolling = self.m * 9.81 * self.Cr  # Rolling resistance (flat road assumed)

        # Effective acceleration after resistive forces
        accel_effective = accel - (F_drag + F_rolling) / self.m

        # Update the state
        next_locs = locs + spds * ca.vertcat(ca.cos(yaws + beta), ca.sin(yaws + beta)) * self.dt
        next_yaws = yaws + spds / self.rear_wb * ca.sin(beta) * self.dt
        next_spds = spds + accel_effective * self.dt

        # Ensure speed does not go below zero using CasADi's fmax function
        next_spds = ca.fmax(next_spds, 0)

        return next_locs, next_yaws, next_spds
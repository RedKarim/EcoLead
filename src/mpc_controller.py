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
# !\ mpc_controller.py
#
# \author  Melih Yazgan <yazgan@fzi.de>
# \date    2025-05-24
#
#
# ---------------------------------------------------------------------

import numpy as np
import casadi as ca

class MPCController:
    def __init__(self, ego_model, mpc_config, weights, fuel_coeffs):
        # Initialize parameters
        self.N = mpc_config.get('N', 10)
        self.dt = mpc_config.get('dt', 0.1)
        self.ref_v = mpc_config.get('ref_v', 11.1)  # Default ~40 km/h
        self.a_max = mpc_config.get('a_max', 3.0)
        steer_max_deg = mpc_config.get('steer_max_deg', 25)
        self.steer_max = np.deg2rad(steer_max_deg)
        self.v_max = mpc_config.get('v_max', 15.0)

        # Cost function weights
        self.weight_cte = weights.get('cte', 1.0)
        self.weight_epsi = weights.get('epsi', 1.0)
        self.weight_v = weights.get('v', 2.0)
        self.weight_delta = weights.get('delta', 2.0)
        self.weight_a = weights.get('a', 5.0)
        self.weight_diff_delta = weights.get('diff_delta', 10.0)
        self.weight_diff_a = weights.get('diff_a', 1.0)
        self.weight_fuel = weights.get('fuel', 1000.0)
        self.weight_diff_fuel = weights.get('diff_fuel', 10.0)
        self.weight_distance = weights.get('dist', 1.0)

        # Fuel consumption coefficients
        self.b0 = fuel_coeffs.get('b0', 0.0)
        self.b1 = fuel_coeffs.get('b1', 0.0)
        self.b2 = fuel_coeffs.get('b2', 0.0)
        self.b3 = fuel_coeffs.get('b3', 0.0)
        self.c0 = fuel_coeffs.get('c0', 0.0)
        self.c1 = fuel_coeffs.get('c1', 0.0)
        self.c2 = fuel_coeffs.get('c2', 0.0)
        self.F_idle = fuel_coeffs.get('F_idle', 0.0)

        # EgoModel for state prediction
        self.ego_model = ego_model
        self.Lf = 2
        self.a_net = 0 
        self.a_lumped_drag = 0

        # Physical parameters (example values, adjust as needed)
        self.M_h = 1500.0    # Mass of the vehicle (kg)
        self.C_D = 0.3       # Drag coefficient (dimensionless)
        self.A_v = 2.2       # Frontal area (m^2)
        self.mu = 0.01       # Rolling resistance coefficient (dimensionless)
        self.g = 9.81        # Gravitational acceleration (m/s^2)
        self.rho_a = 1.225   # Air density (kg/m^3)
        self.theta = 0.0     # Road slope angle (radians)
        self.idle_fuel_rate = self.F_idle / 3600  # Convert idle consumption if needed

        self.previous_solution = {
            'x': None,
            'y': None,
            'psi': None,
            'v': None,
            'cte': None,
            'epsi': None,
            'delta': None,
            'a': None,
        }

        print("MPC Controller initialized successfully!")
    
    def adaptive_velocity_loss(self,e, epsilon=0.5, w_penalty=1.0, w_reward=0.5):
        """
        Adaptive asymmetric loss function for velocity deviation with rewards.
        """
        # Penalize large positive or negative deviations
        penalty_loss = w_penalty * ca.power(e, 2)

        # Reward small errors within the reward region
        reward_loss = -w_reward * ca.power(e - epsilon, 2)

        # Define regions
        return ca.if_else(
            ca.fabs(e) <= epsilon,  # Reward region
            reward_loss,
            penalty_loss  # Penalty region
        )

    def setup_cost_function(self, opti, variables):
        x, y, psi, v, cte, epsi, delta, a, ref_v = variables
        N = self.N
        cost = 0
        # Penalize errors and deviation from reference speed
        for t in range(N):
            # safe_distance = 5 + 0.5 * v[t]
            cost += self.weight_cte * ca.power(cte[t], 2)
            cost += self.weight_epsi * ca.power(epsi[t], 2)
            velocity_error = ref_v[t] - v[t]
            cost += self.weight_v * ca.power(velocity_error, 2)

        # Penalize smooth transitions
        for t in range(N - 2):
            cost += self.weight_diff_a * ca.power(a[t + 1] - a[t], 2)
            cost += self.weight_diff_delta* ca.power(delta[t + 1] - delta[t], 2)


        opti.minimize(cost)
        print("Cost function set up successfully!")

    def setup_constraints(self, opti, variables, state, coeffs):
        x, y, psi, v, cte, epsi, delta, a, ref_v = variables
        N = self.N
        x0, y0, psi0, v0, cte0, epsi0 = state

        # Initial conditions
        opti.subject_to(x[0] == x0)
        opti.subject_to(y[0] == y0)
        opti.subject_to(psi[0] == psi0)
        opti.subject_to(v[0] == v0)
        opti.subject_to(cte[0] == cte0)
        opti.subject_to(epsi[0] == epsi0)

        for t in range(1, N):
            x_t0 = x[t - 1]
            y_t0 = y[t - 1]
            psi_t0 = psi[t - 1]
            v_t0 = v[t - 1]
            delta_t0 = delta[t - 1]  # Control inputs
            a_t0 = a[t - 1]

            # Compute drag and rolling resistance for dynamics
            a_drag = (0.5 * self.C_D * self.rho_a * self.A_v * (v_t0**2)) / self.M_h
            a_roll = self.mu * self.g
            a_net = a_t0 - a_drag - a_roll

            v_t1 = v_t0 + a_net * self.dt
            x_t1 = x_t0 + v_t0 * ca.cos(psi_t0) * self.dt
            y_t1 = y_t0 + v_t0 * ca.sin(psi_t0) * self.dt
            psi_t1 = psi_t0 + (v_t0 * delta_t0 / self.Lf) * self.dt

            # Polynomial fit for the reference path
            f_x = coeffs[0] + coeffs[1] * x_t1 + coeffs[2] * (x_t1**2) + coeffs[3] * (x_t1**3)
            psi_des = ca.arctan(coeffs[1] + 2 * coeffs[2] * x_t1 + 3 * coeffs[3] * (x_t1**2))

            opti.subject_to(x[t] == x_t1)
            opti.subject_to(y[t] == y_t1)
            opti.subject_to(psi[t] == psi_t1)
            opti.subject_to(v[t] == v_t1)
            opti.subject_to(cte[t] == (f_x - y_t1) + v_t1 * ca.sin(epsi[t-1]) * self.dt)
            opti.subject_to(epsi[t] == (psi_t1 - psi_des) + v_t1 * delta_t0 / (self.ego_model.front_wb + self.ego_model.rear_wb) * self.dt)
        print("Constraints set up successfully!")

    def solve(self, state, coeffs, prev_delta, prev_a, ref_v):
        opti = ca.Opti()

        N = self.N
        x = opti.variable(N)
        y = opti.variable(N)
        psi = opti.variable(N)
        v = opti.variable(N)
        cte = opti.variable(N)
        epsi = opti.variable(N)
        delta = opti.variable(N - 1)
        a = opti.variable(N - 1)
        # Create ref_v as a parameter array of length N
        ref_v_param = opti.parameter(N)
        if not hasattr(ref_v, '__len__'):
            ref_v_array = np.full(N, ref_v)
        else:
            ref_v_array = ref_v
        # Set the value of the parameter.
        opti.set_value(ref_v_param, ref_v_array)

        # Group variables (including ref_v_param) for convenience.
        variables = (x, y, psi, v, cte, epsi, delta, a, ref_v_param)
        self.setup_cost_function(opti, variables)
        self.setup_constraints(opti, variables, state, coeffs)

        # Bounds
        opti.subject_to(opti.bounded(-ca.inf, x, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, y, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, psi, ca.inf))
        opti.subject_to(opti.bounded(-self.v_max, v, self.v_max))
        opti.subject_to(opti.bounded(-ca.inf, cte, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, epsi, ca.inf))

        # Actuator bounds
        opti.subject_to(opti.bounded(-self.steer_max, delta, self.steer_max))
        opti.subject_to(opti.bounded(-self.a_max, a, self.a_max))

        # Use warm start if previous solution exists
        if self.previous_solution['x'] is not None:
            opti.set_initial(x, self.previous_solution['x'])
            opti.set_initial(y, self.previous_solution['y'])
            opti.set_initial(psi, self.previous_solution['psi'])
            opti.set_initial(v, self.previous_solution['v'])
            opti.set_initial(cte, self.previous_solution['cte'])
            opti.set_initial(epsi, self.previous_solution['epsi'])
            opti.set_initial(delta, self.previous_solution['delta'])
            opti.set_initial(a, self.previous_solution['a'])
            print("Warm start used for optimization.")
        else:
            print("No warm start available. Using default initialization.")
        
        opts = {
            'ipopt.print_level': 1,
            'ipopt.sb': 'yes',
            'print_time': False,
            'ipopt.max_iter': 1000,
            'ipopt.tol': 1e-6,       # Adjust tolerance for more precision
            'ipopt.acceptable_tol': 1e-5  # Set an acceptable tolerance level
        }
        opti.solver('ipopt', opts)

        try:
            sol = opti.solve()
        except RuntimeError:
            print("Solver failed!")
            return None
        # Save solution for warm start
        self.previous_solution['x'] = sol.value(x)
        self.previous_solution['y'] = sol.value(y)
        self.previous_solution['psi'] = sol.value(psi)
        self.previous_solution['v'] = sol.value(v)
        self.previous_solution['cte'] = sol.value(cte)
        self.previous_solution['epsi'] = sol.value(epsi)
        self.previous_solution['delta'] = sol.value(delta)
        self.previous_solution['a'] = sol.value(a)


        opt_delta = sol.value(delta[0])
        opt_a = sol.value(a)
        mpc_x = sol.value(x)
        mpc_y = sol.value(y)

        return opt_delta, opt_a, mpc_x, mpc_y

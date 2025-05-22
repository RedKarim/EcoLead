# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is a modified version of a CARLA API script.
#
# Copyright (c) 2025, FZI Forschungszentrum Informatik
# Copyright (c) 2018-2020, Computer Vision Center (CVC), Universitat Autonoma de Barcelona
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notices, this list of conditions,
#    and the following disclaimers.
#
# 2. Redistributions in binary form must reproduce the above copyright notices, this list of
#    conditions, and the following disclaimers in the documentation and/or other materials
#    provided with the distribution.
#
# 3. Neither the name of the copyright holders nor the names of its contributors
#    may be used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# -- END LICENSE BLOCK ------------------------------------------------

""" This module contains the different parameters sets for each behavior. """


class Cautious(object):
    """Class for Cautious agent."""
    max_speed = 40
    speed_lim_dist = 6
    speed_decrease = 12
    safety_time = 3
    min_proximity_threshold = 12
    braking_distance = 6
    tailgate_counter = 0


class Normal(object):
    """Class for Normal agent."""
    max_speed = 50
    speed_lim_dist = 3
    speed_decrease = 10
    safety_time = 3
    min_proximity_threshold = 10
    braking_distance = 5
    tailgate_counter = 0


class Aggressive(object):
    """Class for Aggressive agent."""
    max_speed = 70
    speed_lim_dist = 1
    speed_decrease = 8
    safety_time = 3
    min_proximity_threshold = 8
    braking_distance = 4
    tailgate_counter = -1

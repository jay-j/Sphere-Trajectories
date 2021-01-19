# Trajectory Monte Carlo
# Jay Jasper
import numpy as np
import matplotlib.pyplot as plt
from frc2021 import *
from trajectory_sphere import *


launch_angle_min_deg = 20
launch_angle_max_deg = 80
launch_angle_min = degrees_to_radians(launch_angle_min_deg)
launch_angle_max = degrees_to_radians(launch_angle_max_deg)

velocity_high = 30.0
velocity_low = 4.0

distance_increments = 10 # try shots from this many distances

launch_spin = -0


def simulate_trajectory_game(distance, v0, launch_angle_):
    return simulate_trajectory(ball_mass, ball_radius, [distance, launch_height], v0, np.pi-launch_angle_, launch_spin, 10)
	

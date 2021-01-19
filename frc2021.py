# constants related to frc2021 game
# Jay Jasper
import numpy as np

def radians_to_degrees(x):
    return x*180/np.pi
    
def degrees_to_radians(x):
    return x*np.pi/180

def inch_to_m(x):
    return x*.0254

launch_height = 0.6 # m, WAG

ball_mass = 0.142 # kg. from "5 ounce" on AndyMark
ball_radius = inch_to_m(0.5*7) # m

goal_height = 2.496 # m

launch_dist_min = inch_to_m(30)
launch_dist_max = inch_to_m(270)


# Trajectory Monte Carlo
# at each launch distance, what is the launch angle that gives maximum robustness?
# Jay Jasper
import numpy as np
import matplotlib.pyplot as plt
from frc2021 import *
from trajectory_sphere import *

distance_increments = 10 # try shots from this many distances

trials = 100

launch_angle_min_deg = 25
launch_angle_max_deg = 75
launch_angle_min = degrees_to_radians(launch_angle_min_deg)
launch_angle_max = degrees_to_radians(launch_angle_max_deg)
angle_increments_gen = 25
#launch_angles = np.linspace(launch_angle_min, launch_angle_max,angle_increments_gen)
#launch_angles = np.array([degrees_to_radians(55)])
launch_angles = np.array([degrees_to_radians(45), degrees_to_radians(60)]) # 35,60 
angle_increments = launch_angles.shape[0]

distances = np.linspace(launch_dist_min, launch_dist_max, distance_increments)

velocity_high = 30.0
velocity_low = 4.0

launch_spin = -0

# uncertainties!
distance_1sigma = 0.1 # m
velocity_1sigma = 0.1 # m/s
launch_angle_1sigma_deg = 2
launch_angle_1sigma = degrees_to_radians(launch_angle_1sigma_deg)

def simulate_trajectory_game(distance, v0, launch_angle_):
    return simulate_trajectory(ball_mass, ball_radius, [distance, launch_height], v0, np.pi-launch_angle_, launch_spin, 10)
	
def velocity_solve_game(distance, launch_angle_, launch_spin_):
    return velocity_solve(ball_mass, ball_radius, [distance, launch_height], launch_angle_, launch_spin_, goal_height)
    
# for each distance.. sweep a range of launch angles
# for each launch angle try some random variations in errors, record performance
# plot the optimal trajectory for each distance

trajectory_strategies = []

class SingleStrategy:
    distance = 0
    velocity = 0
    launch_angle = 0
    trajectory = None

# initialize best angle competition
launch_angle_best = np.zeros((distance_increments,))
launch_velocity_best = np.zeros((distance_increments,))
shot_std_best = 10*np.ones((distance_increments,))
trajectory_best = distance_increments*[None]

for d in range(distance_increments):
    print("NEW DISTANCE!!!")
    
    for a in range(angle_increments):
        print("   try angle", launch_angles[a])
    
        # find the optimal velocity for this shot, perfect conditions
        traj, velocity, success = velocity_solve_game(distances[d], launch_angles[a], launch_spin)
        
        # if no score on perfect then quit now, move to next angle
        if success == 0:
            continue
        
        # now do a bunch of variation to test the robustness of that solution
        trial_heights = np.zeros((trials,))
        for t in range(trials):
            # add random variation to controls
            distance_mod = np.random.normal(distances[d], distance_1sigma)
            velocity_mod = np.random.normal(velocity, velocity_1sigma)
            launch_angle_mod = np.random.normal(launch_angles[a], launch_angle_1sigma)
            
            # simulate resulting trajectory. don't care if it scores or not, will offset later if needed
            traj_t = simulate_trajectory_game(distance_mod, velocity_mod, launch_angle_mod)
            trial_heights[t]=traj_t.y[-1]
        
        # look at stdev of height range to see how this angle is compared to others in the competition
        shot_std = np.std(trial_heights)
        if shot_std < shot_std_best[d]:
            shot_std_best[d] = shot_std
            launch_angle_best[d] = launch_angles[a]
            launch_velocity_best[d] = velocity
            trajectory_best[d] = traj
            
    print("   Distance", distances[d], " best angle ", launch_angle_best[d], " velocity", launch_velocity_best[d], "  std ", shot_std_best[d])

print("launch angle best", launch_angle_best)
print("launch velocity best", launch_velocity_best)
print("shot std best", shot_std_best)
    
###################################################################
# write out data to files

fd = open('output.csv','w')
fd.write('distance (m),')
for d in distances:
    fd.write(repr(d)+",")
fd.write("\n")

fd.write('accuracy stdev (m),')
for d in range(distance_increments):
    fd.write(repr(shot_std_best[d])+",")
fd.write("\n")

fd.write('angle (deg),')
for d in range(distance_increments):
    fd.write(repr(launch_angle_best[d])+",")
fd.write("\n")

fd.write('velocity (m/s),')
for d in range(distance_increments):
    fd.write(repr(launch_velocity_best[d])+",")
fd.write("\n")
fd.close()

###################################################################
# plot all the optimal trajectories

plt.figure(1)
for traj in trajectory_best:
    if traj is not None:
        plt.plot(traj.x, traj.y)
    
plt.xlabel('x position')
plt.ylabel('y position')
plt.title('optimal launch trajectories')
plt.grid(True)
plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.axis('equal')
plt.show()
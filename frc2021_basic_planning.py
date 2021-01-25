# Jay Jasper
import numpy as np
import matplotlib.pyplot as plt
from frc2021 import *
from trajectory_sphere import *

# for a range of distances from the target
# solve for the required launch velocity

launch_angle_deg = 41 # WAG
launch_angle = degrees_to_radians(launch_angle_deg)
launch_spin = -10 # rad/s. need to use negative for backspin in this scenario

def simulate_trajectory_game(distance, v0, launch_spin_):
    return simulate_trajectory(ball_mass, ball_radius, [distance, launch_height], v0, np.pi-launch_angle, launch_spin_, 10)
    
def velocity_solve_game(distance, launch_angle_, launch_spin_):
    return velocity_solve(ball_mass, ball_radius, [distance, launch_height], launch_angle_, launch_spin_, goal_height)
    
n = 30 # how many distances to check
distances = np.linspace(launch_dist_min, launch_dist_max, n)
velocities = np.zeros(distances.shape)
solutions  = []
success = np.ones(distances.shape)

# bounds for binary search
velocity_high = 30.0
velocity_low = 4.0

for i in range(n):
    traj, v, success[i] = velocity_solve_game(distances[i], launch_angle, launch_spin)
    solutions.append(traj)
        
    print("solved dist[", i, "]=", distances[i],"   v=", v)
    velocities[i] = v

#########################################################################################################
# plots!
plt.figure(1)
plt.plot(distances[np.where(success==1)], velocities[np.where(success==1)])
plt.grid(True)
plt.title("Velocity vs. Launch Distance for successful shots")
plt.ylabel("Launch Velocity, m/s")
plt.xlabel("Launch Distance, m")

# colors as (rgb) tuples
def value_to_rgb(x):
    velocity_tmp = np.delete(velocities, np.where(success==0))
    limit_high = np.max(velocity_tmp)
    limit_low = np.min(velocity_tmp)
    y = (x - limit_low) / np.ptp(velocity_tmp) # y is 0 to 1
    return (y, 0, 1-y)

plt.figure(2)
for i in range(len(solutions)):
    traj = solutions[i]
    if success[i] == 1:
        plt.plot(traj.x, traj.y, color=value_to_rgb(velocities[i]))
    else:
        plt.plot(traj.x, traj.y, color='yellow')
        
# draw zone lines
zones = inch_to_m(np.array([90, 150, 210, 270]))
for z in zones:
    plt.plot(2*[z], [0, 2*goal_height], '--', color='black')
        
plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.grid(True)
plt.title("Trajectories, colored by launch speed")
plt.xlabel("X position, m")
plt.ylabel("Y position, m")

#########################################################################################################
# look at sensitivity of a trajectory
# grab one middle of the road one that has a solution 
# vary the parameters one by one

reference_index = int(n/2)
reference_index = 21 # 27
print(reference_index)
reference_dist = solutions[reference_index].x[0]
reference_vel = velocities[reference_index]

plt.figure(3)
variation = [-0.15, 0, 0.15]
for i in range(3):
    traj = simulate_trajectory_game(reference_dist+variation[i], reference_vel, launch_spin)
    plt.plot(traj.x, traj.y)

plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.axis('equal')
plt.grid(True)
plt.title('Distance Variation')
plt.legend(['-0.15 m', 'perfect', '+0.15 m'])
plt.xlabel("X position, m")
plt.ylabel("Y position, m")


plt.figure(4)
variation = [-5, 0, 5] # 2 
for i in range(3):
    traj = simulate_trajectory(ball_mass, ball_radius, [reference_dist, launch_height], reference_vel, np.pi-launch_angle + degrees_to_radians(variation[i]), launch_spin, 10)
    #traj = simulate_trajectory_game(reference_dist+variation[i], reference_vel, launch_spin)
    plt.plot(traj.x, traj.y)

plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.axis('equal')
plt.grid(True)
plt.title('Angle Variation')
plt.legend(['-5 deg', 'perfect', '+5 deg'])
plt.xlabel("X position, m")
plt.ylabel("Y position, m")


plt.figure(5)
variation = [-0.25, 0, 0.25]
for i in range(3):
    traj = simulate_trajectory_game(reference_dist, reference_vel+variation[i], launch_spin)
    plt.plot(traj.x, traj.y)

plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.axis('equal')
plt.grid(True)
plt.title('Speed Variation')
plt.legend(['-0.25 m/s', 'perfect', '+0.25 m/s'])
plt.xlabel("X position, m")
plt.ylabel("Y position, m")


###########################
# plot a couple different options from the same launch distance

plt.figure(6)
example_angles = degrees_to_radians(np.array([42, 50, 58]))
for angle in example_angles:
    traj, v, success = velocity_solve_game(4, angle, 0)
    plt.plot(traj.x, traj.y)
    
plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.axis('equal')
plt.grid(True)
plt.title('Solutions from the same distance')
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')


plt.show()
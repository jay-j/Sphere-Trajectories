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
    
n = 30 # how many distances to check
distances = np.linspace(launch_dist_min, launch_dist_max, n)
velocities = np.zeros(distances.shape)
solutions  = []
success = np.ones(distances.shape)

# bounds for binary search
velocity_high = 30.0
velocity_low = 4.0

for i in range(n):
    v = np.array([velocity_low, 0, velocity_high], dtype=np.double)
    attempts = 0
    print("NEW DISTANCE!", distances[i])
    
    while np.ptp(v) > 0.01:
        v[1] = 0.5*(v[0] + v[2])
        
        traj = simulate_trajectory_game(distances[i], v[1], launch_spin)
        y_cross = traj.y[-1]
        # ASSUME the goal plan zero crossing event has triggered
        
        #print("  v=", v, "  t=", traj.t[index], "x=", traj.x[index], "y=", traj.y[index])
        #print("  v=", v, "  t=", traj.t[index+1], "x=", traj.x[index+1], "y=", traj.y[index+1])
        #print("  Y estimate!", y_cross).
        
        if y_cross > goal_height:
            v[2] = v[1]
        else:
            v[0] = v[1]
            
        if v[1] > velocity_high*0.95 or v[1] < velocity_low*1.05:
            success[i] = 0
            break
            
    solutions.append(traj)
        
    print("solved dist[", i, "]=", distances[i],"   v=", v[1])
    velocities[i] = v[1]

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
plt.grid(True)
plt.title('Distance Variation')

plt.figure(4)
variation = [-5, 0, 5] # 2 
for i in range(3):
    traj = simulate_trajectory(ball_mass, ball_radius, [reference_dist, launch_height], reference_vel, np.pi-launch_angle + degrees_to_radians(variation[i]), launch_spin, 10)
    #traj = simulate_trajectory_game(reference_dist+variation[i], reference_vel, launch_spin)
    plt.plot(traj.x, traj.y)

plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.grid(True)
plt.title('Angle Variation')


plt.figure(5)
variation = [-0.25, 0, 0.25]
for i in range(3):
    traj = simulate_trajectory_game(reference_dist, reference_vel+variation[i], launch_spin)
    plt.plot(traj.x, traj.y)

plt.axis([0, np.max(distances), 0, 2*goal_height])
plt.grid(True)
plt.title('Velocity Variation')

#########################################################################################################
# monte carlo
# for a range of 

plt.show()

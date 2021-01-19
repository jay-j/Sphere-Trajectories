# Equations of Motion for a spinning sphere in projectile motion
# Author: Jay Jasper
import numpy as np
from scipy.integrate import solve_ivp
from frc2021 import *

class Trajectory:
    t = 0
    x = 0
    y = 0
    vx = 0
    vy = 0
    r = 0
    vr = 0


def ball_eom(t, u, m, r):
    # Inputs: t, u (ode solver inputs)
    #              0  1    2   3   4  5
    #         u = [x dx/dt y dy/dt r dr/dt]
    #         m: mass of the sphere (kg)
    #         r: radius of the sphere (m)
    cd = 0.47;
    rho = 1.22; # atmospheric density, kg/m^3
    g = 9.81;
	
    B1 = (8/3)*np.pi*rho*u[5]*np.power(r,3)
    B2 = 0.5*cd*rho*np.pi*np.power(r,2)
    
    udot = np.zeros((6,))
    theta = np.arctan2(u[3], u[1])
    v = np.sqrt(np.power(u[1],2) + np.power(u[3],2))
    
    udot[0] = u[1]
    udot[1] = (-B2*np.power(v,2)*np.cos(theta) - B1*v*np.sin(theta))/m;
    udot[2] = u[3]
    udot[3] = (-B2*np.power(v,2)*np.sin(theta) + B1*v*np.cos(theta) - m*g)/m;
    udot[4] = u[5]
    udot[5] = 0 # ASSUME spin rate is constant!
    
    return udot

def event_goal_plane(t, u, m, r):
    return u[0]
event_goal_plane.terminal = True

def event_peak(t, u, m, r):
    return u[3]
event_peak.terminal = False

def simulate_trajectory(mass, ball_radius, loc0, v0, theta0, phidot, time_final):
    dx0 = v0*np.cos(theta0)
    dy0 = v0*np.sin(theta0)
    u0 = np.array([loc0[0], dx0, loc0[1], dy0, 0, phidot])
    t_span = (0, time_final)
    parameters = (mass, ball_radius)
    
    soln = solve_ivp(ball_eom, t_span, u0, args=parameters, max_step=0.01, events=[event_goal_plane, event_peak])
    
    T = Trajectory()
    T.t = soln.t
    T.x =  soln.y[0,:]
    T.vx = soln.y[1,:]
    T.y =  soln.y[2,:]
    T.vy = soln.y[3,:]
    T.r =  soln.y[4,:]
    T.vr = soln.y[5,:]
    
    return T


def velocity_solve(ball_mass, ball_radius, loc0, launch_angle, launch_spin_,goal_height, velocity_high=30.0, velocity_low=4.0):
    
    v = np.array([velocity_low, 0, velocity_high], dtype=np.double)
    success = 1
    
    while np.ptp(v) > 0.01:
        v[1] = 0.5*(v[0] + v[2])
        
        #traj = simulate_trajectory_game(distances[i], v[1], launch_spin)
        traj = simulate_trajectory(ball_mass, ball_radius, loc0, v[1], np.pi-launch_angle, launch_spin_, 10)
        
        # ASSUME the goal plan zero crossing event has triggered
        if traj.y[-1] > goal_height:
            v[2] = v[1]
        else:
            v[0] = v[1]
            
        if v[1] > velocity_high*0.95 or v[1] < velocity_low*1.05:
            success = 0
            break
            
    return traj, v[1], success

# run a test/example case
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    traj = simulate_trajectory(1, 0.12, [-8, 1], 10, 0.61, 10, 2)
    print(traj)
    print(traj.t)

    plt.figure(1)
    plt.plot(traj.x, traj.y)
    plt.axis([-10, 1, 0, 3])
    plt.grid(True)

    plt.figure(2)
    plt.plot(traj.t, traj.vx)
    plt.plot(traj.t, traj.vy)
    plt.grid(True)

    plt.show()

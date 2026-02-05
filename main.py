import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Import your local modules
from config import Config
from rocket_model import RocketModel
from controller import MPCController
from visualsation import RocketViz

def get_trajectory_point(t):
    """ Returns a 12-state vector [pos, vel, ang, rate] for time t """
    # Simplified: Just climb straight up with minimal spiral
    radius = 0.02    # Tiny spiral
    omega = 0.01     # Very slow rotation
    climb_rate = 1.0 # m/s - Conservative climbing
    
    # Parametric equations (start at rocket's initial position z=0)
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    z = 0.0 + climb_rate * t 
    
    # Velocities
    vx = -radius * omega * np.sin(omega * t)
    vy =  radius * omega * np.cos(omega * t)
    vz = climb_rate
    
    ref = np.zeros(12)
    ref[0:3] = [x, y, z]
    ref[3:6] = [vx, vy, vz]
    return ref

if __name__ == "__main__":
    # 1. Initialize Objects
    rocket = RocketModel()
    mpc = MPCController()
    
    # Initialize Visualizer (Passive Mode)
    # Note: It only needs 'rocket' because main.py calculates everything else
    viz = RocketViz(rocket) 
    
    print("Starting Variable Trajectory Simulation...")
    print("Press Ctrl+C to stop.")
    
    # 2. Main Simulation Loop
    while viz.t < Config.t_max:
        
        # A. Build the "Preview Horizon" (The Snake)
        # We predict N steps into the future so MPC knows where to turn
        ref_horizon = np.zeros((12, Config.N_horizon))
        for k in range(Config.N_horizon):
            t_pred = viz.t + k * Config.dt
            ref_horizon[:, k] = get_trajectory_point(t_pred)
            
        # B. Get current Target for Visualization (The red X)
        current_target_pos = ref_horizon[0:3, 0]
        current_target_state = ref_horizon[:, 0]  # Full reference state with velocity
        
        # C. Run Control & Physics
        meas_state = rocket.state.copy()
        
        # Pass the full reference state to the MPC (includes target velocity for climbing)
        u_opt, pred_traj = mpc.compute(meas_state, current_target_state)
        
        # Step the Physics
        rocket.step(Config.dt, u_opt)
        
        # D. Update Visualization
        # Pass the calculated control, prediction, and target to the plotter
        viz.update(u_opt, pred_traj, current_target_pos, current_target_state)
        
        # E. Critical: Pause to let Matplotlib draw the frame
        # Without this, the window will appear frozen
        plt.pause(0.001)
    
    # 3. Post-Simulation Analysis - Plot Trajectory Deviation
    print("\nSimulation Complete. Generating deviation plots...")
    viz.plot_deviation_analysis()
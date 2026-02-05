import matplotlib
# 1. Force a window backend (Fixes the "FigureCanvasAgg" error)
try:
    matplotlib.use('TkAgg') 
except:
    pass 

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from config import Config

class RocketViz:
    def __init__(self, rocket):
        self.rocket = rocket
        self.dt = Config.dt
        self.t = 0.0
        
        # --- Data Storage ---
        self.time_history = []
        self.pos_history = {'x': [], 'y': [], 'z': []}
        self.u_history = {'thrust': [], 'gy': [], 'gz': []}
        
        # --- Trajectory Tracking for Deviation Analysis ---
        self.actual_pos_history = []      # Actual position [x, y, z]
        self.reference_pos_history = []   # Reference position [x, y, z]
        self.actual_vel_history = []      # Actual velocity [vx, vy, vz]
        self.reference_vel_history = []   # Reference velocity [vx, vy, vz]
        self.pos_error_history = []       # Position error magnitude

        # --- Setup Figure (3x3 Grid) ---
        plt.ion() # Enable Interactive Mode (Crucial for external loop)
        self.fig = plt.figure(figsize=(16, 12))
        
        # 1. 3D Trajectory (Top Left)
        self.ax3d = self.fig.add_subplot(2, 3, 1, projection='3d')
        self.ax3d.set_title("3D Trajectory")
        self.ax3d.set_xlim(-2, 2); self.ax3d.set_ylim(-2, 2); self.ax3d.set_zlim(0, 8)
        self.ax3d.set_xlabel('X'); self.ax3d.set_ylabel('Y'); self.ax3d.set_zlabel('Z')
        
        # 2. Altitude Tracking (Top Middle)
        self.ax_alt = self.fig.add_subplot(2, 3, 2)
        self.ax_alt.set_title("Altitude Tracking")
        self.ax_alt.set_xlabel("Time (s)"); self.ax_alt.set_ylabel("Height (m)")
        self.ax_alt.grid(True)
        
        # 3. Vertical Velocity Tracking (Top Right)
        self.ax_vz = self.fig.add_subplot(2, 3, 3)
        self.ax_vz.set_title("Vertical Velocity")
        self.ax_vz.set_xlabel("Time (s)"); self.ax_vz.set_ylabel("Vz (m/s)")
        self.ax_vz.grid(True)
        
        # 4. Position Error (Bottom Left)
        self.ax_error = self.fig.add_subplot(2, 3, 4)
        self.ax_error.set_title("Position Error Magnitude")
        self.ax_error.set_xlabel("Time (s)"); self.ax_error.set_ylabel("Error (m)")
        self.ax_error.grid(True)
        
        # 5. Thrust vs Time (Bottom Middle)
        self.ax_thrust = self.fig.add_subplot(2, 3, 5)
        self.ax_thrust.set_title("Thrust Input")
        self.ax_thrust.set_xlabel("Time (s)"); self.ax_thrust.set_ylabel("Force (N)")
        self.ax_thrust.grid(True)
        self.ax_thrust.set_ylim(-10, Config.max_thrust + 20)
        
        # 6. Gimbal Angles vs Time (Bottom Right)
        self.ax_gimbal = self.fig.add_subplot(2, 3, 6)
        self.ax_gimbal.set_title("Gimbal Angles")
        self.ax_gimbal.set_xlabel("Time (s)"); self.ax_gimbal.set_ylabel("Degrees")
        self.ax_gimbal.grid(True)
        self.ax_gimbal.set_ylim(-np.degrees(Config.max_gimbal)*1.5, np.degrees(Config.max_gimbal)*1.5)

        # --- Initialize "Artists" (The drawing objects) ---
        
        # 3D Objects
        self.line_traj, = self.ax3d.plot([], [], [], color='blue', linewidth=1.5, label='Actual Path')
        self.pt_current, = self.ax3d.plot([], [], [], marker='o', color='blue')
        self.line_pred, = self.ax3d.plot([], [], [], color='green', linestyle='--', alpha=0.5, label='MPC Prediction')
        self.line_ref_spiral, = self.ax3d.plot([], [], [], color='orange', linestyle='--', alpha=0.7, linewidth=2, label='Reference Spiral')
        
        # DYNAMIC TARGET (We save this to update it every frame)
        self.pt_target, = self.ax3d.plot([], [], [], marker='x', color='red', markersize=10, markeredgewidth=2, label='Target')
        self.ax3d.legend(loc='upper left')

        # 2D Objects - Altitude (Actual vs Reference)
        self.line_alt_actual, = self.ax_alt.plot([], [], color='red', linewidth=2, label='Actual')
        self.line_alt_ref, = self.ax_alt.plot([], [], color='blue', linestyle='--', linewidth=2, label='Reference')
        self.ax_alt.legend(loc='upper left')
        
        # 2D Objects - Vertical Velocity (Actual vs Reference)
        self.line_vz_actual, = self.ax_vz.plot([], [], color='red', linewidth=2, label='Actual')
        self.line_vz_ref, = self.ax_vz.plot([], [], color='blue', linestyle='--', linewidth=2, label='Reference')
        self.ax_vz.legend(loc='upper left')
        
        # 2D Objects - Position Error
        self.line_error, = self.ax_error.plot([], [], color='purple', linewidth=2)
        
        # 2D Objects - Thrust
        self.line_thrust, = self.ax_thrust.plot([], [], color='orange', linewidth=2)
        
        # 2D Objects - Gimbal
        self.line_gy, = self.ax_gimbal.plot([], [], label='Gimbal Y', color='purple', linewidth=2)
        self.line_gz, = self.ax_gimbal.plot([], [], label='Gimbal Z', color='cyan', linewidth=2)
        self.ax_gimbal.legend(loc='upper right')

    def update(self, u_opt, pred_traj, target_pos, ref_state=None):
        """
        Called from main.py loop.
        u_opt: [Thrust, Gy, Gz]
        pred_traj: (12, N)
        target_pos: [x, y, z] (Current target point)
        ref_state: [12] reference state for tracking deviation
        """
        self.t += self.dt
        
        # 1. Store Data
        pos = self.rocket.state[0:3]
        vel = self.rocket.state[3:6]
        self.time_history.append(self.t)
        
        self.pos_history['x'].append(pos[0])
        self.pos_history['y'].append(pos[1])
        self.pos_history['z'].append(pos[2])
        
        self.u_history['thrust'].append(u_opt[0])
        self.u_history['gy'].append(np.degrees(u_opt[1]))
        self.u_history['gz'].append(np.degrees(u_opt[2]))
        
        # 1b. Store Trajectory Tracking Data
        if ref_state is not None:
            self.actual_pos_history.append(pos.copy())
            self.reference_pos_history.append(ref_state[0:3].copy())
            self.actual_vel_history.append(vel.copy())
            self.reference_vel_history.append(ref_state[3:6].copy())
            
            # Calculate position error
            pos_error = np.linalg.norm(pos - ref_state[0:3])
            self.pos_error_history.append(pos_error)
        
        # 2. Update 3D Plot
        self.line_traj.set_data(self.pos_history['x'], self.pos_history['y'])
        self.line_traj.set_3d_properties(self.pos_history['z'])
        
        self.pt_current.set_data([pos[0]], [pos[1]])
        self.pt_current.set_3d_properties([pos[2]])
        
        # Update Variable Target
        self.pt_target.set_data([target_pos[0]], [target_pos[1]])
        self.pt_target.set_3d_properties([target_pos[2]])
        
        # Update Reference Spiral
        if self.reference_pos_history:
            ref_pos_array = np.array(self.reference_pos_history)
            self.line_ref_spiral.set_data(ref_pos_array[:, 0], ref_pos_array[:, 1])
            self.line_ref_spiral.set_3d_properties(ref_pos_array[:, 2])
        
        if pred_traj is not None:
            self.line_pred.set_data(pred_traj[0, :], pred_traj[1, :])
            self.line_pred.set_3d_properties(pred_traj[2, :])

        # 3. Update 2D Plots
        # Altitude (Actual vs Reference)
        self.line_alt_actual.set_data(self.time_history, self.pos_history['z'])
        if self.reference_pos_history:
            ref_z = [pos[2] for pos in self.reference_pos_history]
            self.line_alt_ref.set_data(self.time_history[:len(ref_z)], ref_z)
        self.ax_alt.set_xlim(0, max(5, self.t + 1))
        self.ax_alt.set_ylim(0, max(6, max(self.pos_history['z']) + 1))
        
        # Vertical Velocity (Actual vs Reference)
        actual_vz = [v[2] for v in self.actual_vel_history] if self.actual_vel_history else []
        ref_vz = [v[2] for v in self.reference_vel_history] if self.reference_vel_history else []
        if actual_vz:
            self.line_vz_actual.set_data(self.time_history[:len(actual_vz)], actual_vz)
        if ref_vz:
            self.line_vz_ref.set_data(self.time_history[:len(ref_vz)], ref_vz)
        self.ax_vz.set_xlim(0, max(5, self.t + 1))
        if actual_vz or ref_vz:
            all_vz = actual_vz + ref_vz
            self.ax_vz.set_ylim(min(all_vz) - 1, max(all_vz) + 1)
        
        # Position Error Magnitude
        if self.pos_error_history:
            self.line_error.set_data(self.time_history[:len(self.pos_error_history)], self.pos_error_history)
            self.ax_error.set_xlim(0, max(5, self.t + 1))
            self.ax_error.set_ylim(0, max(max(self.pos_error_history) + 0.1, 0.5))
        
        # Thrust
        self.line_thrust.set_data(self.time_history, self.u_history['thrust'])
        self.ax_thrust.set_xlim(0, max(5, self.t + 1))
        
        # Gimbal
        self.line_gy.set_data(self.time_history, self.u_history['gy'])
        self.line_gz.set_data(self.time_history, self.u_history['gz'])
        self.ax_gimbal.set_xlim(0, max(5, self.t + 1))

        # 4. Render
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def plot_deviation_analysis(self):
        """Plot trajectory deviation analysis after simulation completes."""
        if not self.actual_pos_history:
            print("No trajectory data to plot.")
            return
        
        # Convert lists to numpy arrays
        time_history = np.array(self.time_history)
        actual_pos_history = np.array(self.actual_pos_history)
        reference_pos_history = np.array(self.reference_pos_history)
        actual_vel_history = np.array(self.actual_vel_history)
        reference_vel_history = np.array(self.reference_vel_history)
        pos_error_history = np.array(self.pos_error_history)
        
        # Calculate position error components
        pos_error_x = actual_pos_history[:, 0] - reference_pos_history[:, 0]
        pos_error_y = actual_pos_history[:, 1] - reference_pos_history[:, 1]
        pos_error_z = actual_pos_history[:, 2] - reference_pos_history[:, 2]
        
        # Create deviation plots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Trajectory Deviation Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Position Error Magnitude
        axes[0, 0].plot(time_history, pos_error_history, 'r-', linewidth=2, label='Position Error')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Error (m)')
        axes[0, 0].set_title('Total Position Error Magnitude')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        
        # Plot 2: Position Error Components (X, Y, Z)
        axes[0, 1].plot(time_history, pos_error_x, 'b-', label='X Error', linewidth=2)
        axes[0, 1].plot(time_history, pos_error_y, 'g-', label='Y Error', linewidth=2)
        axes[0, 1].plot(time_history, pos_error_z, 'r-', label='Z Error', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].set_title('Position Error Components')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()
        
        # Plot 3: Altitude Comparison (Z position)
        axes[1, 0].plot(time_history, reference_pos_history[:, 2], 'b--', label='Reference Altitude', linewidth=2)
        axes[1, 0].plot(time_history, actual_pos_history[:, 2], 'r-', label='Actual Altitude', linewidth=2)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Altitude (m)')
        axes[1, 0].set_title('Altitude Tracking')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()
        
        # Plot 4: Vertical Velocity Comparison
        axes[1, 1].plot(time_history, reference_vel_history[:, 2], 'b--', label='Reference Vz', linewidth=2)
        axes[1, 1].plot(time_history, actual_vel_history[:, 2], 'r-', label='Actual Vz', linewidth=2)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Vertical Velocity (m/s)')
        axes[1, 1].set_title('Vertical Velocity Tracking')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
        
        plt.tight_layout()
        plt.show()
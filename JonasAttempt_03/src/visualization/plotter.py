import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, RadioButtons

from src import config
from src.utils import euler_to_dcm, body_z_axis_in_world


class Plotter:
    def __init__(self, traj_pos):
        self.traj_pos = traj_pos
        self.fig = plt.figure(figsize=(12, 6))
        self.ax_local = self.fig.add_subplot(1, 2, 1, projection="3d")
        self.ax_global = self.fig.add_subplot(1, 2, 2, projection="3d")

        self._setup_axes()

        # Global plot: static reference
        self.ax_global.plot(
            self.traj_pos[:, 0], self.traj_pos[:, 1], self.traj_pos[:, 2], "k--", lw=1
        )
        self.global_path_line, = self.ax_global.plot([], [], [], "b-", lw=2)
        self.target_dot, = self.ax_global.plot([], [], [], "ro")

        # Local plot: rocket and exhaust
        self.rocket_line, = self.ax_local.plot([], [], [], "k-", lw=3)
        self.exhaust_line, = self.ax_local.plot([], [], [], "r-", lw=2)
        
        # Landing legs (3 legs, spaced 120° apart)
        self.leg_lines = [
            self.ax_local.plot([], [], [], "grey", lw=2.5)[0] for _ in range(3)
        ]
        
        # Global plot: phase 3 follow cam
        self.global_rocket_line, = self.ax_global.plot([], [], [], "k-", lw=3)
        self.global_exhaust_line, = self.ax_global.plot([], [], [], "r-", lw=2)
        self.global_leg_lines = [
            self.ax_global.plot([], [], [], "grey", lw=2.5)[0] for _ in range(3)
        ]
        
        # Live telemetry text box
        self.telemetry_text = self.fig.text(
            0.02, 0.98, "", fontsize=9, family="monospace",
            verticalalignment="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8)
        )
        
        self._history_state = None
        self._history_control = None
        self._history_target = None
        self._history_phase = None
        self._slider_updating = False
        
        # Centralized playback state
        self.is_playing = True
        self.current_frame = 0
        self._speed = 1

    def _setup_axes(self):
        self.ax_local.set_title("Local View (CG Locked)")
        self.ax_local.set_xlim(-10, 10)
        self.ax_local.set_ylim(-10, 10)
        self.ax_local.set_zlim(-10, 10)
        self.ax_local.set_xlabel("X")
        self.ax_local.set_ylabel("Y")
        self.ax_local.set_zlabel("Z")

        self.ax_global.set_title("Global View")
        self.ax_global.set_xlabel("X")
        self.ax_global.set_ylabel("Y")
        self.ax_global.set_zlabel("Z")
        self.ax_global.set_xlim(0, config.TRAJ_X_REF * 1.05)
        self.ax_global.set_ylim(-5_000, 5_000)
        self.ax_global.set_zlim(0, config.TRAJ_Z_REF * 1.1)

    def update(self, sim_state, control, target_pos, frame_idx=None, phase="phase1", landed=False):
        pos = sim_state[0:3]
        vel = sim_state[3:6]
        phi, theta, psi = sim_state[6:9]

        # Update telemetry text with gimbal and aileron data
        throttle_pct = control["throttle"] * 100
        gimbal_y_deg = np.rad2deg(control["gimbal_y"])
        gimbal_z_deg = np.rad2deg(control["gimbal_z"])
        
        # Aileron/Grid fin data
        aero_torque = control.get("aero_torque", np.zeros(3))
        grid_fins = control.get("grid_fins", 0.0)
        aero_torque_mag = np.linalg.norm(aero_torque)
        
        # Build telemetry string
        telemetry_str = (
            f"PHASE: {phase.upper()}\n"
            f"Pos: [{pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:7.1f}] m\n"
            f"Vel: [{vel[0]:6.1f}, {vel[1]:6.1f}, {vel[2]:6.1f}] m/s\n"
            f"Gimbal: [{gimbal_y_deg:6.1f}, {gimbal_z_deg:6.1f}] deg  Thrust: {throttle_pct:5.1f}%\n"
            f"Ailerons: τ={aero_torque_mag:7.0f} N⋅m  Grid Fins: {grid_fins*100:5.1f}%"
        )
        self.telemetry_text.set_text(telemetry_str)

        # Update global path
        if self._history_state is not None and frame_idx is not None:
            hist = self._history_state[: frame_idx + 1, 0:3]
        else:
            hist = np.array([pos])
        self.global_path_line.set_data(hist[:, 0], hist[:, 1])
        self.global_path_line.set_3d_properties(hist[:, 2])

        # Phase-aware camera for global view
        if phase == "phase3":
            # Follow cam: ±50m box around rocket
            self.ax_global.set_xlim(pos[0] - 50, pos[0] + 50)
            self.ax_global.set_ylim(pos[1] - 50, pos[1] + 50)
            self.ax_global.set_zlim(max(0, pos[2] - 50), pos[2] + 50)
            self.ax_global.set_title("Landing Cam (±50m)")
            
            # Show rocket and exhaust in global view during phase 3
            dcm = euler_to_dcm(phi, theta, psi)
            body_z = dcm[:, 2]
            half_len = config.LENGTH / 2.0
            nose_global = pos + body_z * half_len
            tail_global = pos - body_z * half_len
            self.global_rocket_line.set_data([tail_global[0], nose_global[0]], [tail_global[1], nose_global[1]])
            self.global_rocket_line.set_3d_properties([tail_global[2], nose_global[2]])
            
            # Thrust vector in global view
            delta_y = control["gimbal_y"]
            delta_z = control["gimbal_z"]
            exhaust_dir_body = np.array([delta_y, delta_z, -1.0])
            exhaust_dir_body /= np.linalg.norm(exhaust_dir_body) + 1e-9
            exhaust_dir_world = dcm @ exhaust_dir_body
            exhaust_len = 4.0 + 8.0 * control["throttle"]
            engine_origin_global = pos - body_z * config.CG_FROM_GIMBAL
            exhaust_tip_global = engine_origin_global + exhaust_dir_world * exhaust_len
            self.global_exhaust_line.set_data([engine_origin_global[0], exhaust_tip_global[0]], 
                                              [engine_origin_global[1], exhaust_tip_global[1]])
            self.global_exhaust_line.set_3d_properties([engine_origin_global[2], exhaust_tip_global[2]])
            self.global_exhaust_line.set_color((1.0, 0.2 + 0.8 * control["throttle"], 0.0))
        else:
            # Global view for phases 1 & 2
            self.ax_global.set_xlim(0, config.TRAJ_X_REF * 1.05)
            self.ax_global.set_ylim(-5_000, 5_000)
            self.ax_global.set_zlim(0, config.TRAJ_Z_REF * 1.1)
            self.ax_global.set_title("Global View")
            
            # Hide rocket/exhaust in global view during phases 1 & 2
            self.global_rocket_line.set_data([], [])
            self.global_rocket_line.set_3d_properties([])
            self.global_exhaust_line.set_data([], [])
            self.global_exhaust_line.set_3d_properties([])

        if phase in ("phase1a", "phase1b", "phase1c"):
            self.target_dot.set_data([target_pos[0]], [target_pos[1]])
            self.target_dot.set_3d_properties([target_pos[2]])
        else:
            self.target_dot.set_data([], [])
            self.target_dot.set_3d_properties([])

        # Local rocket body (centered at origin)
        dcm = euler_to_dcm(phi, theta, psi)
        body_z = dcm[:, 2]
        half_len = config.LENGTH / 2.0
        nose = body_z * half_len
        tail = -body_z * half_len

        self.rocket_line.set_data([tail[0], nose[0]], [tail[1], nose[1]])
        self.rocket_line.set_3d_properties([tail[2], nose[2]])

        # Exhaust line based on gimbal and throttle (corrected origin)
        throttle = control["throttle"]
        delta_y = control["gimbal_y"]
        delta_z = control["gimbal_z"]
        exhaust_dir_body = np.array([delta_y, delta_z, -1.0])
        exhaust_dir_body /= np.linalg.norm(exhaust_dir_body) + 1e-9
        exhaust_dir_world = dcm @ exhaust_dir_body

        exhaust_len = 4.0 + 8.0 * throttle
        engine_origin = -body_z * config.CG_FROM_GIMBAL  # Engine at rocket base
        exhaust_tip = engine_origin + exhaust_dir_world * exhaust_len
        self.exhaust_line.set_data([engine_origin[0], exhaust_tip[0]], [engine_origin[1], exhaust_tip[1]])
        self.exhaust_line.set_3d_properties([engine_origin[2], exhaust_tip[2]])
        self.exhaust_line.set_color((1.0, 0.2 + 0.8 * throttle, 0.0))
        
        # Landing legs (3 legs at 120° spacing) - LOCAL VIEW
        leg_deploy = control.get("leg_deploy", 0.0)
        leg_length = 2.5
        leg_azimuths = [0, 120, 240]
        
        for i, azimuth_deg in enumerate(leg_azimuths):
            # Deployment angle: 0° (retracted up along body) → 150° (extended down)
            alpha = np.deg2rad(leg_deploy * 150)  # 0° when retracted, 150° when deployed
            psi_leg = np.deg2rad(azimuth_deg)
            
            x_tip_body = leg_length * np.sin(alpha) * np.cos(psi_leg)
            y_tip_body = leg_length * np.sin(alpha) * np.sin(psi_leg)
            z_tip_body = leg_length * np.cos(alpha) - config.CG_FROM_GIMBAL
            
            leg_tip_body_vec = np.array([x_tip_body, y_tip_body, z_tip_body])
            leg_tip_world = dcm @ leg_tip_body_vec
            leg_base = engine_origin
            
            self.leg_lines[i].set_data([leg_base[0], leg_tip_world[0]], 
                                        [leg_base[1], leg_tip_world[1]])
            self.leg_lines[i].set_3d_properties([leg_base[2], leg_tip_world[2]])
        
        # Landing legs - GLOBAL VIEW (Phase 3 only)
        if phase == "phase3":
            # Change leg color to green if landed, otherwise grey
            leg_color = "green" if landed else "grey"
            
            for i, azimuth_deg in enumerate(leg_azimuths):
                alpha = np.deg2rad(leg_deploy * 150)  # 0° when retracted, 150° when deployed
                psi_leg = np.deg2rad(azimuth_deg)
                
                x_tip_body = leg_length * np.sin(alpha) * np.cos(psi_leg)
                y_tip_body = leg_length * np.sin(alpha) * np.sin(psi_leg)
                z_tip_body = leg_length * np.cos(alpha) - config.CG_FROM_GIMBAL
                
                leg_tip_body_vec = np.array([x_tip_body, y_tip_body, z_tip_body])
                leg_tip_global = pos + (dcm @ leg_tip_body_vec)
                leg_base_global = engine_origin_global
                
                self.global_leg_lines[i].set_data([leg_base_global[0], leg_tip_global[0]], 
                                                   [leg_base_global[1], leg_tip_global[1]])
                self.global_leg_lines[i].set_3d_properties([leg_base_global[2], leg_tip_global[2]])
                self.global_leg_lines[i].set_color(leg_color)
                self.leg_lines[i].set_color(leg_color)
        else:
            for leg_line in self.global_leg_lines:
                leg_line.set_data([], [])
                leg_line.set_3d_properties([])

        return (
            self.global_path_line,
            self.target_dot,
            self.rocket_line,
            self.exhaust_line,
            *self.leg_lines,
            self.global_rocket_line,
            self.global_exhaust_line,
            *self.global_leg_lines,
            self.telemetry_text,
        )

    def animate(self, history, total_time, dt):
        self._history_state = np.array(history["state"])
        self._history_control = history["control"]
        self._history_target = history["target"]
        self._history_phase = history["phase"]
        self._history_leg_deploy = history["leg_deploy"]  # Store leg deployment history
        self._history_landed = history.get("landed", [False] * len(history["phase"]))  # Track landing status

        frames = len(self._history_state)

        # UI layout
        slider_ax = self.fig.add_axes([0.12, 0.03, 0.76, 0.03])
        button_ax = self.fig.add_axes([0.02, 0.02, 0.08, 0.06])
        speed_ax = self.fig.add_axes([0.90, 0.05, 0.08, 0.30])

        self.slider = Slider(
            ax=slider_ax,
            label="Time",
            valmin=0,
            valmax=frames - 1,
            valinit=0,
            valstep=1,
        )
        self.play_button = Button(button_ax, "Pause")
        self.speed_radio = RadioButtons(
            speed_ax,
            ("1x", "2x", "4x", "8x", "16x", "32x", "64x"),
            active=0,
        )

        def _update_frame(frame_idx):
            # Ensure frame is valid
            frame_idx = int(np.clip(frame_idx, 0, frames - 1))
            self.current_frame = frame_idx
            
            state = self._history_state[frame_idx]
            control = self._history_control[frame_idx].copy()  # Make a copy to add leg_deploy
            control["leg_deploy"] = self._history_leg_deploy[frame_idx]  # Add deployment state
            target = self._history_target[frame_idx]
            phase = self._history_phase[frame_idx]
            landed = self._history_landed[frame_idx]
            return self.update(state, control, target, frame_idx=frame_idx, phase=phase, landed=landed)

        def _on_slider(val):
            if self._slider_updating:
                return
            # Slider directly sets current frame and updates display
            self.current_frame = int(val)
            _update_frame(self.current_frame)
            self.fig.canvas.draw_idle()

        def _toggle_play(_event):
            self.is_playing = not self.is_playing
            self.play_button.label.set_text("Pause" if self.is_playing else "Play")

        def _set_speed(label):
            self._speed = int(label.replace("x", ""))

        self.slider.on_changed(_on_slider)
        self.play_button.on_clicked(_toggle_play)
        self.speed_radio.on_clicked(_set_speed)

        def _step(_frame_idx):
            # Only increment if playing
            if self.is_playing:
                self.current_frame += self._speed
                if self.current_frame >= frames:
                    self.current_frame = frames - 1
                    self.is_playing = False
                    self.play_button.label.set_text("Play")
                
                # Update slider to match current frame
                self._slider_updating = True
                self.slider.set_val(self.current_frame)
                self._slider_updating = False
            
            # Always render current frame
            return _update_frame(self.current_frame)

        return FuncAnimation(
            self.fig,
            _step,
            frames=frames,
            interval=dt * 1000,
            blit=False,
        )

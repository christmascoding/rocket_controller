"""
Dual-panel 3D visualisation with interactive playback controls.

Left  panel : **Local / body-frame view** — camera locked to the CG.
              Shows thrust vector, desired-direction arrow, velocity arrow,
              gimbal geometry, aileron torques.

Right panel : **Global / world-frame view** — inertial scene.
              Shows reference trajectory, flown trajectory, rocket attitude,
              landing target marker.  Switches to a close-up "landing cam"
              (±50 m box) once Phase 3 begins.

Bottom bar  : time slider, Play / Pause, speed selector (1×–32×).

Telemetry overlay on both panels:  phase, altitude, velocity, attitude,
angular rates, throttle, gimbal, aero torque, dynamic pressure.
"""

import json
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from src.math_utils import quat_to_dcm, quat_to_euler, euler_to_quat


# ═══════════════════════════════════════════════════════════════════════════
#  Helper — draw a 3-D arrow (quiver with arrowhead)
# ═══════════════════════════════════════════════════════════════════════════

def _arrow3d(ax, origin, vec, color='r', lw=1.5, label=None, alpha=1.0):
    """Plot a single 3-D arrow from *origin* in direction *vec*."""
    o = np.asarray(origin); v = np.asarray(vec)
    return ax.quiver(*o, *v, color=color, linewidth=lw,
                     arrow_length_ratio=0.12, alpha=alpha, label=label)


# ═══════════════════════════════════════════════════════════════════════════
#  Landing-leg geometry
# ═══════════════════════════════════════════════════════════════════════════

_LEG_LENGTH   = 2.5           # m
_LEG_AZIMUTHS = [0, 120, 240]        # 3 legs, 120° spacing
_DEPLOY_ALT   = 50.0         # m — deploy starts here
_DEPLOY_FULL  = 10.0         # m — fully open by this alt
_LEG_STOW_ANGLE = np.radians(10)    # retracted: 10° from body axis
_LEG_OPEN_ANGLE = np.radians(135)   # deployed : 135° outward from body


def _draw_legs(ax, engine_pt, body_z, body_x, body_y, R, alt, phase,
               *, local=True, pos=None):
    """Draw 4 landing legs on *ax*.

    *engine_pt* is the 3-D base point in the view's coordinate system.
    Deploy fraction ramps linearly from 0 above 50 m to 1 below 10 m.
    """
    # deploy fraction
    if phase == 'landed':
        deploy = 1.0
    elif alt < _DEPLOY_ALT:
        deploy = np.clip(1.0 - (alt - _DEPLOY_FULL) /
                         (_DEPLOY_ALT - _DEPLOY_FULL), 0.0, 1.0)
    else:
        deploy = 0.0

    sweep = _LEG_STOW_ANGLE + deploy * (_LEG_OPEN_ANGLE - _LEG_STOW_ANGLE)
    color = '#20c997' if phase == 'landed' else '#444'

    for az_deg in _LEG_AZIMUTHS:
        az = np.radians(az_deg)
        # Leg tip in body frame (from engine end, pointing upward with outward angle)
        radial_body = np.cos(az) * np.array([1, 0, 0]) + \
                      np.sin(az) * np.array([0, 1, 0])
        tip_body = (np.cos(sweep) * np.array([0, 0, 1]) +
                    np.sin(sweep) * radial_body) * _LEG_LENGTH

        if local:
            tip = engine_pt + R @ tip_body
        else:
            tip = engine_pt + R @ tip_body

        ax.plot([engine_pt[0], tip[0]],
                [engine_pt[1], tip[1]],
                [engine_pt[2], tip[2]],
                color=color, lw=1.8, solid_capstyle='round')


# ═══════════════════════════════════════════════════════════════════════════
#  Plotter
# ═══════════════════════════════════════════════════════════════════════════

class Plotter:
    """Interactive animation of a completed simulation run."""

    # ------------------------------------------------------------------ init

    def __init__(self, data: dict, ref_traj_xyz=None):
        """
        Parameters
        ----------
        data : dict as produced by ``DataLogger.to_dict()`` or loaded
               from the exported JSON.
        ref_traj_xyz : tuple (x, y, z) arrays for the reference trajectory.
        """
        self.d = data
        self.N = len(data['time'])
        self.ref = ref_traj_xyz          # may be None

        self._idx   = 0
        self._play  = False
        self._speed = 1
        self._timer = None

    # ------------------------------------------------------------------ show

    def show(self):
        """Build the figure and enter the Matplotlib event loop."""
        self.fig = plt.figure(figsize=(18, 9))
        self.fig.patch.set_facecolor('#1a1a2e')
        self.fig.subplots_adjust(left=0.03, right=0.97, bottom=0.17,
                                 top=0.95, wspace=0.10)

        # --- two 3-D axes ---
        self.ax_local  = self.fig.add_subplot(121, projection='3d',
                                              facecolor='#16213e')
        self.ax_global = self.fig.add_subplot(122, projection='3d',
                                              facecolor='#16213e')
        for ax in (self.ax_local, self.ax_global):
            ax.tick_params(colors='#aaa')
            ax.xaxis.label.set_color('#ccc')
            ax.yaxis.label.set_color('#ccc')
            ax.zaxis.label.set_color('#ccc')

        # --- slider ---
        ax_slider = self.fig.add_axes([0.12, 0.06, 0.55, 0.03],
                                       facecolor='#0f3460')
        self.slider = Slider(ax_slider, 'Time', 0, self.N - 1,
                             valinit=0, valstep=1, color='#e94560')
        self.slider.on_changed(self._on_slider)

        # --- play / pause ---
        ax_btn = self.fig.add_axes([0.72, 0.05, 0.06, 0.04])
        self.btn = Button(ax_btn, '▶ Play', color='#0f3460',
                          hovercolor='#e94560')
        self.btn.label.set_color('white')
        self.btn.on_clicked(self._toggle_play)

        # --- speed selector ---
        ax_radio = self.fig.add_axes([0.82, 0.02, 0.08, 0.10],
                                      facecolor='#0f3460')
        self.radio = RadioButtons(ax_radio,
                                  ('1×', '2×', '4×', '8×', '16×', '32×'),
                                  activecolor='#e94560')
        for lbl in self.radio.labels:
            lbl.set_color('white')
            lbl.set_fontsize(8)
        self.radio.on_clicked(self._on_speed)

        # --- initial draw ---
        self._draw_frame(0)

        plt.show()

    # ─────────────────────── frame drawing ────────────────────────────────

    def _draw_frame(self, idx):
        idx = int(np.clip(idx, 0, self.N - 1))
        self._idx = idx
        d = self.d

        t     = d['time'][idx]
        phase = d['phase'][idx]
        pos   = np.array([d['states']['x'][idx],
                          d['states']['y'][idx],
                          d['states']['z'][idx]])
        vel   = np.array([d['states']['vx'][idx],
                          d['states']['vy'][idx],
                          d['states']['vz'][idx]])
        quat  = np.array([d['states']['qw'][idx], d['states']['qx'][idx],
                          d['states']['qy'][idx], d['states']['qz'][idx]])
        euler = np.degrees([d['states']['phi'][idx],
                            d['states']['theta'][idx],
                            d['states']['psi'][idx]])
        omega = np.array([d['states']['p'][idx],
                          d['states']['q'][idx],
                          d['states']['r'][idx]])

        throttle = d['controls']['throttle'][idx]
        gy       = d['controls']['gimbal_y'][idx]
        gz       = d['controls']['gimbal_z'][idx]
        ail      = np.array([d['controls']['aileron_x'][idx],
                             d['controls']['aileron_y'][idx],
                             d['controls']['aileron_z'][idx]])

        R = quat_to_dcm(quat)
        body_z = R @ np.array([0, 0, 1])          # nose direction
        body_x = R @ np.array([1, 0, 0])
        body_y = R @ np.array([0, 1, 0])

        v_mag = np.linalg.norm(vel)

        # --- LEFT: local / body-frame view --------------------------------
        ax = self.ax_local
        ax.cla()
        ax.set_facecolor('#16213e')
        ax.set_title('Local View (CG-fixed)', color='white', fontsize=11)

        L = 8  # visual scale
        # rocket body (line from engine to nose)
        nose = body_z * 3.0
        tail = -body_z * 3.0
        ax.plot(*zip(tail, nose), color='white', lw=3, solid_capstyle='round')

        # landing legs (local view)
        _draw_legs(ax, tail, body_z, body_x, body_y,
                   R, pos[2], phase, local=True)

        # body-frame axes
        _arrow3d(ax, [0,0,0], body_x * L * 0.4, '#ff6b6b', 1, 'x_B')
        _arrow3d(ax, [0,0,0], body_y * L * 0.4, '#51cf66', 1, 'y_B')
        _arrow3d(ax, [0,0,0], body_z * L * 0.4, '#339af0', 1, 'z_B (nose)')

        # thrust plume
        if throttle > 0.01:
            plume_dir = -body_z  # exhaust opposite to nose
            # add gimbal deflection visually
            plume_dir_body = np.array([-np.sin(gy), np.sin(gz),
                                       -np.cos(gy)*np.cos(gz)])
            plume_dir = R @ plume_dir_body
            # Length: 10% (min) to 100% (max) of L*0.8
            plume_len = (0.1 + 0.9 * throttle) * L * 0.8
            # Color: white → yellow → orange → dark red
            # Define color stops
            colors = [
                (1.0, 1.0, 1.0),      # white
                (1.0, 1.0, 0.0),      # yellow
                (1.0, 0.5, 0.0),      # orange
                (0.7, 0.0, 0.0)       # dark red
            ]
            # Interpolate color
            t = np.clip(throttle, 0.0, 1.0)
            if t < 0.33:
                # white to yellow
                frac = t / 0.33
                c0, c1 = colors[0], colors[1]
            elif t < 0.66:
                # yellow to orange
                frac = (t - 0.33) / (0.33)
                c0, c1 = colors[1], colors[2]
            else:
                # orange to dark red
                frac = (t - 0.66) / (0.34)
                c0, c1 = colors[2], colors[3]
            plume_color = tuple(np.array(c0) * (1 - frac) + np.array(c1) * frac)
            _arrow3d(ax, tail, plume_dir * plume_len,
                     plume_color, 2.5, f'Thrust {throttle*100:.0f}%')

        # velocity arrow
        if v_mag > 10:
            v_dir = vel / v_mag
            _arrow3d(ax, [0,0,0], v_dir * L * 0.6, '#ffd43b', 1.5,
                     f'v={v_mag:.0f} m/s')

        ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

        # telemetry text (left panel)
        telem = (
            f"Phase: {phase}\n"
            f"t = {t:.2f} s\n"
            f"──────────────\n"
            f"Alt: {pos[2]:.1f} m\n"
            f"Vel: [{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}] m/s\n"
            f"|v| = {v_mag:.1f} m/s\n"
            f"──────────────\n"
            f"φ={euler[0]:.1f}° θ={euler[1]:.1f}° ψ={euler[2]:.1f}°\n"
            f"p={omega[0]:.3f} q={omega[1]:.3f} r={omega[2]:.3f} rad/s\n"
            f"──────────────\n"
            f"Throttle: {throttle*100:.1f}%\n"
            f"Gimbal Y: {np.degrees(gy):.2f}°  Z: {np.degrees(gz):.2f}°\n"
            f"Aileron: [{ail[0]:.0f}, {ail[1]:.0f}, {ail[2]:.0f}] N·m"
        )
        ax.text2D(0.02, 0.98, telem, transform=ax.transAxes,
                  fontsize=7, color='#ddd', family='monospace',
                  verticalalignment='top',
                  bbox=dict(facecolor='#0f3460', alpha=0.85, pad=4))

        # --- RIGHT: global / world-frame view -----------------------------
        ax2 = self.ax_global
        ax2.cla()
        ax2.set_facecolor('#16213e')

        is_landing_cam = (phase == '3')

        if is_landing_cam:
            ax2.set_title('Landing Cam (±50 m)', color='#e94560', fontsize=11)
        else:
            ax2.set_title('Global Trajectory', color='white', fontsize=11)

        # reference trajectory
        if self.ref is not None and not is_landing_cam:
            rx, ry, rz = self.ref
            ax2.plot(rx, ry, rz, '--', color='#555', lw=0.8, label='Reference')

        # flown trajectory up to now
        xs = d['states']['x'][:idx+1]
        ys = d['states']['y'][:idx+1]
        zs = d['states']['z'][:idx+1]

        # colour by phase
        phase_colors = {
            '1a': '#339af0', '1b': '#51cf66', '1c': '#ffd43b',
            '2': '#ff922b', '3': '#e94560', 'landed': '#20c997',
        }
        if not is_landing_cam:
            ax2.plot(xs, ys, zs, color=phase_colors.get(phase, 'w'),
                     lw=1.0, alpha=0.8)

        # rocket at current position
        nose_w = pos + body_z * 3.0
        tail_w = pos - body_z * 3.0
        ax2.plot(*zip(tail_w, nose_w), color='white', lw=2.5)

        # landing legs (global view)
        _draw_legs(ax2, tail_w, body_z, body_x, body_y,
                   R, pos[2], phase, local=False, pos=pos)

        # thrust plume in world
        if throttle > 0.01:
            plume_body = np.array([-np.sin(gy), np.sin(gz),
                                   -np.cos(gy)*np.cos(gz)])
            plume_w = R @ plume_body * throttle * 6
            _arrow3d(ax2, tail_w, plume_w, plt.cm.hot(0.3+0.7*throttle), 2)

        # landing target
        tgt = np.array([0, 0, 0])
        ax2.scatter(*tgt, marker='x', s=80, color='#e94560', linewidths=2)

        # ground plane hint
        if not is_landing_cam:
            gnd = 0.0
            lim_x = max(abs(pos[0]) * 1.2, 5000)
            lim_y = max(abs(pos[1]) * 1.2, 5000)
            corners = np.array([[-lim_x, -lim_y, gnd],
                                [ lim_x, -lim_y, gnd],
                                [ lim_x,  lim_y, gnd],
                                [-lim_x,  lim_y, gnd]])
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            ground = Poly3DCollection([corners], alpha=0.08, color='#20c997')
            ax2.add_collection3d(ground)

        # camera
        if is_landing_cam:
            box = 50.0
            ax2.set_xlim(pos[0]-box, pos[0]+box)
            ax2.set_ylim(pos[1]-box, pos[1]+box)
            ax2.set_zlim(max(pos[2]-box, -5), pos[2]+box)
        else:
            all_x = d['states']['x'][:idx+1]
            all_z = d['states']['z'][:idx+1]
            mx = max(max(abs(v) for v in all_x) if all_x else 1, 1000)
            mz = max(max(abs(v) for v in all_z) if all_z else 1, 1000)
            span = max(mx, mz) * 1.1
            ax2.set_xlim(-span*0.1, span)
            ax2.set_ylim(-span*0.3, span*0.3)
            ax2.set_zlim(-span*0.05, span)

        ax2.set_xlabel('X (downrange)')
        ax2.set_ylabel('Y (crossrange)')
        ax2.set_zlabel('Z (altitude)')

        # forces telemetry (right panel)
        ft_w = np.array([d['forces']['Ftx'][idx],
                         d['forces']['Fty'][idx],
                         d['forces']['Ftz'][idx]])
        fa_w = np.array([d['forces']['Fax'][idx],
                         d['forces']['Fay'][idx],
                         d['forces']['Faz'][idx]])
        from src.physics.environment import dynamic_pressure
        q_dyn = dynamic_pressure(pos[2], v_mag)

        telem2 = (
            f"Phase: {phase}   t = {t:.2f} s\n"
            f"Pos: [{pos[0]/1000:.2f}, {pos[1]/1000:.2f}, {pos[2]/1000:.2f}] km\n"
            f"Vel: {v_mag:.1f} m/s  Vz: {vel[2]:.1f} m/s\n"
            f"q_dyn: {q_dyn:.0f} Pa\n"
            f"|F_thrust|: {np.linalg.norm(ft_w)/1000:.1f} kN\n"
            f"|F_aero|: {np.linalg.norm(fa_w)/1000:.2f} kN\n"
        )
        ax2.text2D(0.02, 0.98, telem2, transform=ax2.transAxes,
                   fontsize=7, color='#ddd', family='monospace',
                   verticalalignment='top',
                   bbox=dict(facecolor='#0f3460', alpha=0.85, pad=4))

        # phase coloured marker in title area
        pcolor = phase_colors.get(phase, 'white')
        ax2.text2D(0.95, 0.98, f"●  {phase}", transform=ax2.transAxes,
                   fontsize=10, color=pcolor, ha='right', va='top',
                   fontweight='bold')

        self.fig.canvas.draw_idle()

    # ───────────────── callbacks ──────────────────────────────────────────

    def _on_slider(self, val):
        self._draw_frame(int(val))

    def _toggle_play(self, event):
        self._play = not self._play
        if self._play:
            self.btn.label.set_text('⏸ Pause')
            self._animate()
        else:
            self.btn.label.set_text('▶ Play')
            if self._timer is not None:
                self._timer.stop()

    def _on_speed(self, label):
        self._speed = int(label.replace('×', ''))

    def _animate(self):
        if self._timer is not None:
            self._timer.stop()

        def step(frame):
            if not self._play:
                return
            new_idx = self._idx + self._speed
            if new_idx >= self.N:
                self._play = False
                self.btn.label.set_text('▶ Play')
                self._timer.stop()
                return
            self.slider.set_val(new_idx)

        self._timer = self.fig.canvas.new_timer(interval=50)
        self._timer.add_callback(step, None)
        self._timer.start()


# ═══════════════════════════════════════════════════════════════════════════
#  Static multi-panel telemetry plots (shown after animation closes)
# ═══════════════════════════════════════════════════════════════════════════

def plot_telemetry(data: dict):
    """7-panel time-series plot of the simulation results."""
    t = np.array(data['time'])
    s = data['states']
    c = data['controls']
    f = data['forces']
    phases = data['phase']

    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=True)
    fig.patch.set_facecolor('#1a1a2e')
    fig.suptitle('TTHopper Telemetry', color='white', fontsize=14)

    phase_colours = {
        '1a': '#339af0', '1b': '#51cf66', '1c': '#ffd43b',
        '2': '#ff922b', '3': '#e94560', 'landed': '#20c997',
    }

    def shade_phases(ax):
        i = 0
        while i < len(phases):
            p = phases[i]
            j = i
            while j < len(phases) and phases[j] == p:
                j += 1
            ax.axvspan(t[i], t[min(j, len(t)-1)],
                       alpha=0.08, color=phase_colours.get(p, 'gray'))
            i = j

    def style(ax, ylabel):
        ax.set_facecolor('#16213e')
        ax.tick_params(colors='#aaa')
        ax.set_ylabel(ylabel, color='#ccc', fontsize=9)
        ax.grid(True, alpha=0.15)
        shade_phases(ax)

    # 1 — altitude + Vz
    ax = axes[0, 0]
    ax.plot(t, s['z'], color='#339af0', lw=0.8, label='Alt (m)')
    ax.legend(fontsize=7, loc='upper left')
    style(ax, 'Altitude [m]')
    ax2 = ax.twinx()
    ax2.plot(t, s['vz'], color='#e94560', lw=0.6, alpha=0.7, label='Vz')
    ax2.tick_params(colors='#e94560')
    ax2.set_ylabel('Vz [m/s]', color='#e94560', fontsize=8)

    # 2 — horizontal velocity
    ax = axes[0, 1]
    ax.plot(t, s['vx'], color='#339af0', lw=0.7, label='Vx')
    ax.plot(t, s['vy'], color='#51cf66', lw=0.7, label='Vy')
    ax.legend(fontsize=7); style(ax, 'Vel [m/s]')

    # 3 — throttle
    ax = axes[1, 0]
    ax.plot(t, np.array(c['throttle']) * 100, color='#ffd43b', lw=0.8)
    style(ax, 'Throttle [%]')
    ax.set_ylim(-5, 110)

    # 4 — gimbal angles
    ax = axes[1, 1]
    ax.plot(t, np.degrees(c['gimbal_y']), color='#ff6b6b', lw=0.7, label='gy')
    ax.plot(t, np.degrees(c['gimbal_z']), color='#339af0', lw=0.7, label='gz')
    ax.legend(fontsize=7); style(ax, 'Gimbal [°]')

    # 5 — euler angles
    ax = axes[2, 0]
    ax.plot(t, np.degrees(s['phi']),   color='#ff6b6b', lw=0.7, label='φ')
    ax.plot(t, np.degrees(s['theta']), color='#51cf66', lw=0.7, label='θ')
    ax.plot(t, np.degrees(s['psi']),   color='#339af0', lw=0.7, label='ψ')
    ax.legend(fontsize=7); style(ax, 'Euler [°]')

    # 6 — angular rates
    ax = axes[2, 1]
    ax.plot(t, s['p'], color='#ff6b6b', lw=0.7, label='p')
    ax.plot(t, s['q'], color='#51cf66', lw=0.7, label='q')
    ax.plot(t, s['r'], color='#339af0', lw=0.7, label='r')
    ax.legend(fontsize=7); style(ax, 'ω [rad/s]')

    # 7 — forces (thrust + aero magnitude)
    ax = axes[3, 0]
    ft = np.sqrt(np.array(f['Ftx'])**2 + np.array(f['Fty'])**2
                 + np.array(f['Ftz'])**2) / 1000
    fa = np.sqrt(np.array(f['Fax'])**2 + np.array(f['Fay'])**2
                 + np.array(f['Faz'])**2) / 1000
    ax.plot(t, ft, color='#ffd43b', lw=0.7, label='|F_thrust| kN')
    ax.plot(t, fa, color='#ff922b', lw=0.7, label='|F_aero| kN')
    ax.legend(fontsize=7); style(ax, 'Force [kN]')

    # 8 — position XY
    ax = axes[3, 1]
    ax.plot(t, np.array(s['x'])/1000, color='#339af0', lw=0.7, label='X km')
    ax.plot(t, np.array(s['y'])/1000, color='#51cf66', lw=0.7, label='Y km')
    ax.legend(fontsize=7); style(ax, 'Position [km]')

    axes[3, 0].set_xlabel('Time [s]', color='#ccc')
    axes[3, 1].set_xlabel('Time [s]', color='#ccc')

    plt.tight_layout()
    plt.show()

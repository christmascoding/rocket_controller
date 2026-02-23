import numpy as np
import matplotlib.pyplot as plt

from src import config
from src.models.rocket import Rocket
from src.models.trajectory import generate_parabolic_trajectory
from src.simulation.simulator import Simulator
from src.visualization.plotter import Plotter


def main():
    rocket = Rocket()
    traj_t, traj_pos, traj_vel = generate_parabolic_trajectory()

    sim = Simulator(rocket, traj_t, traj_pos, traj_vel)
    history = sim.run(total_time=config.SIM_T_MAX, dt=config.SIM_DT)
    plotter = Plotter(traj_pos)

    anim = plotter.animate(history, total_time=config.SIM_T_MAX, dt=config.SIM_DT)
    plt.tight_layout()
    plt.show()

    # Telemetry plots
    t = np.array(history["t"])
    state = np.array(history["state"])
    control = history["control"]

    throttle = np.array([c["throttle"] for c in control]) * 100.0
    gimbal_y = np.rad2deg(np.array([c["gimbal_y"] for c in control]))
    gimbal_z = np.rad2deg(np.array([c["gimbal_z"] for c in control]))

    aero_torque = np.array([c["aero_torque"] for c in control])
    fin_deflection = (
        np.linalg.norm(aero_torque, axis=1) / config.AERO_TORQUE_MAX * 20.0
    )

    # Extract state components
    pos_x, pos_y, altitude = state[:, 0], state[:, 1], state[:, 2]
    vx, vy, vz = state[:, 3], state[:, 4], state[:, 5]
    phi, theta, psi = np.rad2deg(state[:, 6]), np.rad2deg(state[:, 7]), np.rad2deg(state[:, 8])
    p, q, r = np.rad2deg(state[:, 9]), np.rad2deg(state[:, 10]), np.rad2deg(state[:, 11])

    fig2, axs = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    
    # Thrust
    axs[0].plot(t, throttle)
    axs[0].set_ylabel("Thrust (%)")
    axs[0].grid(True)
    axs[0].set_title("Rocket Telemetry")

    # Gimbal
    axs[1].plot(t, gimbal_y, label="Gimbal Pitch")
    axs[1].plot(t, gimbal_z, label="Gimbal Yaw")
    axs[1].set_ylabel("Gimbal (deg)")
    axs[1].legend(loc="upper right")
    axs[1].grid(True)

    # Fin deflection
    axs[2].plot(t, fin_deflection)
    axs[2].set_ylabel("Fin Defl. (deg)")
    axs[2].grid(True)

    # Velocity components
    axs[3].plot(t, vx, label="Vx", color='green')
    axs[3].plot(t, vy, label="Vy", color='orange')
    axs[3].plot(t, vz, label="Vz (Vertical)", color='red', linewidth=2)
    axs[3].set_ylabel("Velocity (m/s)")
    axs[3].legend(loc="upper right")
    axs[3].grid(True)
    axs[3].axhline(0, color='k', linestyle='--', alpha=0.3)

    # Attitude angles
    axs[4].plot(t, phi, label="φ (Roll)", color='blue')
    axs[4].plot(t, theta, label="θ (Pitch)", color='red', linewidth=2)
    axs[4].plot(t, psi, label="ψ (Yaw)", color='green')
    axs[4].set_ylabel("Attitude (deg)")
    axs[4].legend(loc="upper right")
    axs[4].grid(True)
    axs[4].axhline(180, color='r', linestyle='--', alpha=0.3, label='Target (180°)')

    # Position error from landing target
    axs[5].plot(t, pos_x, label="X position", color='blue')
    axs[5].plot(t, pos_y, label="Y position", color='green')
    axs[5].set_ylabel("Position (m)")
    axs[5].legend(loc="upper right")
    axs[5].grid(True)
    axs[5].axhline(0, color='k', linestyle='--', alpha=0.3, label='Target')

    # Altitude and Vz
    axs[6].plot(t, altitude, label="Altitude", color='blue')
    axs[6].set_ylabel("Altitude (m)", color='blue')
    axs[6].tick_params(axis='y', labelcolor='blue')
    ax6_twin = axs[6].twinx()
    ax6_twin.plot(t, vz, label="Vz", color='red')
    ax6_twin.set_ylabel("Vz (m/s)", color='red')
    ax6_twin.tick_params(axis='y', labelcolor='red')
    axs[6].set_xlabel("Time (s)")
    axs[6].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

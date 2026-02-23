import numpy as np

from src import config
from src.physics.aerodynamics import compute_aero_forces_moments
from src.utils import euler_to_dcm, omega_to_euler_rates, clamp


def state_derivative(t, state, control, rocket, phase):
    # Unpack state
    pos = state[0:3]
    vel = state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]

    dcm = euler_to_dcm(phi, theta, psi)

    # Gravity (world)
    f_grav = np.array([0.0, 0.0, -rocket.mass * config.G])

    # Thrust (body, with gimbal)
    throttle = clamp(control["throttle"], 0.0, 1.0)
    thrust = throttle * rocket.max_thrust
    gimbal_limit = config.GIMBAL_MAX_PHASE3 if phase == "phase3" else config.GIMBAL_MAX
    delta_y = clamp(control["gimbal_y"], -gimbal_limit, gimbal_limit)
    delta_z = clamp(control["gimbal_z"], -gimbal_limit, gimbal_limit)

    thrust_body = np.array([thrust * delta_y, thrust * delta_z, thrust])
    f_thrust_world = dcm @ thrust_body

    # Aerodynamics
    f_aero_world, m_aero_body = compute_aero_forces_moments(vel, dcm, rocket, phase, control)

    # Control moment from gimbal (lever arm to CG)
    r_gimbal = rocket.gimbal_to_cg
    m_gimbal_body = np.cross(r_gimbal, thrust_body)

    # Additional control moment from aero surfaces in descent
    m_ctrl_body = np.array(control.get("aero_torque", [0.0, 0.0, 0.0]))

    # Artificial roll damping in Phase 3 (simulates RCS/friction)
    if phase == "phase3":
        m_roll_damp = np.array([-config.PHASE3_ROLL_DAMPING * omega[0], 0.0, 0.0])
        m_ctrl_body += m_roll_damp

    # Sum forces and moments
    f_total = f_grav + f_thrust_world + f_aero_world
    m_total_body = m_aero_body + m_gimbal_body + m_ctrl_body

    # Translational dynamics
    acc = f_total / rocket.mass

    # Rotational dynamics
    omega_dot = rocket.inertia_inv @ (m_total_body - np.cross(omega, rocket.inertia @ omega))

    # Kinematics
    euler_rates = omega_to_euler_rates(phi, theta, omega)

    dstate = np.zeros_like(state)
    dstate[0:3] = vel
    dstate[3:6] = acc
    dstate[6:9] = euler_rates
    dstate[9:12] = omega_dot

    return dstate

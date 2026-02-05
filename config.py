import numpy as np

class Config:
    # Simulation
    dt = 0.05             # Time step (s)
    t_max = 8.0           # Total run time (s)
    N_horizon = 20       # MPC Horizon
    
    # Physical Properties
    mass = 10.0           # kg
    L = 2.0               # Length (m)
    J = np.diag([0.2, 5.0, 5.0]) # Inertia Tensor
    g = 9.81
    L_com_to_gimbal = 0.4 # Distance from CoM to engine
    
    # Actuator Constraints
    max_thrust = 400.0    # Newtons (Increased to 2.5g for better control authority)
    min_thrust = 0.0
    max_gimbal = np.radians(10) # 10 degrees
    
    # Actuator Lag
    tau_thrust = 0.10
    tau_gimbal = 0.05
    
    # MPC Settings
    N_horizon = 30        # Look-ahead steps

    # --- CONTROLLER TUNING (The Weights) ---
    # Q: State Penalty [x,y,z, vx,vy,vz, r,p,y, wx,wy,wz]
    # Tuned for aggressive altitude tracking during climb
    Q_diag = [
        50, 50, 300,      # Position: very high Z weight (altitude is critical)
        20, 20, 150,      # Velocity: high Vz weight (track climbing rate)
        0.5, 0.5, 0.1,    # Orientation: low (small angles OK)
        0.05, 0.05, 0.05  # Angular Rates: very low (don't saturate gimbal)
    ]
    
    # R: Input Penalty [Thrust, GimbalY, GimbalZ]
    # Weighted to allow full thrust authority while minimizing unnecessary gimbal
    R_diag = [
        0.0005,  # Thrust (very cheap - maximize control authority)
        5.0,     # Gimbal Y (expensive - minimize tilt)
        5.0      # Gimbal Z (expensive - minimize tilt)
    ]

    gps_noise = 0.1      # meters
    accel_noise = 0.05   # m/s^2
    gyro_noise = 0.01    # rad/s
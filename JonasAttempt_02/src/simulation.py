"""
Main Simulation Engine
Orchestrates the rocket physics, control, and visualization.
"""

import numpy as np
from src.config import SimulationParams, InitialConditions, RocketSpecs
from src.models.rocket import Rocket, RocketState
from src.controllers.controller import RocketController
from src.visualization.plotter import RocketVisualizer3D


class RocketSimulation:
    """Main simulation engine that ties everything together."""
    
    def __init__(self, trajectory, specs=None, visualize=True):
        """
        Initialize simulation.
        
        Args:
            trajectory: TrajectoryPath object defining the target path
            specs: RocketSpecs object
            visualize: Whether to visualize the simulation
        """
        self.specs = specs or RocketSpecs()
        self.trajectory = trajectory
        self.visualize = visualize
        
        # Create rocket and controller
        self.rocket = Rocket(specs=self.specs)
        self.controller = RocketController(specs=self.specs)
        
        # Set initial conditions
        self._initialize_rocket_state()
        
        # Create visualizer
        if self.visualize:
            self.visualizer = RocketVisualizer3D(specs=self.specs)
        else:
            self.visualizer = None
        
        # Simulation state
        self.current_time = 0.0
        self.dt = SimulationParams.dt
        self.max_time = SimulationParams.max_simulation_time
        
        # Telemetry data
        self.telemetry = {
            'time': [],
            'position': [],
            'velocity': [],
            'orientation': [],
            'angular_velocity': [],
            'thrust': [],
            'gimbal_pitch': [],
            'gimbal_yaw': [],
            'altitude': [],
            'velocity_magnitude': []
        }
        
        # Control loop counter for visualization updates
        self.update_counter = 0
        self.update_frequency = max(1, int(1.0 / (self.dt * VisualizationParams.plot_update_hz)))
    
    def _initialize_rocket_state(self):
        """Initialize rocket with initial conditions."""
        init = InitialConditions()
        self.rocket.state = RocketState(
            position=init.position.copy(),
            velocity=init.velocity.copy(),
            orientation=init.orientation.copy(),
            angular_velocity=init.angular_velocity.copy()
        )
    
    def step(self):
        """
        Execute one simulation step.
        
        Returns:
            True if simulation should continue, False otherwise
        """
        # Get desired state from trajectory
        desired_state = self.trajectory.get_desired_state(self.current_time)
        
        # Compute control inputs
        thrust, gimbal_pitch, gimbal_yaw = self.controller.compute_control(
            self.rocket.state, desired_state, self.dt
        )
        
        # Apply control inputs
        self.rocket.set_control_inputs(thrust, gimbal_pitch, gimbal_yaw)
        
        # Update rocket physics
        self.rocket.update(self.dt)
        
        # Record telemetry
        self._record_telemetry(thrust, gimbal_pitch, gimbal_yaw)
        
        # Update visualization if enabled
        if self.visualize and self.update_counter % self.update_frequency == 0:
            self.visualizer.update(
                self.rocket.state,
                self.current_time,
                thrust,
                gimbal_pitch,
                gimbal_yaw,
                desired_position=desired_state['position']
            )
        
        self.update_counter += 1
        
        # Check termination conditions
        should_continue = self._check_termination()
        
        # Advance time
        self.current_time += self.dt
        
        return should_continue
    
    def _record_telemetry(self, thrust, gimbal_pitch, gimbal_yaw):
        """Record simulation telemetry."""
        self.telemetry['time'].append(self.current_time)
        self.telemetry['position'].append(self.rocket.state.position.copy())
        self.telemetry['velocity'].append(self.rocket.state.velocity.copy())
        self.telemetry['orientation'].append(self.rocket.state.orientation.copy())
        self.telemetry['angular_velocity'].append(self.rocket.state.angular_velocity.copy())
        self.telemetry['thrust'].append(thrust)
        self.telemetry['gimbal_pitch'].append(gimbal_pitch)
        self.telemetry['gimbal_yaw'].append(gimbal_yaw)
        self.telemetry['altitude'].append(self.rocket.state.position[2])
        self.telemetry['velocity_magnitude'].append(np.linalg.norm(self.rocket.state.velocity))
    
    def _check_termination(self):
        """
        Check if simulation should terminate.
        
        Returns:
            True to continue, False to stop
        """
        # Stop if time exceeds maximum
        if self.current_time > self.max_time:
            print(f"Simulation ended: Maximum time ({self.max_time}s) reached")
            return False
        
        # Stop if rocket crashes (altitude becomes negative)
        if self.rocket.state.position[2] < -100:
            print(f"Simulation ended: Rocket crashed at {self.current_time:.2f}s")
            return False
        
        # Continue otherwise
        return True
    
    def run(self):
        """Run the complete simulation."""
        print("Starting rocket control simulation...")
        print(f"Target trajectory: {self.trajectory.__class__.__name__}")
        print(f"Initial altitude: {self.rocket.state.position[2]:.1f}m")
        print()
        
        step_count = 0
        try:
            while self.step():
                step_count += 1
                
                # Print progress every second of simulation
                if step_count % int(1.0 / self.dt) == 0:
                    alt = self.rocket.state.position[2]
                    vel_mag = np.linalg.norm(self.rocket.state.velocity)
                    print(f"Time: {self.current_time:7.2f}s | Altitude: {alt:10.1f}m | Velocity: {vel_mag:8.1f}m/s")
        
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        print(f"\nSimulation completed in {self.current_time:.2f} seconds ({step_count} steps)")
        print(f"Final altitude: {self.rocket.state.position[2]:.1f}m")
        print(f"Final velocity: {np.linalg.norm(self.rocket.state.velocity):.1f}m/s")
        
        # Keep visualization window open
        if self.visualize:
            self.visualizer.show()
        
        return self.telemetry
    
    def get_telemetry(self):
        """Get simulation telemetry data."""
        return self.telemetry


# Import after class definition to avoid circular imports
from src.config import VisualizationParams

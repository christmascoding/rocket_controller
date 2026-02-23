import pickle
from src.config import CONFIG
from src.simulation.simulator import RocketSimulator
from src.visualization.plotter import animate_trajectory


def main():
    print("Running simulation...")
    sim = RocketSimulator(CONFIG)
    history = sim.run()
    print("Simulation completed successfully!")
    
    # Save history for plotting
    with open('simulation_history.pkl', 'wb') as f:
        pickle.dump(history, f)
    print("Saved simulation_history.pkl")
    
    # GUI disabled for testing
    # print("Starting animation...")
    # animate_trajectory(history, trail_length=500)


if __name__ == "__main__":
    main()

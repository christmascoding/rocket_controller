from src.config import CONFIG
from src.simulation.simulator import RocketSimulator
from src.visualization.plotter import animate_trajectory


def main():
    sim = RocketSimulator(CONFIG)
    history = sim.run()
    animate_trajectory(history, trail_length=CONFIG.viz.trail_length)


if __name__ == "__main__":
    main()

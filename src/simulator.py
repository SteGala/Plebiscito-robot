from src.robot import Robot
import random
import matplotlib.pyplot as plt
import numpy as np

from src.utils import compute_adjacency_matrix

class Simulator:
    def __init__(self, n_robots, charging_threshold=0.1, operating_threshold=0.9, probability=1) -> None:
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        self.robots = []
        for i in range(n_robots):
            self.robots.append(Robot("robot" + str(i), battery_level=random.randint(20, 80), total_battery=100, charge_rate=random.randint(2, 5), disharge_rate=random.randint(1, 2)))
            
        # Compute probability-defined adjacency matrix 
        self.adjacency_matrix = compute_adjacency_matrix(n_robots, probability)   
            
    def run(self, epochs):
        res = {}
        for r in self.robots:
            res[r.name] = []
            
        for i in range(epochs):
            available_robots_ids = []
            
            for id, robot in enumerate(self.robots):
                battery = robot.tick()
                res[robot.name].append(battery)
                r_status = robot.get_status()
                if battery < self.charging_threshold and r_status != "charging":
                    robot.charge()
                    available_robots_ids.append(id)
                elif battery > self.operating_threshold and r_status != "operating":
                    robot.operate()
                    # remove offloaded task
                else:
                    if not robot.is_hosting() and r_status != "charging":
                        available_robots_ids.append(id)
            
            # use available robots to host tasks
        
        self.plot_results(res)
        
    def plot_results(self, data):
        for robot, values in data.items():
            plt.plot(values, label=robot)

        # Add labels and title
        plt.xlabel('Time')
        plt.ylabel('Battery Level')
        plt.title('Battery Levels of Robots Over Time')
        plt.legend()

        # Show plot
        plt.savefig("battery_levels.png")
        
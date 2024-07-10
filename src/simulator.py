from src.robot import Robot
import random
import numpy as np
import pandas as pd
from src.utils import compute_adjacency_matrix, dijkstra
import os

import matplotlib.pyplot as plt


class Simulator:
    def __init__(self, sim_name, n_robots, charging_threshold=0, operating_threshold=1, probability=1, move_computation_enabled=True) -> None:
        """
        Initialize the Simulator class.

        Args:
            sim_name (str): Name of the simulation.
            n_robots (int): Number of robots in the simulation.
            charging_threshold (int, optional): Battery level threshold for starting charging. Defaults to 0.
            operating_threshold (int, optional): Battery level threshold for starting operating. Defaults to 1.
            probability (float, optional): Probability of adjacency between robots. Defaults to 1.
            move_computation_enabled (bool, optional): Flag to enable move computation. Defaults to True.
        """
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        self.robots = []
        self.move_computation_enabled = move_computation_enabled
        self.sim_name = sim_name
        
        self.initialize_stats()
        
        random.seed(n_robots)
        
        # Create n_robots instances of the Robot class with random battery levels, charge rates, and discharge rates
        for i in range(n_robots):
            self.robots.append(Robot("robot" + str(i), battery_level=random.randint(20, 80), total_battery=100, charge_rate=random.randint(2, 5), disharge_rate=random.randint(1, 2)))
            
        # Compute probability-defined adjacency matrix 
        self.adjacency_matrix = compute_adjacency_matrix(n_robots, probability)   
        
    def initialize_stats(self):
        """
        Initialize the statistics dictionary.
        """
        self.stats = {}
        self.stats["wasted_charging"] = 0
        self.stats["wasted_operating"] = 0
            
    def run(self, epochs):
        """
        Run the simulation for the specified number of epochs.

        Args:
            epochs (int): Number of epochs to run the simulation.
        """
        res = {}
        
        # Initialize a dictionary to store battery levels for each robot
        for r in self.robots:
            res[r.name] = []
            
        for ep in range(epochs):
            available_robots_ids = []
            
            # Iterate over each robot
            for id, robot in enumerate(self.robots):
                battery = robot.tick()
                res[robot.name].append(battery)
                r_status = robot.get_status()
                
                # If battery level is below charging threshold and the robot is not already charging, start charging
                if battery < self.charging_threshold and r_status != "charging":
                    robot.charge()
                    available_robots_ids.append(id)
                
                # If battery level is above operating threshold and the robot is not already operating, set it to operate
                elif battery >= self.operating_threshold and r_status != "operating":
                    # Check if the robot is hosting a task
                    hosted_task = robot.get_hosted_task()
                    if hosted_task is not None:
                        # Unoffload the task
                        hosted_task.get_from().unoffload()
                    
                    # Set the robot to operate
                    robot.operate()
                else:
                    # If the robot is not hosting a task and it is currently charging, add it to the available robots list
                    if not robot.is_hosting() and r_status == "charging":
                        available_robots_ids.append(id)
            
            # Use available robots to host tasks
            if self.move_computation_enabled:
                self.move_computation(available_robots_ids)
                
            self.update_stats()

        self.dump_report() 
        self.plot_results(res)

    def move_computation(self, available_robots_ids):
        """
        Perform move computation for available robots.

        Args:
            available_robots_ids (list): List of available robot IDs.
        """
        for i in available_robots_ids:
            robot = self.robots[i]
            
            # Skip if the robot is charging or already hosting a task
            if robot.is_hosting():
                continue
                
            # Find the nearest robot that is operating
            distances = dijkstra(self.adjacency_matrix, i)
                
            found = False
            for _, ids in distances.items():
                if found:
                    break
                    
                for id in ids:
                    if not self.robots[id].has_offloaded() and self.robots[id].get_status() == "operating":
                        self.robots[id].offload()
                        assert robot.host(self.robots[id].get_self_task()) != False
                        found = True
                        break

    def update_stats(self):
        """
        Update the simulation statistics.
        """
        for robot in self.robots:
            if robot.get_status() == "charging" and not robot.is_hosting():
                self.stats["wasted_charging"] += 1
            elif robot.get_status() == "operating" and not robot.has_offloaded():
                self.stats["wasted_operating"] += 1

    def dump_report(self):
        """
        Dump the simulation report to CSV files.
        """
        offloaded_computing = {}
        operation_time = {}
        charging_time = {}
        n_charging = {} 
        n_operating = {} 
        n_offloaded = {} 
        n_hosted = {} 
        
        for _, robot in enumerate(self.robots):
            stat = robot.get_stats()
            offloaded_computing[robot.name] = stat["offloaded_computing"]
            operation_time[robot.name] = stat["operation_time"]
            charging_time[robot.name] = stat["charging_time"]
            n_charging[robot.name] = stat["n_charging"]
            n_operating[robot.name] = stat["n_operating"]
            n_offloaded[robot.name] = stat["n_offloaded"]
            n_hosted[robot.name] = stat["n_hosted"]
            
        
        # Create the directory if it doesn't exist
        if not os.path.exists(self.sim_name):
            os.makedirs(self.sim_name)

        # Save the dataframes as CSV files in the directory
        pd.DataFrame([offloaded_computing]).to_csv(f"{self.sim_name}/offloaded_computing.csv", index=False)
        pd.DataFrame([operation_time]).to_csv(f"{self.sim_name}/operation_time.csv", index=False)
        pd.DataFrame([charging_time]).to_csv(f"{self.sim_name}/charging_time.csv", index=False)
        pd.DataFrame([n_charging]).to_csv(f"{self.sim_name}/n_charging.csv", index=False)
        pd.DataFrame([n_operating]).to_csv(f"{self.sim_name}/n_operating.csv", index=False)
        pd.DataFrame([n_offloaded]).to_csv(f"{self.sim_name}/n_offloaded.csv", index=False)
        pd.DataFrame([n_hosted]).to_csv(f"{self.sim_name}/n_hosted.csv", index=False)
        
        pd.DataFrame([self.stats]).to_csv(f"{self.sim_name}/simulation_stats.csv", index=False)
             
    def plot_results(self, data):
        """
        Plot the battery levels of each robot over time.

        Args:
            data (dict): Dictionary containing battery levels for each robot.
        """
        num_robots = len(data)
        fig, axs = plt.subplots(num_robots, 1, figsize=(8, 6*num_robots))
        
        for i, (robot, values) in enumerate(data.items()):
            axs[i].plot(values, label=robot)
            axs[i].set_xlabel('Time')
            axs[i].set_ylabel('Battery Level')
            axs[i].set_title(f'Battery Levels of {robot} Over Time')
            axs[i].legend()
        
        # Save the plot as an image file
        plt.savefig("battery_levels.png")
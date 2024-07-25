from src.robot import Robot
from src.mpc import BruteForceAllocation
import random
import numpy as np
import pandas as pd
from src.utils import compute_adjacency_matrix, dijkstra
import os
from tqdm import tqdm

import matplotlib.pyplot as plt

class Simulator:
    def __init__(self, sim_name, n_robots, charging_threshold=0.05, operating_threshold=0.95, probability=1, move_computation_enabled=True, optimize_computation_frequency=None, target_battery_fleet=0.5, prediction_horizon=5, weight_battery=1, weight_offloading=0.1) -> None:
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        self.robots = []
        self.move_computation_enabled = move_computation_enabled
        self.optimize_computation_frequency = optimize_computation_frequency
        self.sim_name = "res/" + sim_name
        self.target_battery_fleet = target_battery_fleet
        self.prediction_horizon = prediction_horizon
        self.weight_battery = weight_battery
        self.weight_offloading = weight_offloading
        
        if optimize_computation_frequency is not None:
            self.allocator = BruteForceAllocation(n_robots)
        
        self.initialize_stats()
        
        random.seed(n_robots)
        
        # Create n_robots instances of the Robot class with random battery levels, charge rates, and discharge rates
        for i in range(n_robots):
            self.robots.append(Robot(i, battery_level=random.randint(200, 800), total_battery=1000, charge_rate=random.randint(4, 8), disharge_rate=random.randint(1, 2))) #disharge_rate=random.randint(1, 2)
            
        if probability != 1:
            print("WARNING: The code has not being tested with probability != 1. Unexpected results may arise.")
        
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
            
        for ep in tqdm(range(epochs), desc = 'Simulating epoch: '):
            available_robots_ids = []
            
            # Iterate over each robot
            for id, robot in enumerate(self.robots):
                battery = robot.tick()
                res[robot.name].append(battery)
                r_status = robot.get_status()
                
                # If battery level is below charging threshold and the robot is not already charging, start charging
                if battery <= self.charging_threshold and r_status != "charging":
                    #print("ch", id)
                    robot.charge()
                    available_robots_ids.append(id)
                
                # If battery level is above operating threshold and the robot is not already operating, set it to operate
                elif battery >= self.operating_threshold and r_status != "operating":
                    # Set the robot to operate
                    #print("op", id)
                    robot.operate()
                else:
                    # If the robot is not hosting a task and it is currently charging, add it to the available robots list
                    if not robot.is_hosting() and r_status == "charging":
                        available_robots_ids.append(id)
                                
            # Use available robots to host tasks
            if self.move_computation_enabled:
                self.move_computation(available_robots_ids)
                
            if self.optimize_computation_frequency is not None and ep%self.optimize_computation_frequency == 0:
                self.optimize_computation()
                
            for r in self.robots:
                r.update_computation()
            
            assert self.check_infrastructure(), self.print_infrastructure(ep)
            self.update_stats(ep)

        self.dump_report() 
        self.plot_results(res)
        
    def check_infrastructure(self): 
        count = 0
        for r in self.robots:   
            count += r.stats["computation"]
        return count%len(self.robots) == 0
    
    def print_infrastructure(self, ep):
        print("Epoch: ", ep)
        for r in self.robots:
            print(r, "\t", r.get_self_task(), "\t", r.get_hosted_task())
        print()
    
    def optimize_computation(self):
        task_requirements = [r.get_self_task().get_consumption() for r in self.robots]
        battery_levels = [r.get_battery_level() for r in self.robots]
        battery_status = [r.get_battery_status() for r in self.robots]
        discharge_rate = [r.get_discharge_rate() for r in self.robots]
        charge_rate = [r.get_charge_rate() for r in self.robots]
        constrained_allocation = [-1 for _ in range(len(self.robots))]
        
        for id, r in enumerate(self.robots):
            if r.get_status() == "charging" and r.get_hosted_task() is not None:
                constrained_allocation[r.get_hosted_task().get_from().get_name()] = id
        
        offloading_decision_brute = self.allocator.find_best_allocation(task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, self.optimize_computation_frequency, costrained_allocation=constrained_allocation)
        
        for r in self.robots:
            r.unhost()
            r.unoffload()
            
        for i, id in enumerate(offloading_decision_brute):     
            if self.robots[id] != self.robots[i]:
                self.robots[i].offload(self.robots[id])
                self.robots[id].host(self.robots[i].get_self_task())
        
        # for r in self.robots:
            # print(r, r.get_self_task(), r.get_hosted_task())
        # print()
        
        return

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
                        self.robots[id].offload(robot)
                        assert robot.host(self.robots[id].get_self_task()) != False
                        found = True
                        break

    def update_stats(self, time_instant):
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
        n_computation = {}
        
        for _, robot in enumerate(self.robots):
            stat = robot.get_stats()
            offloaded_computing[robot.name] = stat["offloaded_computing"]
            operation_time[robot.name] = stat["operation_time"]
            charging_time[robot.name] = stat["charging_time"]
            n_charging[robot.name] = stat["n_charging"]
            n_operating[robot.name] = stat["n_operating"]
            n_offloaded[robot.name] = stat["n_offloaded"]
            n_hosted[robot.name] = stat["n_hosted"]
            n_computation[robot.name] = stat["computation"]
            
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
        pd.DataFrame([n_computation]).to_csv(f"{self.sim_name}/n_computation.csv", index=False)
        
        pd.DataFrame([self.stats]).to_csv(f"{self.sim_name}/simulation_stats.csv", index=False)
             
    def plot_results(self, data):
        """
        Plot the battery levels of each robot over time.

        Args:
            data (dict): Dictionary containing battery levels for each robot.
        """
        num_robots = len(data)
        _, axs = plt.subplots(num_robots, 1, figsize=(8, 6*num_robots))
        
        for i, (robot, values) in enumerate(data.items()):
            axs[i].plot(values, label=robot)
            axs[i].set_xlabel('Time')
            axs[i].set_ylabel('Battery Level')
            axs[i].set_title(f'Battery Levels of {robot} Over Time')
            axs[i].legend()
        
        # Save the plot as an image file
        plt.savefig(self.sim_name + "/battery_levels.png")
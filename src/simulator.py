import copy
from src.robot import Robot
from src.mpc import BruteForceAllocation
import random
import numpy as np
import pandas as pd
from src.utils import compute_adjacency_matrix, progress_simulation
import os
from tqdm import tqdm
import sys

import matplotlib.pyplot as plt

class Simulator:
    def __init__(self, run_number, sim_name, charging_threshold=0.05, operating_threshold=0.95, probability=1, move_computation_enabled=True, optimize_computation_frequency=None, optimize_computation_window=10, config=None) -> None:
        if config is None:
            print("ERROR: No configuration provided.")
            sys.exit(1)
        
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        self.robots = []
        self.move_computation_enabled = move_computation_enabled
        self.optimize_computation_frequency = optimize_computation_frequency
        self.sim_name = "res/" + sim_name
        self.optimize_computation_window = optimize_computation_window
        
        if optimize_computation_frequency is not None:
            self.allocator = BruteForceAllocation(config["n_robots"])
        
        self.initialize_stats()
        
        random.seed(run_number)
        
        # Create n_robots instances of the Robot class with random battery levels, charge rates, and discharge rates
        for i in range(config["n_robots"]):
            dr = config["discharge_rate"]
            tb = config["total_battery"]
            cr = config["charge_rate"]
            td = config["AI_computation"]
            bl = random.randint(int(tb*0.15), int(tb*0.85))
            # bl = tb
            
            if dr == 0:
                self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status=random.choice(["charging", "operating"])))
            else:
                self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status=random.choice(["charging", "operating"])))
                # self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status="operating"))

            
        if probability != 1:
            print("WARNING: The code has not being tested with probability != 1. Unexpected results may arise.")
        
        # Compute probability-defined adjacency matrix 
        self.adjacency_matrix = compute_adjacency_matrix(config["n_robots"], probability)   
        
        print(f"Initialized simulation with {tb} total battery, {config['n_robots']} robots, charge rate {cr}, discharge rate {dr}.")
        
    def initialize_stats(self):
        """
        Initialize the statistics dictionary.
        """
        self.stats = {}
        self.stats["wasted_charging"] = 0
        self.stats["wasted_operating"] = 0
        
        self.stats_status_robot = {}
        self.stats_status_robot["epoch"] = []
        self.stats_status_robot["charging"] = []
        self.stats_status_robot["operating"] = []
            
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
            progress_simulation(res, self.robots, self.charging_threshold, self.operating_threshold, self.move_computation_enabled, self.adjacency_matrix)
                
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
        constrained_allocation = [-1 for _ in range(len(self.robots))]
                
        for id, r in enumerate(self.robots):
            if r.get_status() == "charging":
                if r.get_hosted_task() is not None:
                    constrained_allocation[r.get_hosted_task().get_from().get_name()] = id
                constrained_allocation[id] = id
        
        #self.print_infrastructure(0)
        offloading_decision_brute = self.allocator.find_best_allocation(self.optimize_computation_window, copy.deepcopy(self.robots), self.charging_threshold, self.operating_threshold, self.move_computation_enabled, self.adjacency_matrix, constrained_allocation)
        
        for r in self.robots:
            r.unhost()
            r.unoffload()
            
        for i, id in enumerate(offloading_decision_brute):     
            if self.robots[id] != self.robots[i]:
                self.robots[i].offload(self.robots[id])
                self.robots[id].host(self.robots[i].get_self_task())
                
        #self.print_infrastructure(0)
                
        return

    def update_stats(self, time_instant):
        """
        Update the simulation statistics.
        """
        charging = 0
        operating = 0
        
        for robot in self.robots:
            if robot.get_status() == "charging" and not robot.is_hosting():
                self.stats["wasted_charging"] += 1
            elif robot.get_status() == "operating" and not robot.has_offloaded():
                self.stats["wasted_operating"] += 1
                
            if robot.get_status() == "charging":
                charging += 1
            elif robot.get_status() == "operating":
                operating += 1
                
        self.stats_status_robot["epoch"].append(time_instant)
        self.stats_status_robot["charging"].append(charging)
        self.stats_status_robot["operating"].append(operating)

    def dump_report(self):
        """
        Dump the simulation report to CSV files.
        """        
        d = {}
        
        for _, robot in enumerate(self.robots):
            stat = robot.get_stats()
            
            d["n_robot"] = len(self.robots)
            d["robot_" + str(robot.name) + "_offloaded_computing"] = stat["offloaded_computing"]
            d["robot_" + str(robot.name) + "_operation_time"] = stat["operation_time"]
            d["robot_" + str(robot.name) + "_charging_time"] = stat["charging_time"]
            d["robot_" + str(robot.name) + "_n_charging"] = stat["n_charging"]
            d["robot_" + str(robot.name) + "_n_operating"] = stat["n_operating"]
            d["robot_" + str(robot.name) + "_n_offloaded"] = stat["n_offloaded"]
            d["robot_" + str(robot.name) + "_n_hosted"] = stat["n_hosted"]
            d["robot_" + str(robot.name) + "_n_computation"] = stat["computation"]
            
        # Create the directory if it doesn't exist
        if not os.path.exists(self.sim_name):
            os.makedirs(self.sim_name)

        # Save the dataframes as CSV files in the directory
        pd.DataFrame([d]).to_csv(f"{self.sim_name}/simulation_stats.csv", index=False)
        pd.DataFrame([self.stats]).to_csv(f"{self.sim_name}/missed_chances.csv", index=False)
        pd.DataFrame(self.stats_status_robot).to_csv(f"{self.sim_name}/robot_status.csv", index=False)
             
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
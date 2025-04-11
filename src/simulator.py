import copy
import json

import numpy as np
from src.robot import Robot
from src.mpc import Allocator, AllocationPolicy
import random
import pandas as pd
from src.utils import compute_adjacency_matrix, move_computation, tick, MoveComputationPolicy
import os
from tqdm import tqdm
from datetime import datetime

import matplotlib.pyplot as plt

class Simulator:
    def __init__(self, run_number, config, move_computation_policies=[MoveComputationPolicy.NONE], allocation_strategies=[], report_dir=None) -> None: 
        """
        Initialize the simulator with the given parameters.

        Args:
            run_number (int): The number of runs for the simulation.
            config (dict): Configuration dictionary containing simulation parameters.
            move_computation_policies (list, optional): List of move computation policies. Defaults to [MoveComputationPolicy.NONE].
            allocation_strategies (list, optional): List of allocation strategies. Defaults to an empty list.
            report_dir (str, optional): Directory to save the simulation reports. Defaults to None.
        """
        self.charging_threshold = config["charging_threshold"]
        self.operating_threshold = config["operating_threshold"]
        self.__robots_backup = []
        self.move_computation_policies = move_computation_policies
        if report_dir is None:
            self.sim_name = "results-" + datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
        else:
            self.sim_name = report_dir
        self.allocation_strategies = allocation_strategies
        self.config = config
        self.n_runs = run_number

        if not os.path.exists(self.sim_name):
            os.makedirs(self.sim_name)
        
        
    def initialize_robots(self, run_number):
        random.seed(run_number)

        self.robots = []
        
        # Create n_robots instances of the Robot class with random battery levels, charge rates, and discharge rates
        for i in range(self.config["n_robots"]):
            dr = self.config["discharge_rate"]
            tb = self.config["total_battery"]
            cr = self.config["charge_rate"]
            td = self.config["AI_computation"]
            bl = random.randint(int(tb*0.15), int(tb*0.85))
            # bl = tb
            
            if dr == 0:
                self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status=random.choice(["charging", "operating"])))
            else:
                self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status=random.choice(["charging", "operating"])))
                # self.robots.append(Robot(i, battery_level=bl, total_battery=tb, charge_rate=cr, disharge_rate=dr, task_demand=td, status="operating"))
      
        self.__robots_backup = copy.deepcopy(self.robots)

        print(f"Initialized simulation with {tb} total battery, {self.config['n_robots']} robots, charge rate {cr}, discharge rate {dr}.")
        
        return
        
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
        offset = 0
        for dir in os.listdir(self.sim_name):
            if int(dir) > offset:
                offset = int(dir)

        if offset != 0:
            offset += 1

        for it in range(self.n_runs):
            self.initialize_stats()   
            self.initialize_robots(it)
            # Compute probability-defined adjacency matrix 
            self.adjacency_matrix = compute_adjacency_matrix(self.config["n_robots"], 1)   

            # Run the referebce simulation
            self.allocator = None
            self.robots = copy.deepcopy(self.__robots_backup)
            self.move_computation_policy = MoveComputationPolicy.NONE
            self.optimize_computation_frequency = None
            self.optimize_computation_window = None
            self.__run(epochs, it+offset)

            for move_computation_policy in self.move_computation_policies:
                self.initialize_stats()   
                self.allocator = None
                self.robots = copy.deepcopy(self.__robots_backup)
                self.move_computation_policy = move_computation_policy
                self.optimize_computation_frequency = None
                self.optimize_computation_window = None
                self.__run(epochs, it+offset)
                
            for allocation_strategy in self.allocation_strategies:
                self.initialize_stats()   
                self.robots = copy.deepcopy(self.__robots_backup)
                self.move_computation_policy = None
                self.allocator = Allocator(len(self.robots), allocation_strategy, charging_threshold=self.charging_threshold, operating_threshold=self.operating_threshold, tot_epochs=epochs)
                self.optimize_computation_frequency = allocation_strategy.optimize_computation_frequency
                self.optimize_computation_window = allocation_strategy.optimize_computation_window
                self.__run(epochs, it+offset)

    def __run(self, epochs, iter):
        res = {}
        self.epochs = epochs

        x = []
        u = []
        o = []
        
        # Initialize a dictionary to store battery levels for each robot
        for r in self.robots:
            res[r.name] = []
            x.append([])
            u.append([])
            o.append([])
            
        for ep in tqdm(range(epochs), desc = 'Simulating epoch: ', smoothing=0):
            # for id, r in enumerate(self.robots):
            #     x[id].append(r.get_battery_level())
            #     u[id].append(1 if r.get_status() == "operating" else 0)
            #     o[id].append(1 if r.has_offloaded() else 0)
                
            self.progress_simulation(res, ep)
                
            for r in self.robots:
                r.update_computation()
            
            assert self.check_infrastructure(), self.print_infrastructure(ep)
            self.update_stats(ep)

        # for id, r in enumerate(self.robots):
        #     x[id].append(r.get_battery_level())
        #     u[id].append(1 if r.get_status() == "operating" else 0)
        #     o[id].append(1 if r.has_offloaded() else 0)

        # if self.allocator is None and self.move_computation_policy != MoveComputationPolicy.NONE:
        #     with open("tmp-bat", "w") as f:
        #         json.dump(self.marshal_variables(x, u, o), f, indent=4)

        # if self.allocator is not None:
        #     with open("tmp-mpc", "w") as f:
        #         json.dump(self.marshal_variables(x, u, o), f, indent=4)

        if self.allocator is not None:
            self.allocator.terminate()
            
        self.dump_report(iter) 
        # self.plot_results(res)
    
    def marshal_variables(self, x, u, o):
        """
        Serialize the variables x, u, and o into a dictionary.

        Args:
            x (list): Battery levels of robots over time.
            u (list): Operating status of robots over time.
            o (list): Offloading status of robots over time.

        Returns:
            dict: Serialized representation of x, u, and o.
        """
        return {
            "x": x,
            "u": u,
            "o": o
        }
        
    def progress_simulation(self, res, ep):                                
        # Use available robots to host tasks
        if self.move_computation_policy is not None:
            available_robots_ids = tick(res, self.robots, self.operating_threshold, self.charging_threshold)
                    
            if self.move_computation_policy != MoveComputationPolicy.NONE:
                move_computation(available_robots_ids, self.robots, self.adjacency_matrix, self.move_computation_policy)
            
        if self.optimize_computation_frequency is not None and ep%self.optimize_computation_frequency == 0:
            self.optimize_computation(ep)
            
    def optimize_computation(self, ep=0):            
        for r in self.robots:
            r.tick()
        
        _, status, offload = self.allocator.find_best_allocation_new(copy.deepcopy(self.robots), ep)
        
        for r in self.robots:
            r.unhost()
            r.unoffload()

        charging_ids = []
        for id, s in enumerate(status):
            if s == 1 and self.robots[id].get_status() == "charging":
                self.robots[id].operate()
            elif s == 0 and self.robots[id].get_status() == "operating":
                self.robots[id].charge()

            if self.robots[id].get_status() == "charging":
                charging_ids.append(id)
        
        for id, o in enumerate(offload):
            if o == 1:
                iid = charging_ids.pop(0)
                self.robots[id].offload(self.robots[iid])  
                assert self.robots[iid].host(self.robots[id].get_self_task()) != False  
                                
        return
    
    def compute_delay_solutions(self, targets, duration):
        def sol_r(n, targets, sol, all_sol):
            if n == len(targets):
                all_sol.append(copy.deepcopy(sol))
                return
            
            for i in range(duration):
                sol[n] = i * 5
                sol_r(n+1, targets, sol, all_sol)
        
        sol = [-1 for _ in range(len(targets))]
        all_sol = []
        
        sol_r(0, targets, sol, all_sol)
        
        return all_sol
            
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

    def dump_report(self, iter):
        """
        Dump the simulation report to CSV files.
        """        
        d = {}
        
        for _, robot in enumerate(self.robots):
            stat = robot.get_stats()
            
            d["n_robot"] = len(self.robots)
            d["robot_" + str(robot.name) + "_operation_time"] = stat["operation_time"]
            d["robot_" + str(robot.name) + "_charging_time"] = stat["charging_time"]
            d["robot_" + str(robot.name) + "_n_charging"] = stat["n_charging"]
            d["robot_" + str(robot.name) + "_n_operating"] = stat["n_operating"]
            d["robot_" + str(robot.name) + "_n_offloaded"] = stat["n_offloaded"]
            d["robot_" + str(robot.name) + "_n_hosted"] = stat["n_hosted"]
            d["robot_" + str(robot.name) + "_n_computation"] = stat["computation"]
            d["robot_" + str(robot.name) + "_free_computing"] = stat["free_computing"]
            d["robot_" + str(robot.name) + "_self_computing"] = stat["self_computing"]
            d["robot_" + str(robot.name) + "_offload_computing"] = stat["offload_computing"]
            
        if self.move_computation_policy is not None:
            conf = str(self.move_computation_policy)
        else:
            if self.allocator.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OP or self.allocator.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OF:
                conf = str(self.allocator.allocation_policy) + str(self.allocator.alloc_strategy.increment) + f"_{self.optimize_computation_window}"
            if self.allocator.allocation_policy is AllocationPolicy.MPC_OPT_OP or self.allocator.allocation_policy is AllocationPolicy.MPC_OPT_OF:
                conf = str(self.allocator.allocation_policy) + f"_{self.optimize_computation_window}"

        # Create the directory if it doesn't exist
        if not os.path.exists(f"{self.sim_name}/{iter}/{conf}"):
            os.makedirs(f"{self.sim_name}/{iter}/{conf}")

        # Save the dataframes as CSV files in the directory
        pd.DataFrame([d]).to_csv(f"{self.sim_name}/{iter}/{conf}/simulation_stats.csv", index=False)
        pd.DataFrame([self.stats]).to_csv(f"{self.sim_name}/{iter}/{conf}/missed_chances.csv", index=False)
        pd.DataFrame(self.stats_status_robot).to_csv(f"{self.sim_name}/{iter}/{conf}/robot_status.csv", index=False)
             
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
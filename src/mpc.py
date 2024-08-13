import copy
import numpy as np
from scipy.optimize import basinhopping
from itertools import permutations
from collections import Counter
import time
from src.utils import progress_simulation

def optimize(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants):
    res = []

    for _ in range(time_instants):
        charging = 0
        operating = 0
        for r in robots:
            if r.get_status() == "charging":
                charging += 1
            elif r.get_status() == "operating":
                operating += 1 

        res.append((charging - operating) ** 2)
        progress_simulation({}, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix)
    
    return np.sum(res)        

class IntegerBounds:
    def __init__(self, xmin, xmax):
        self.xmin = np.array(xmin)
        self.xmax = np.array(xmax)

    def __call__(self, **kwargs):
        x = kwargs["x_new"]
        tmax = bool(np.all(x <= self.xmax))
        tmin = bool(np.all(x >= self.xmin))
        return tmax and tmin

def mpc(task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, time_instants):
    initial_guess = [i for i in range(len(task_requirements))]
    bounds = IntegerBounds([0] * len(task_requirements), [len(battery_levels) - 1] * len(task_requirements))
    minimizer_kwargs = {"method": "L-BFGS-B", "args": (task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, time_instants)}
    
    res = basinhopping(optimize, initial_guess, minimizer_kwargs=minimizer_kwargs, niter=100, accept_test=bounds)
    
    return np.round(res.x).astype(int)

class BruteForceAllocation:
    def __init__(self, n_robots):
        self.n_robots = n_robots
        self.alloc_options = self.custom_powerset()
        
    def custom_powerset(self):
        #print("Computing powerset...")
        res = []
        current = [0] * self.n_robots
        self._rec_custom_powerser(current, res, 0)
        return res
    
    def print_powerset(self):
        for i in self.alloc_options:
            print(i)
            
    def print_powerset_count(self):
        print(len(self.alloc_options))
    
    def _rec_custom_powerser(self, current, result, index):
        if index == self.n_robots:
            result.append(copy.deepcopy(current))
            return

        for i in range(self.n_robots):
            current[index] = i
            if self._is_consistent(current, index + 1):
                self._rec_custom_powerser(current, result, index + 1)
    
    def _is_consistent(self, current, index):
        return self._validate_count(current, index)
    
    def _validate_with_constraints(self, allocation, constrained_allocation, index=None):
        if constrained_allocation is None:
            return True
        
        count = 0
        for i, j in zip(allocation, constrained_allocation):
            if j != -1 and i != j:
                return False
            count += 1
            if index is not None and count == index:
                break
        
        return True
    
    def _validate_count(self, current, index):
        occurrences = [0 for _ in range(self.n_robots)]
        for i in range(index):
            occurrences[current[i]] += 1
            
        for id1, o in enumerate(occurrences):
            # if a robot is used more than twice, return False
            # NOTE: this is a constraint that can be removed
            if o > 2:
                return False
            
            # if a robot is used twice, it must be used once by the same robot
            if o == 2:
                found = False
                for j in range(index):
                    if current[j] == id1 and j == id1:
                        found = True
                if not found:
                    return False
                
            # if o == 1:
            #     if id1 != current[id1]:
            #         return False
        
        return True
    
    def find_best_allocation(self, time_instants, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, costrained_allocation=None):
        best_cost = np.inf
        best_solution = None
        robots_backup = copy.deepcopy(robots)

        for alloc in self.alloc_options:
            #offloading_decision = list(p)
            if not self._validate_with_constraints(alloc, costrained_allocation):
                continue

            robots = copy.deepcopy(robots_backup)
            
            for r in robots:
                r.unhost()
                r.unoffload()
                
            for i, id in enumerate(alloc):     
                if robots[id] != robots[i]:
                    robots[i].offload(robots[id])
                    robots[id].host(robots[i].get_self_task())                
            
            cost = optimize(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants)
            if cost < best_cost:
                best_cost = cost
                best_solution = alloc

        # how to read: task (owned by robot) i is executed by robot[best_solution[i]]
        return best_solution
    

if __name__ == "__main__":
    # Example usage
    task_requirements = [20, 20]
    battery_levels = [80, 20]
    battery_status = ['operating', 'operating']
    discharge_rate = [5, 5]
    charge_rate = [5, 5]
    costrained_allocation = [-1, 1]
    time_instants = 5

    bf = BruteForceAllocation(4)
    bf.print_powerset_count()
    # bf.print_powerset()
    #print(bf.find_best_allocation(task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, time_instants, costrained_allocation))
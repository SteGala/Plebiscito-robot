import copy
from enum import Enum
import numpy as np
from src.utils import tick, move_computation, MoveComputationPolicy
import sys
import multiprocessing as mp
import cvxpy as cp

class AllocationPolicy(Enum):
    MPC = 1
    MPC_REDUCE_FLIP_A1 = 2
    NONE = 3

    def __str__(self):
        if self is AllocationPolicy.MPC:
            return "MPC"
        elif self is AllocationPolicy.MPC_REDUCE_FLIP_A1:
            return "MPC_REDUCE_FLIP_ALPA1"
        elif self is AllocationPolicy.NONE:
            return ""

class AllocationStrategy:
    def __init__(self, alloc, optimize_computation_frequency=50, optimize_computation_window=50, num_processes=1) -> None:
        self.alloc = alloc
        self.optimize_computation_frequency = optimize_computation_frequency
        self.optimize_computation_window = optimize_computation_window
        self.num_processes = num_processes
        
class Allocator:
    def __init__(self, n_robots, alloc_strategy=AllocationStrategy(alloc=AllocationPolicy.MPC), charging_threshold=0, operating_threshold=1):#, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
        self.n_robots = n_robots
        self.alloc_strategy = alloc_strategy
        self.allocation_policy = alloc_strategy.alloc
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
            
        print(cp.installed_solvers())
        if self.allocation_policy is AllocationPolicy.MPC or self.allocation_policy is AllocationPolicy.MPC_REDUCE_FLIP_A1:
            pass
        else:
            print(f"Allocation policy {self.allocation_policy} not yet supported.")
            sys.exit(1)

    def terminate(self):
        pass
    
    def __mpc(self, robots):
        N = len(robots)
        T = self.alloc_strategy.optimize_computation_window
        consume_rate = robots[0].get_discharge_rate()
        charge_rate = robots[0].get_charge_rate()
        computation_cost = robots[0].get_self_task().get_consumption()
        max_battery = robots[0].get_total_battery()
        
        b_low = self.charging_threshold * max_battery
        b_high = self.operating_threshold * max_battery
        
        # Variabili decisionali
        x = cp.Variable((N, T))  # Livello di batteria per ogni robot nel tempo
        u = cp.Variable((N, T), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
        o = cp.Variable((N, T), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
        
        constraints = []
        
        # Stato iniziale (vincolo invece di assegnazione diretta)
        x_init = [r.get_battery_level() for r in robots]
        constraints.append(x[:, 0] == x_init)  # Vincolo per i livelli iniziali della batteria
        u_init = [1 if r.get_status() == "operating" else 0 for r in robots]
        constraints.append(u[:, 0] == u_init)  # Vincolo per lo stato iniziale degli operatori
        o_init = [1 if r.has_offloaded() else 0 for r in robots]
        constraints.append(o[:, 0] == o_init)  # Vincolo per lo stato iniziale dell'offloading
        
        for t in range(T - 1):
            for i in range(N):
                constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * u[i, t] - computation_cost * (u[i, t] - o[i, t]))
                constraints.append(x[i, t+1] >= b_low)
                constraints.append(x[i, t+1] <= b_high)
                
                constraints.append(o[i, t] <= u[i, t])
            
            constraints.append(cp.sum(o[:, t]) <= N - cp.sum(u[:, t]))
        
        if self.allocation_policy is AllocationPolicy.MPC:    
            objective = cp.Maximize(cp.sum(u))
        elif self.allocation_policy is AllocationPolicy.MPC_REDUCE_FLIP_A1:
            objective = cp.Maximize(cp.sum(u) - cp.sum(cp.abs(u[:, 1:] - u[:, :-1])))
        
        # Risoluzione del problema
        problem = cp.Problem(objective, constraints)
        solver = cp.GUROBI if "GUROBI" in cp.installed_solvers() else "SCIP" if "SCIP" in cp.installed_solvers() else "GLPK_MI"
        problem.solve(solver=solver, TimeLimit=20)
        
        return x.value, u.value, o.value#, w.value
    
    def find_best_allocation_new(self, robots): 
        return self.__mpc(robots)
    
import copy
from enum import Enum
import json
import numpy as np
from src.utils import tick, move_computation, MoveComputationPolicy
import sys
import multiprocessing as mp
import cvxpy as cp

class AllocationPolicy(Enum):
    MPC_OPT_OP = 1
    MPC_OPT_OF = 2
    MPC_INCREMENTAL_OPT_OP = 3
    MPC_INCREMENTAL_OPT_OF = 4

    def __str__(self):
        if self is AllocationPolicy.MPC_OPT_OP:
            return "MPC_OP"
        elif self is AllocationPolicy.MPC_OPT_OF:
            return "MPC_OF"
        elif self is AllocationPolicy.MPC_INCREMENTAL_OPT_OF:
            return "MPC_INC_OF"
        elif self is AllocationPolicy.MPC_INCREMENTAL_OPT_OP:
            return "MPC_INC_OP"
        else:
            return "UNKNOWN"

class AllocationStrategy:
    def __init__(self, alloc, optimize_computation_frequency=50, optimize_computation_window=50, num_processes=1, increment=1) -> None:
        self.alloc = alloc
        self.optimize_computation_frequency = optimize_computation_frequency
        self.optimize_computation_window = optimize_computation_window
        self.num_processes = num_processes
        self.increment = increment
        
class Allocator:
    def __init__(self, n_robots, alloc_strategy=AllocationStrategy(alloc=AllocationPolicy.MPC_OPT_OP, increment=0), charging_threshold=0, operating_threshold=1, tot_epochs=10):#, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
        self.n_robots = n_robots
        self.alloc_strategy = alloc_strategy
        self.allocation_policy = alloc_strategy.alloc
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        self.tot_epochs = tot_epochs
        
        self.x = None
        self.u = None
        self.o = None
            
    def terminate(self):
        pass
    
    def __mpc(self, robots, iter):
        N = len(robots)
        T = self.alloc_strategy.optimize_computation_window
        consume_rate = robots[0].get_discharge_rate()
        charge_rate = robots[0].get_charge_rate()
        computation_cost = robots[0].get_self_task().get_consumption()
        max_battery = robots[0].get_total_battery()
        
        b_low = self.charging_threshold * max_battery - consume_rate - computation_cost
        b_high = self.operating_threshold * max_battery + charge_rate

        retry = 5
        while retry > 0:
            # something went wrong, reset the state
            if self.x is not None:
                for id, r in enumerate(robots):
                    if r.get_battery_level() != self.x[id][1]:
                        self.x = None
                        self.u = None
                        self.o = None
                        break
            
            # Variabili decisionali
            x = cp.Variable((N, T), integer=True)  # Livello di batteria per ogni robot nel tempo
            u = cp.Variable((N, T), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
            o = cp.Variable((N, T), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
            
            constraints = []
            start_id = 0
            
            if self.x is None:
                # Stato iniziale (vincolo invece di assegnazione diretta)
                x_init = [r.get_battery_level() for r in robots]
                constraints.append(x[:, 0] == x_init)  # Vincolo per i livelli iniziali della batteria
            else:
                end_idx = 0
                if self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OP or self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OF:
                    end_idx = T - self.alloc_strategy.increment 
                
                for t in range(end_idx):
                    constraints.append(x[:, t] == self.x[:, t+1])
                    if t != end_idx - 1:
                        constraints.append(u[:, t] == self.u[:, t+1])
                        constraints.append(o[:, t] == self.o[:, t+1])
                
                start_id = end_idx - 1
                        
            for t in range(start_id, T-1):
                for i in range(N):
                    constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * u[i, t] - computation_cost * (u[i, t] - o[i, t]))
                    constraints.append(x[i, t+1] >= b_low)
                    constraints.append(x[i, t+1] <= b_high)
                    
                    constraints.append(o[i, t] <= u[i, t])
                
                constraints.append(cp.sum(o[:, t]) <= N - cp.sum(u[:, t]))

            # constraints.append(cp.max(x) <= b_high)
            # constraints.append(cp.min(x) >= b_low)
            
            if self.allocation_policy is AllocationPolicy.MPC_OPT_OP or self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OP:
                objective = cp.Maximize(cp.sum(u[:, :T-1]))
            elif self.allocation_policy is AllocationPolicy.MPC_OPT_OF or self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL_OPT_OF:
                objective = cp.Maximize(cp.sum(o[:, :T-1]))
            
            # Risoluzione del problema
            problem = cp.Problem(objective, constraints)
            solver = cp.GUROBI if "GUROBI" in cp.installed_solvers() else "SCIP" if "SCIP" in cp.installed_solvers() else "GLPK_MI"
            problem.solve(solver=solver, TimeLimit=20, OutputFlag=0, LogToConsole=0)

            # if the solution is not found, retry
            if x.value is None:
                self.x = None
                self.u = None
                self.o = None
                retry -= 1
                continue
                        
            if self.allocation_policy is not AllocationPolicy.MPC_OPT_OP and self.allocation_policy is not AllocationPolicy.MPC_OPT_OF:
                self.x = copy.deepcopy(x.value)
                self.u = copy.deepcopy(u.value)
                self.o = copy.deepcopy(o.value)
                    
            #print(x.value)
            return x.value[:, 0], u.value[:, 0], o.value[:, 0]#, w.value
        
        sys.exit(-1)
    
    def find_best_allocation_new(self, robots, ep): 
        return self.__mpc(robots, ep)
    
import copy
from enum import Enum
import json
import numpy as np
from src.utils import tick, move_computation, MoveComputationPolicy
import sys
import multiprocessing as mp
import cvxpy as cp

class AllocationPolicy(Enum):
    MPC = 1
    MPC_INCREMENTAL = 2
    MPC_INCREMENTAL2 = 3
    MPC_INCREMENTAL3 = 4
    MPC_INCREMENTAL4 = 5
    MPC_INCREMENTAL5 = 6
    MPC_INCREMENTAL6 = 7
    MPC_INCREMENTAL7 = 8

    def __str__(self):
        if self is AllocationPolicy.MPC:
            return "MPC"
        elif self is AllocationPolicy.MPC_INCREMENTAL:
            return "MPC_INC1"
        elif self is AllocationPolicy.MPC_INCREMENTAL2:
            return "MPC_INC2"
        elif self is AllocationPolicy.MPC_INCREMENTAL3:
            return "MPC_INC3"
        elif self is AllocationPolicy.MPC_INCREMENTAL4:
            return "MPC_INC4"
        elif self is AllocationPolicy.MPC_INCREMENTAL5:
            return "MPC_INC5"
        elif self is AllocationPolicy.MPC_INCREMENTAL6:
            return "MPC_INC6"
        elif self is AllocationPolicy.MPC_INCREMENTAL7:
            return "MPC_INC7"
        else:
            return "UNKNOWN"

class AllocationStrategy:
    def __init__(self, alloc, optimize_computation_frequency=50, optimize_computation_window=50, num_processes=1) -> None:
        self.alloc = alloc
        self.optimize_computation_frequency = optimize_computation_frequency
        self.optimize_computation_window = optimize_computation_window
        self.num_processes = num_processes
        
class Allocator:
    def __init__(self, n_robots, alloc_strategy=AllocationStrategy(alloc=AllocationPolicy.MPC), charging_threshold=0, operating_threshold=1, tot_epochs=10):#, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
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

        with open("tmp-bat", "r") as f:
            data = json.load(f)

        x_s = np.array(data.get("x", []))
        u_s = np.array(data.get("u", []))
        o_s = np.array(data.get("o", []))

        self.x = x_s[:, iter+1:iter+T+1] if len(x_s[0, iter+1:iter+T+1]) >= 1 else None
        self.u = u_s[:, iter+1:iter+T+1] if len(u_s[0, iter+1:iter+T+1]) >= 1 else None
        self.o = o_s[:, iter+1:iter+T+1] if len(o_s[0, iter+1:iter+T+1]) >= 1 else None

        # something went wrong, reset the state
        # if self.x is not None:
        #     for id, r in enumerate(robots):
        #         if r.get_battery_level() != self.x[id][0]:
        #             self.x = None
        #             self.u = None
        #             self.o = None
        #             break
        
        # Variabili decisionali
        x = cp.Variable((N, min(T, self.tot_epochs-iter)))  # Livello di batteria per ogni robot nel tempo
        u = cp.Variable((N, min(T, self.tot_epochs-iter)), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
        o = cp.Variable((N, min(T, self.tot_epochs-iter)), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
        
        constraints = []
        start_id = 0
        
        if self.x is None:
            # Stato iniziale (vincolo invece di assegnazione diretta)
            x_init = [r.get_battery_level() for r in robots]
            constraints.append(x[:, 0] == x_init)  # Vincolo per i livelli iniziali della batteria
        else:
            end_idx = 0
            if self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL:
                end_idx = T - 1
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL2:
                end_idx = T - 2
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL3:
                end_idx = T - 3
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL4:
                end_idx = T - 4
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL5:
                end_idx = T - 5
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL6:
                end_idx = T - 6
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL7:
                end_idx = T - 7

            if end_idx <= len(self.x[0, :]):
                end_idx_n = end_idx
            else:
                end_idx_n = len(self.x[0, :])
            #end_idx_n = min(end_idx, len(self.x[0, :]))
            for t in range(end_idx_n):
                # print(type(list(self.x[i, 1:])), list(self.x[i, 1:]))
                constraints.append(x[:, t] == self.x[:, t])
                constraints.append(u[:, t] == self.u[:, t])
                constraints.append(o[:, t] == self.o[:, t])
            
            if end_idx_n == len(self.x[0, :]):
                start_id = T # just needs to be big enough
            else:
                start_id = end_idx_n - 1
                
        
        for t in range(start_id, min(T-1, self.tot_epochs-iter)):
            for i in range(N):
                constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * u[i, t] - computation_cost * (u[i, t] - o[i, t]))
                constraints.append(x[i, t+1] >= b_low)
                constraints.append(x[i, t+1] <= b_high)
                
                constraints.append(o[i, t] <= u[i, t])
            
            constraints.append(cp.sum(o[:, t]) <= N - cp.sum(u[:, t]))

        # constraints.append(cp.max(x) <= b_high)
        # constraints.append(cp.min(x) >= b_low)
        
        objective = cp.Maximize(cp.sum(u))
        
        # Risoluzione del problema
        problem = cp.Problem(objective, constraints)
        solver = cp.GUROBI if "GUROBI" in cp.installed_solvers() else "SCIP" if "SCIP" in cp.installed_solvers() else "GLPK_MI"
        problem.solve(solver=solver, TimeLimit=20, OutputFlag=0, LogToConsole=0)

        assert x.value is not None

        for i in range(len(x.value)):
            for j in range(len(x.value[i])-1):
                if x.value[i, j] != self.x[i, j]:
                    print(f"(i, j) = ({i}, {j})")
                    print("x:", x.value)
                    print("file:", self.x)
                    break
                    
        if self.allocation_policy is not AllocationPolicy.MPC:
            self.x = copy.deepcopy(x.value)
            self.u = copy.deepcopy(u.value)
            self.o = copy.deepcopy(o.value)
                
        return x.value[:, 0], u.value[:, 0], o.value[:, 0]#, w.value
    
    def find_best_allocation_new(self, robots, ep): 
        return self.__mpc(robots, ep)
    
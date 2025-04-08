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
    def __init__(self, n_robots, alloc_strategy=AllocationStrategy(alloc=AllocationPolicy.MPC), charging_threshold=0, operating_threshold=1):#, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
        self.n_robots = n_robots
        self.alloc_strategy = alloc_strategy
        self.allocation_policy = alloc_strategy.alloc
        self.charging_threshold = charging_threshold
        self.operating_threshold = operating_threshold
        
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
        
        b_low = self.charging_threshold * max_battery
        b_high = self.operating_threshold * max_battery

        with open("tmp", "r") as f:
            data = json.load(f)

        x_s = np.array(data.get("x", []))
        u_s = np.array(data.get("u", []))
        o_s = np.array(data.get("o", []))

        # x_s = [[8291, 8246, 8201, 8156, 8111, 8066, 8021, 7976, 7931, 7886, 7841, 7796, 7751, 7706, 7661, 7616, 7571, 7526, 7481, 7436], [2643, 2598, 2553, 2508, 2463, 2418, 2373, 2328, 2283, 2238, 2193, 2148, 2103, 2058, 2013, 1968, 1923, 1878, 1833, 1788], [10356, 10311, 10266, 10221, 10176, 10131, 10086, 10041, 9996, 9951, 9906, 9861, 9816, 9771, 9726, 9681, 9636, 9591, 9546, 9501], [8614, 8569, 8524, 8479, 8434, 8389, 8344, 8299, 8254, 8209, 8164, 8119, 8074, 8029, 7984, 7939, 7894, 7849, 7804, 7759], [9788, 9743, 9698, 9653, 9608, 9563, 9518, 9473, 9428, 9383, 9338, 9293, 9248, 9203, 9158, 9113, 9068, 9023, 8978, 8933], [5558, 5623, 5578, 5533, 5488, 5443, 5398, 5353, 5308, 5263, 5218, 5173, 5128, 5083, 5038, 4993, 4948, 4903, 4858, 4813], [6597, 6662, 6617, 6572, 6527, 6482, 6437, 6392, 6347, 6302, 6257, 6212, 6167, 6122, 6077, 6032, 5987, 5942, 5897, 5852], [3533, 3488, 3443, 3398, 3353, 3308, 3263, 3218, 3173, 3128, 3083, 3038, 2993, 2948, 2903, 2858, 2813, 2768, 2723, 2678], [10705, 10770, 10725, 10680, 10635, 10590, 10545, 10500, 10455, 10410, 10365, 10320, 10275, 10230, 10185, 10140, 10095, 10050, 10005, 9960], [7061, 7126, 7081, 7036, 6991, 6946, 6901, 6856, 6811, 6766, 6721, 6676, 6631, 6586, 6541, 6496, 6451, 6406, 6361, 6316], [3188, 3143, 3098, 3053, 3008, 2963, 2918, 2873, 2828, 2783, 2738, 2693, 2648, 2603, 2558, 2513, 2468, 2423, 2378, 2333], [9715, 9780, 9735, 9690, 9645, 9600, 9555, 9510, 9465, 9420, 9375, 9330, 9285, 9240, 9195, 9150, 9105, 9060, 9015, 8970], [7776, 7731, 7686, 7641, 7596, 7551, 7506, 7461, 7416, 7371, 7326, 7281, 7236, 7191, 7146, 7101, 7056, 7011, 6966, 6921], [7160, 7225, 7180, 7135, 7090, 7045, 7000, 6955, 6910, 6865, 6820, 6775, 6730, 6685, 6640, 6595, 6550, 6505, 6460, 6415], [11032, 10987, 10942, 10897, 10852, 10807, 10762, 10717, 10672, 10627, 10582, 10537, 10492, 10447, 10402, 10357, 10312, 10267, 10222, 10177]]
        # u_s = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
        # o_s = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        
        self.x = x_s[:, iter:iter+T+1]
        self.u = u_s[:, iter:iter+T+1]
        self.o = o_s[:, iter:iter+T+1]

        if iter == 10:
            pass

        # something went wrong, reset the state
        if self.x is not None:
            for id, r in enumerate(robots):
                if r.get_battery_level() != self.x[id][1]:
                    self.x = None
                    self.u = None
                    self.o = None
                    break
        
        # Variabili decisionali
        x = cp.Variable((N, T))  # Livello di batteria per ogni robot nel tempo
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
            if self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL:
                end_idx = T - 2
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL2:
                end_idx = T - 3
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL3:
                end_idx = T - 4
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL4:
                end_idx = T - 5
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL5:
                end_idx = T - 6
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL6:
                end_idx = T - 7
            elif self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL7:
                end_idx = T - 8

            for t in range(end_idx):
                # print(type(list(self.x[i, 1:])), list(self.x[i, 1:]))
                constraints.append(x[:, t] == list(self.x[:, t+1]))
                constraints.append(u[:, t] == list(self.u[:, t+1]))
                constraints.append(o[:, t] == list(self.o[:, t+1]))
            
            start_id = end_idx - 1
        
        for t in range(start_id, T - 1):
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

        if x.value is None:
            pass
        assert x.value is not None

        # print("status", u.value)
        # print("offload", o.value)
        
        if self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL or self.allocation_policy is AllocationPolicy.MPC_INCREMENTAL2:
            self.x = copy.deepcopy(x.value)
            self.u = copy.deepcopy(u.value)
            self.o = copy.deepcopy(o.value)
                
        return x.value[:, 0], u.value[:, 0], o.value[:, 0]#, w.value
    
    def find_best_allocation_new(self, robots, ep): 
        return self.__mpc(robots, ep)
    
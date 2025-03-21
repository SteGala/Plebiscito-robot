import copy
from enum import Enum
import numpy as np
from src.utils import tick, move_computation, MoveComputationPolicy
import sys
import multiprocessing as mp
import cvxpy as cp

class AllocationPolicy(Enum):
    BRUTE_FORCE = 1
    MOVE1 = 2
    MOVE2 = 3
    MOVE3 = 4
    MPC = 5
    NONE = 6

    def __str__(self):
        if self is AllocationPolicy.BRUTE_FORCE:
            return "Brute_force"
        elif self is AllocationPolicy.MOVE1:
            return "Move1"
        elif self is AllocationPolicy.MOVE2:
            return "Move2"
        elif self is AllocationPolicy.MOVE3:
            return "Move3"
        elif self is AllocationPolicy.MPC:
            return "MPC"
        elif self is AllocationPolicy.NONE:
            return ""

class AllocationStrategy:
    def __init__(self, alloc, optimize_computation_frequency=50, optimize_computation_window=50, num_processes=1) -> None:
        self.alloc = alloc
        self.optimize_computation_frequency = optimize_computation_frequency
        self.optimize_computation_window = optimize_computation_window
        self.num_processes = num_processes

class ProcessPool:
    def __init__(self, n_processes, policy) -> None:
        self.n_processes = n_processes

        self.queue = mp.Queue()
        self.result_queue = mp.Queue()
        self.n_submitted = 0

        # Create and start the worker processes
        self.processes = []
        for _ in range(n_processes):
            p = mp.Process(target=self.work, args=(self.queue, self.result_queue, policy))
            p.start()
            self.processes.append(p)

    def terminate(self):
        for _ in range(self.n_processes):
            self.queue.put(None)

        for p in self.processes:
            p.join()

    def submit(self, rob, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, alloc, time_instants):
        self.n_submitted += 1
        self.queue.put({"robots": rob, "charging_threshold": charging_threshold, "operating_threshold": operating_threshold, "move_computation_enabled": move_computation_enabled, "adjacency_matrix": adjacency_matrix, "alloc": alloc, "time_instants": time_instants})

    def get_best_result(self):
        best_cost = np.inf
        best_alloc = None

        for _ in range(self.n_submitted):
            result = self.result_queue.get()
            if result["cost"] < best_cost:
                best_cost = result["cost"]
                best_alloc = result["alloc"]

        self.n_submitted = 0

        return best_alloc

    def work(self, queue, result_queue, policy):
        while True:
            # Get data from the queue
            data = queue.get()

            # If data is None, the worker will exit
            if data is None:
                break

            rob = data["robots"]
            charging_threshold = data["charging_threshold"]
            operating_threshold = data["operating_threshold"]
            move_computation_enabled = data["move_computation_enabled"]
            adjacency_matrix = data["adjacency_matrix"]
            alloc = data["alloc"]
            time_instants = data["time_instants"]

            for r in rob:
                r.unhost()
                r.unoffload()
                
            for i, id in enumerate(alloc):     
                # if rob[id] != robots[i]:
                rob[i].offload(rob[id])
                rob[id].host(rob[i].get_self_task()) 

            # cost = Allocator.optimize_missed_chanches(rob, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy)
            # cost = np.sum(cost)
            cost = Allocator.optimize_operation_time(rob, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy)
            cost = 1/np.sum(cost)
            
            # Push the result to the result_queue
            result_queue.put({"alloc": alloc, "cost": cost})

class Allocator:
    def __init__(self, n_robots, alloc_strategy=AllocationStrategy()):#, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
        self.n_robots = n_robots
        self.alloc_strategy = alloc_strategy
        self.allocation_policy = alloc_strategy.alloc
        #self.move_policy = move_policy
        # self.alloc_options = None
                
        # if alloc_policy is AllocationPolicy.BRUTE_FORCE:
        #     self.alloc_options = self.__custom_powerset()
            
        if self.allocation_policy is AllocationPolicy.MPC:
            pass
        else:
            print(f"Allocation policy {self.allocation_policy} not yet supported.")
            sys.exit(1)

        # if alloc_policy is not AllocationPolicy.MPC:
        #     self.process_pool = ProcessPool(n_processes, move_policy)

    def terminate(self):
        pass
        # if self.allocation_policy is not AllocationPolicy.MPC:
        #     self.process_pool.terminate()
            
    def __move_n_powerset(self, n, constrained_allocation, current_allocation):
        res = []
        alloc = [i for i in range(len(constrained_allocation))]
        res.append(copy.deepcopy(current_allocation))
                 
        self._rec_move_n_powerset(alloc, res, 0, n, constrained_allocation, current_allocation)
        return res
        
    def __custom_powerset(self):
        res = []
        current = [-1] * self.n_robots
        self.__rec_custom_powerser(current, res, 0)
        return res
    
    def print_powerset(self):
        for i in self.alloc_options:
            print(i)
            
    def print_powerset_count(self):
        print(len(self.alloc_options))
        
    def _rec_move_n_powerset(self, alloc, result, index, n, costrained_allocation, current_allocation):
        def is_allowed():
            ret = self.__validate_count(alloc, index+1)
            if ret is False:
                return False
            
            count = 0
            for i in range(index+1):
                if alloc[i] != current_allocation[i]:
                    count += 1
            if count > n:
                return False
            return True

        if index == self.n_robots:
            result.append(copy.deepcopy(alloc))
            return
        
        val = current_allocation[index]
        if costrained_allocation[index] != -1:
            alloc[index] = costrained_allocation[index]
            if is_allowed():   
                self._rec_move_n_powerset(alloc, result, index + 1, n, costrained_allocation, current_allocation)
        else:
            for i in range(self.n_robots):
                alloc[index] = i
                if is_allowed():   
                    self._rec_move_n_powerset(alloc, result, index + 1, n, costrained_allocation, current_allocation)
                alloc[index] = val
    
    def __rec_custom_powerser(self, current, result, index):
        if index == self.n_robots:
            result.append(copy.deepcopy(current))
            return

        for i in range(self.n_robots):
            current[index] = i
            if self.__is_consistent(current, index + 1):
                self.__rec_custom_powerser(current, result, index + 1)
            current[index] = -1

    def __is_consistent(self, current, index):
        return True and self.__validate_count(current, index) and self.__check_for_loop(current, index)# and self.__check_chain(current, index) # the last check remove some valid solutions
    
    def __validate_with_constraints(self, allocation, constrained_allocation, index=None):
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
    
    def __check_chain(self, current, index):            
        for i in range(index):
            id = current[i]
            if current[id] == -1:
                continue
            if current[id] != id:
                return False
        return True
    
    def __check_for_loop(self, current, index):
        for i in range(index):
            starting_point = i
            id = current[i]
            if starting_point == id:
                continue # no loop if the robot is hosting itself
            
            while True:
                if id == current[id]:
                    break
                id = current[id]
                if id == -1:
                    break
                if id == starting_point:
                    return False
        return True
    
    def __validate_count(self, current, index):
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
    
    def __mpc_new(self, robots):
        N = len(robots)
        T = self.alloc_strategy.optimize_computation_window
        consume_rate = robots[0].get_self_task().get_consumption()
        charge_rate = robots[0].get_charge_rate_percentage()
        computation_cost = robots[0].get_self_task().get_consumption()
        
        # Variabili decisionali
        x = cp.Variable((N, T))  # Livello di batteria per ogni robot nel tempo
        u = cp.Variable((N, T), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
        o = cp.Variable((N, T), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
        #w = cp.Variable((N, T))  # Variabile ausiliaria per eliminare il prodotto tra booleani
        
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
                constraints.append(x[i, t+1] >= 0)
                constraints.append(x[i, t+1] <= 100)
                
                constraints.append(o[i, t] <= u[i, t])
            
            constraints.append(cp.sum(o[:, t]) == N - cp.sum(u[:, t]))
            
        objective = cp.Maximize(cp.sum(u))
        
        # Risoluzione del problema
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.GUROBI if "GUROBI" in cp.installed_solvers() else cp.ECOS)
        
        return x.value, u.value, o.value#, w.value
    
    def __mpc(self, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants):
        # Define the number of robots and tasks
        num_robots = len(robots)
        num_tasks = len(robots)

        # State variable: x_ij is 1 if task j is assigned to robot i, else 0
        x = cp.Variable((num_robots, num_tasks), boolean=True)

        r_c = []
        r_d = []
        c = []
        b = []
        for r in robots:
            r_c.append(r.get_charge_rate_percentage())
            r_d.append(r.get_discharge_rate_percentage())
            c.append(r.get_self_task().get_consumption())
            b.append(r.get_total_battery())

        F_0 = 0
        for i in range(num_robots):
            F_0 += cp.log(r_c[i]*b[i]) - cp.log(r_c[i]*b[i] + r_d[i]*b[i] + cp.sum(cp.multiply(x[i, :], c)))

        # Objective: Maxize operation time
        objective = cp.Maximize(F_0)

        # Constraints
        constraints = []

        # 1. Each task must be assigned to exactly one robot
        for j in range(num_tasks):
            constraints.append(cp.sum(x[:, j]) == 1)

        # 2. Each robot can have at most two tasks
        # for i in range(num_robots):
        #     constraints.append(cp.sum(x[i, :]) <= 2)
            
        for i in range(num_robots):
            #constraints.append(cp.sum(x[i, :]) <= 2)
            constraints.append(x[i, i] >= cp.sum(x[i, :]) - 1)

        for id, r in enumerate(robots):
            if r.get_status() == "charging":
                constraints.append(x[id, id] == 1)
            
        

        prob = cp.Problem(objective, constraints)
        
        # Solve the problem
        prob.solve()

        # Final MPC task allocation and its cost
        final_allocation = np.round(x.value)
        
        res = [i for i in range(len(robots))]
        for id, r_alloc in enumerate(final_allocation):
            for id2, val in enumerate(r_alloc):
                if abs(val) == 1:
                    res[id2] = id
        
        return res
    
    def wrap(self, constrained_allocation, current_allocation):
        if self.allocation_policy is AllocationPolicy.MOVE1:
            return self.__move_n_powerset(1, constrained_allocation, current_allocation)
        elif self.allocation_policy is AllocationPolicy.MOVE2:
            return self.__move_n_powerset(2, constrained_allocation, current_allocation)
        elif self.allocation_policy is AllocationPolicy.MOVE3:
            return self.__move_n_powerset(3, constrained_allocation, current_allocation)
        
    def find_best_allocation_new(self, time_instants, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix): 
        constrained_allocation = [-1 for _ in range(len(robots))]
        current_allocation = [-1 for _ in range(len(robots))]
                
        for id, r in enumerate(robots):
            if r.get_status() == "charging":
                # if r.get_hosted_task() is not None:
                #     constrained_allocation[r.get_hosted_task().get_from().get_name()] = id
                constrained_allocation[id] = id
                current_allocation[id] = id
            else:
                current_allocation[id] = r.get_self_task().get_to().get_name()
            # if r.get_status() == "operating" and r.get_battery_percentage() < 0.5:
            #     constrained_allocation[id] = id
            
        if self.allocation_policy is AllocationPolicy.MPC:
            return self.__mpc_new(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants)
    
    def find_best_allocation(self, time_instants, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix):
        best_solution = None
        
        constrained_allocation = [-1 for _ in range(len(robots))]
        current_allocation = [-1 for _ in range(len(robots))]
                
        for id, r in enumerate(robots):
            if r.get_status() == "charging":
                # if r.get_hosted_task() is not None:
                #     constrained_allocation[r.get_hosted_task().get_from().get_name()] = id
                constrained_allocation[id] = id
                current_allocation[id] = id
            else:
                current_allocation[id] = r.get_self_task().get_to().get_name()
            # if r.get_status() == "operating" and r.get_battery_percentage() < 0.5:
            #     constrained_allocation[id] = id

        if self.allocation_policy is AllocationPolicy.MPC:
            return self.__mpc(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants)
        
        if self.alloc_options is None:
            if self.allocation_policy is AllocationPolicy.MOVE1:
                self.alloc_options = self.__move_n_powerset(1, constrained_allocation, current_allocation)
            elif self.allocation_policy is AllocationPolicy.MOVE2:
                self.alloc_options = self.__move_n_powerset(2, constrained_allocation, current_allocation)
            elif self.allocation_policy is AllocationPolicy.MOVE3:
                self.alloc_options = self.__move_n_powerset(3, constrained_allocation, current_allocation)
        
        # for a in self.alloc_options:
        #     print(a)        
        # sys.exit(1)
        
        for alloc in self.alloc_options:
            # if not self.__validate_with_constraints(alloc, costrained_allocation):
            #     continue
                        
            self.process_pool.submit(copy.deepcopy(robots), charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, alloc, time_instants)               
            
        best_solution = self.process_pool.get_best_result()
                
        if self.allocation_policy is not AllocationPolicy.BRUTE_FORCE:
            self.alloc_options = None
                
        return best_solution
    
    @staticmethod
    def optimize_missed_chanches(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy):
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
            available_robots_ids, _ = tick({}, robots, operating_threshold, charging_threshold)
            
            if move_computation_enabled:
                move_computation(available_robots_ids, robots, adjacency_matrix, policy)
        
        return res      

    @staticmethod
    def optimize_operation_time(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy):
        res = []

        for _ in range(time_instants):
            charging = 0
            operating = 0
            for r in robots:
                if r.get_status() == "charging":
                    charging += 1
                elif r.get_status() == "operating":
                    operating += 1 

            res.append(operating)
            available_robots_ids, _ = tick({}, robots, operating_threshold, charging_threshold)
            
            if move_computation_enabled:
                move_computation(available_robots_ids, robots, adjacency_matrix, policy)
        
        return res
    
    @staticmethod
    def optimize_operation_time_mpc_wrapper(x, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy):
        for r in robots:
            r.unhost()
            r.unoffload()
            
        res = [i for i in range(len(robots))]
        for id, r_alloc in enumerate(x.value):
            for id2, val in enumerate(r_alloc):
                if val == 1:
                    res[id2] = id
                    
        for i, id in enumerate(res):     
            if robots[id] != robots[i]:
                robots[i].offload(robots[id])
                robots[id].host(robots[i].get_self_task())
        
        return Allocator.optimize_operation_time(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy)
    

if __name__ == "__main__":
    # Example usage
    task_requirements = [20, 20]
    battery_levels = [80, 20]
    battery_status = ['operating', 'operating']
    discharge_rate = [5, 5]
    charge_rate = [5, 5]
    costrained_allocation = [-1, -1, 0, -1]
    current_allocation = [0, 1, 0, 3]
    time_instants = 5
    
    bf = Allocator(4, AllocationPolicy.MOVE1)
    a1 = bf.wrap(costrained_allocation, current_allocation)
    print(a1)
    bf.terminate()
    
    bf = Allocator(4, AllocationPolicy.MOVE2)
    a2 = bf.wrap(costrained_allocation, current_allocation)
    print(a2)
    bf.terminate()
    
    bf = Allocator(4, AllocationPolicy.MOVE3)
    a3 = bf.wrap(costrained_allocation, current_allocation)
    print(a3)
    bf.terminate()
    
    for a in a1:
        found = False
        for aa in a2:
            if a == aa:
                found = True
                break
        if not found:
            print("Error1")
            
    for a in a1:
        found = False
        for aa in a3:
            if a == aa:
                found = True
                break
        if not found:
            print("Error2")
            
    for a in a2:
        found = False
        for aa in a3:
            if a == aa:
                found = True
                break
        if not found:
            print("Error3")
            
    print("All good")
        
    # bf = Allocator(8, AllocationPolicy.MOVE2)
    # bf.print_powerset_count()
    
    # bf = Allocator(8, AllocationPolicy.MOVE3)
    # bf.print_powerset_count()
    # bf.print_powerset()
    #print(bf.find_best_allocation(task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, time_instants, costrained_allocation))
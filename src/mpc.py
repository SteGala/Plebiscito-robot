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

            # cost = Allocator.optimize_missed_chanches(rob, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants)
            cost = Allocator.optimize_operation_time(rob, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants, policy)
            cost = 1/np.sum(cost)
            
            # Push the result to the result_queue
            result_queue.put({"alloc": alloc, "cost": cost})

class Allocator:
    def __init__(self, n_robots, alloc_policy=AllocationPolicy.BRUTE_FORCE, n_processes=4, move_policy=MoveComputationPolicy.LARGEST_BATTERY):
        self.n_robots = n_robots
        self.allocation_policy = alloc_policy
        self.move_policy = move_policy
        self.alloc_options = None
                
        if alloc_policy is AllocationPolicy.BRUTE_FORCE:
            self.alloc_options = self.__custom_powerset()
            
        # shound be computed at every optimization request. Just check if the allocation policy is consistent
        elif alloc_policy is AllocationPolicy.MOVE1:
            pass
        elif alloc_policy is AllocationPolicy.MOVE2:
            pass
        elif alloc_policy is AllocationPolicy.MOVE3:
            pass
        elif alloc_policy is AllocationPolicy.MPC:
            pass
        else:
            print(f"Allocation policy {alloc_policy} not supported")
            sys.exit(1)

        if alloc_policy is not AllocationPolicy.MPC:
            self.process_pool = ProcessPool(n_processes, move_policy)

    def terminate(self):
        if self.allocation_policy is not AllocationPolicy.MPC:
            self.process_pool.terminate()
            
    def __move_n_powerset(self, n, constrained_allocation):
        res = []
        current = [i for i in range(self.n_robots)]
        
        count = 0
        for id, i in enumerate(constrained_allocation):
            if i != -1:
                current[id] = i
            if i != -1 and i != id:
                count += 1
                 
        self._rec_move_n_powerset(current, res, 0, n+count, constrained_allocation)
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
        
    def _rec_move_n_powerset(self, current, result, index, n, costrained_allocation):
        def is_allowed():
            for i in range(index):
                if costrained_allocation[i] != -1 and costrained_allocation[i] != current[i]:
                    return False
                
            ret = self.__validate_count(current, index+1)
            if ret is False:
                return False
            
            count = 0
            for i in range(self.n_robots):
                if current[i] != i:
                    count += 1
            if count > n:
                return False
            return True
        
        if index == self.n_robots:
            result.append(copy.deepcopy(current))
            return
        
        for i in range(self.n_robots):
            current[index] = i
            if is_allowed():   
                self._rec_move_n_powerset(current, result, index + 1, n, costrained_allocation)
            current[index] = index
    
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
    
    def find_best_allocation(self, time_instants, robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, costrained_allocation=None):
        best_solution = None

        if self.allocation_policy is AllocationPolicy.MPC:
            return self.__mpc(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants)
        
        if self.alloc_options is None:
            if self.allocation_policy is AllocationPolicy.MOVE1:
                self.alloc_options = self.__move_n_powerset(1, costrained_allocation)
            elif self.allocation_policy is AllocationPolicy.MOVE2:
                self.alloc_options = self.__move_n_powerset(2, costrained_allocation)
            elif self.allocation_policy is AllocationPolicy.MOVE3:
                self.alloc_options = self.__move_n_powerset(3, costrained_allocation)
        
        # for a in self.alloc_options:
        #     print(a)        
        # sys.exit(1)
        
        for alloc in self.alloc_options:
            if not self.__validate_with_constraints(alloc, costrained_allocation):
                continue
                        
            self.process_pool.submit(copy.deepcopy(robots), charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, alloc, time_instants)               
            
        best_solution = self.process_pool.get_best_result()
                
        if self.allocation_policy is not AllocationPolicy.BRUTE_FORCE:
            self.alloc_options = None
                
        return best_solution
    
    @staticmethod
    def optimize_missed_chanches(robots, charging_threshold, operating_threshold, move_computation_enabled, adjacency_matrix, time_instants):
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
            available_robots_ids, _ = tick({}, robots, operating_threshold, charging_threshold, False)
            
            if move_computation_enabled:
                move_computation(available_robots_ids, robots, adjacency_matrix)
        
        return np.sum(res)   

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
            available_robots_ids, _ = tick({}, robots, operating_threshold, charging_threshold, False)
            
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
    costrained_allocation = [-1, -1, 0, -1, -1, -1]
    time_instants = 5

    # bf = Allocator(8, AllocationPolicy.BRUTE_FORCE)
    # bf.print_powerset_count()
    
    bf = Allocator(6, AllocationPolicy.MOVE1)
    bf.find_best_allocation(10, {}, 0.15, 0.85, True, [], costrained_allocation)
    
    # bf = Allocator(8, AllocationPolicy.MOVE2)
    # bf.print_powerset_count()
    
    # bf = Allocator(8, AllocationPolicy.MOVE3)
    # bf.print_powerset_count()
    # bf.print_powerset()
    #print(bf.find_best_allocation(task_requirements, battery_levels, battery_status, discharge_rate, charge_rate, time_instants, costrained_allocation))
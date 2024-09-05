from enum import Enum
import numpy as np
import heapq
import random

class MoveComputationPolicy(Enum):
    NONE = 0
    LARGEST_BATTERY = 1
    SMALLEST_BATTERY = 2
    RANDOM = 3

def compute_adjacency_matrix(n_robots, probability):
    adjacency_matrix = np.zeros((n_robots, n_robots))
    for i in range(n_robots):
        for j in range(0, i):
            value = np.random.choice([0, 1], p=[1-probability, probability])
            adjacency_matrix[i][j] = value
            adjacency_matrix[j][i] = value 
            
    return adjacency_matrix

def dijkstra(adjacency_matrix, source):
    n_nodes = len(adjacency_matrix)
    distances = {node: float('inf') for node in range(n_nodes)}
    distances[source] = 0
    queue = [(0, source)]
    visited = set()

    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_node in visited:
            continue
        visited.add(current_node)

        for neighbor in range(n_nodes):
            if adjacency_matrix[current_node][neighbor] == 1:
                distance = current_distance + 1
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(queue, (distance, neighbor))

    distance_dict = {}
    for node, distance in distances.items():
        if distance not in distance_dict and distance != float('inf') and node != source:
            distance_dict[distance] = []
        if distance != float('inf') and node != source:
            distance_dict[distance].append(node)

    return distance_dict

if __name__ == "__main__":
    adjacency_matrix = compute_adjacency_matrix(10, 0.1)
    print(dijkstra(adjacency_matrix, 0))
    
def count_missed_offload(allocation, battery_level, status):
    missed_offload = 0
    for i, (alloc, battery, stat) in enumerate(zip(allocation, battery_level, status)):
        if alloc == 1 and battery <= 0 and stat == 'operating':
            missed_offload += 1
    return missed_offload

def tick(res, robots, operating_threshold, charging_threshold, delay_enabled):
    available_robots_ids = []
    target_for_operating = []
        
    # Iterate over each robot
    for id, robot in enumerate(robots):
        battery = robot.tick()
        if robot.name not in res:
            res[robot.name] = []
        res[robot.name].append(battery)
        r_status = robot.get_status()
            
        # If battery level is below charging threshold and the robot is not already charging, start charging
        if battery <= charging_threshold and r_status != "charging":
            robot.charge()
            available_robots_ids.append(id)    
        # If battery level is above operating threshold and the robot is not already operating, set it to operate
        elif battery >= operating_threshold and r_status != "operating":
            # Set the robot to operate
            #robot.operate()
            if len(target_for_operating) < 1 and delay_enabled:
                target_for_operating.append(id)
            else:
                robot.operate()
        else:
            # If the robot is not hosting a task and it is currently charging, add it to the available robots list
            if not robot.is_hosting() and r_status == "charging":
                available_robots_ids.append(id)
    
    return available_robots_ids, target_for_operating

def move_computation(available_robots_ids, robots, adjacency_matrix, policy):
    """
    Perform move computation for available robots.

    Args:
        available_robots_ids (list): List of available robot IDs.
    """
    for i in available_robots_ids:
        robot = robots[i]
        
        # Skip if the robot is charging or already hosting a task
        if robot.is_hosting():
            continue
            
        # Find the nearest robot that is operating
        distances = dijkstra(adjacency_matrix, i)
            
        found = False
        for _, ids in distances.items():
            if found:
                break
            
            sorted_ids = sort_ids(ids, robots, policy)
                
            for id in sorted_ids:
                if not robots[id].has_offloaded() and robots[id].get_status() == "operating":
                    robots[id].offload(robot)
                    assert robot.host(robots[id].get_self_task()) != False
                    found = True
                    break
                
def sort_ids(ids, robots, policy):
    if policy is MoveComputationPolicy.NONE:
        return ids
    
    if policy is MoveComputationPolicy.RANDOM:
        random.shuffle(ids)
        return ids
    
    sorted_ids = []
    if policy is MoveComputationPolicy.LARGEST_BATTERY or policy is MoveComputationPolicy.SMALLEST_BATTERY:
        for id in ids:
            if len(sorted_ids) == 0:
                sorted_ids.append(id)
            else:
                target = None
                for id2 in sorted_ids:
                    if robots[id].get_battery_percentage() <= robots[id2].get_battery_percentage():
                        target = id2
                        break
                if target is not None:
                    sorted_ids.insert(target, id)
                else:
                    sorted_ids.append(id)
                    
        if policy is MoveComputationPolicy.LARGEST_BATTERY:
            sorted_ids.reverse()
            
    return sorted_ids
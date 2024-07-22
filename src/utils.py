import numpy as np
import heapq

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
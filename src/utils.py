import numpy as np

def compute_adjacency_matrix(n_robots, probability):
    adjacency_matrix = np.zeros((n_robots, n_robots))
    for i in range(n_robots):
        for j in range(0, i):
            value = np.random.choice([0, 1], p=[1-probability, probability])
            adjacency_matrix[i][j] = value
            adjacency_matrix[j][i] = value 
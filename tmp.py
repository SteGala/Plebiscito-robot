import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# Define the number of robots and tasks
num_robots = 4
num_tasks = 6

# Prediction horizon
H = 3

# Randomly generate task costs for each robot
np.random.seed(0)
task_costs = np.random.randint(1, 10, (num_robots, num_tasks))

# State variable: x_ij is 1 if task j is assigned to robot i, else 0
x = cp.Variable((num_robots, num_tasks), boolean=True)

# Objective: Minimize total cost
objective = cp.Minimize(cp.sum(cp.multiply(task_costs, x)))

# Constraints
constraints = []

# Each task must be assigned to exactly one robot
for j in range(num_tasks):
    constraints.append(cp.sum(x[:, j]) == 1)

# Each robot can have at most two tasks
for i in range(num_robots):
    constraints.append(cp.sum(x[i, :]) <= 2)

# MPC loop for H steps
for step in range(H):
    # Define the optimization problem
    prob = cp.Problem(objective, constraints)
    
    # Solve the problem
    prob.solve()
    
    # Print the results
    print(f"Step {step+1} allocation:")
    print(np.round(x.value))

# Final MPC task allocation and its cost
final_allocation = np.round(x.value)
mpc_cost = np.sum(task_costs * final_allocation)

print("Final task allocation:")
print(final_allocation)
print(f"MPC Cost: {mpc_cost}")

# --- Generate Random Allocations and Costs ---

num_random_allocations = 100
random_costs = []

for _ in range(num_random_allocations):
    random_allocation = np.zeros((num_robots, num_tasks))
    for j in range(num_tasks):
        i = np.random.choice(num_robots)
        if np.sum(random_allocation[i]) < 2:
            random_allocation[i, j] = 1
        else:
            # Choose another robot if the selected one already has 2 tasks
            available_robots = [k for k in range(num_robots) if np.sum(random_allocation[k]) < 2]
            random_allocation[np.random.choice(available_robots), j] = 1
    
    # Compute the cost of the random allocation
    cost = np.sum(task_costs * random_allocation)
    random_costs.append(cost)

# --- Plot the Results ---
plt.figure(figsize=(10, 6))

# Plot boxplot of random allocations costs
plt.boxplot(random_costs, positions=[1], widths=0.6, patch_artist=True)

# Plot the MPC cost
plt.scatter([1], [mpc_cost], color='red', zorder=5, label='MPC Solution')

plt.xticks([1], ['Random Allocations'])
plt.ylabel('Total Cost')
plt.title('Cost Comparison: MPC vs Random Allocations')
plt.legend()
plt.savefig("tmp.png")

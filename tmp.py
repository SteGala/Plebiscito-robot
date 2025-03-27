import pulp as plp

# Parameters
N = 5  # Number of devices
T = 10  # Time horizon (discrete time steps)
discharge_rate = [5, 6, 4, 5, 7]  # Discharge rate (percentage per task per time step)
charge_rate = [10, 9, 11, 10, 8]  # Charge rate (percentage per time step)
initial_battery = [100, 100, 100, 100, 100]  # Initial battery levels (percentage)

# Create the PuLP problem
prob = plp.LpProblem("Maximize_Operating_Time", plp.LpMaximize)

# Variables
S = plp.LpVariable.dicts("S", [(i, t) for i in range(N) for t in range(T)], cat=plp.LpBinary)  # Operating state
B = plp.LpVariable.dicts("B", [(i, t) for i in range(N) for t in range(T)], lowBound=0, upBound=100)  # Battery level
x = plp.LpVariable.dicts("x", [(i, j, t) for i in range(N) for j in range(N) for t in range(T)], cat=plp.LpBinary)  # Task assignment
z = plp.LpVariable.dicts("z", [(i, t) for i in range(N) for t in range(T)], lowBound=0)  # Linearized variable for task execution

# Objective: Maximize the sum of operating time
prob += plp.lpSum([S[i, t] for i in range(N) for t in range(T)])

# Constraints
for t in range(T):
    for i in range(N):
        # Battery dynamics (discharge when operating)
        if t == 0:
            prob += B[i, t] == initial_battery[i]
        else:
            prob += B[i, t] == B[i, t-1] + charge_rate[i] * (1 - S[i, t]) - discharge_rate[i] * z[i, t]

        # Ensure battery stays within bounds
        prob += B[i, t] >= 0
        prob += B[i, t] <= 100

        # Ensure each task is executed exactly once
        prob += plp.lpSum([x[i, j, t] for j in range(N)]) == 1

        # Linearization of task execution based on operating state
        prob += z[i, t] <= S[i, t]
        prob += z[i, t] <= plp.lpSum([x[i, j, t] for j in range(N)])
        prob += z[i, t] >= S[i, t] + plp.lpSum([x[i, j, t] for j in range(N)]) - 1

    for j in range(N):
        # Ensure no device executes more than 2 tasks at a time
        prob += plp.lpSum([x[i, j, t] for i in range(N)]) <= 2

# Solve the optimization problem
prob.solve()

# Results
print("Status:", plp.LpStatus[prob.status])
print("Total Operating Time:", plp.value(prob.objective))

# Output task assignments and battery states
for t in range(T):
    print(f"\nTime Step {t}")
    for i in range(N):
        print(f"Device {i}: Operating={S[i,t].varValue}, Battery={B[i,t].varValue:.2f}")
        for j in range(N):
            if x[i,j,t].varValue == 1:
                print(f"  Task {i} executed by Device {j}")

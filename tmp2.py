import cvxpy as cp
import numpy as np

# Parametri del sistema
N = 10  # Numero di robot
T = 20  # Orizzonte temporale
B_max = 100  # Capacità massima della batteria
B_min = 10   # Soglia minima della batteria
charge_rate = 5  # Velocità di ricarica
consume_rate = 4  # Consumo per operazione

def mpc_optimization():
    # Variabili decisionali
    x = cp.Variable((N, T))  # Livello di batteria per ogni robot nel tempo
    u = cp.Variable((N, T), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
    o = cp.Variable((N, T), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
    w = cp.Variable((N, T))  # Variabile ausiliaria per eliminare il prodotto tra booleani
    
    constraints = []
    
    # Stato iniziale (vincolo invece di assegnazione diretta)
    x_init = np.random.randint(B_min, B_max, size=N)
    constraints.append(x[:, 0] == x_init)  # Vincolo per i livelli iniziali della batteria
    
    for t in range(T - 1):
        for i in range(N):
            # Definizione della variabile ausiliaria w[i, t] = u[i, t] * (1 - o[i, t])
            constraints.append(w[i, t] <= u[i, t])
            constraints.append(w[i, t] <= 1 - o[i, t])
            constraints.append(w[i, t] >= u[i, t] + (1 - o[i, t]) - 1)

            # Dinamica della batteria con variabile ausiliaria
            constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * w[i, t])
            
            # Vincoli di batteria
            constraints.append(x[i, t+1] >= B_min)
            constraints.append(x[i, t+1] <= B_max)
            
        # Bilanciamento 50%-50%
        constraints.append(cp.sum(u[:, t]) == N / 2)
    
    # Funzione obiettivo: massimizzare il tempo operativo e l'offloading
    objective = cp.Maximize(cp.sum(u) + cp.sum(o))
    
    # Risoluzione del problema
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.GUROBI if "GUROBI" in cp.installed_solvers() else cp.ECOS)
    
    return x.value, u.value, o.value, w.value

# Esegui l'ottimizzazione
battery_levels, operations, offloading, aux_var = mpc_optimization()

# Stampa risultati
print("Battery Levels:\n", battery_levels)
print("Operations:\n", operations)
print("Offloading:\n", offloading)
print("Auxiliary Variable:\n", aux_var)

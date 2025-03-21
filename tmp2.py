import cvxpy as cp
import numpy as np

# Parametri del sistema
N = 20  # Numero di robot
T = 20  # Orizzonte temporale
B_max = 100  # Capacità massima della batteria
B_min = 10   # Soglia minima della batteria
charge_rate = 1  # Velocità di ricarica
consume_rate = 1  # Consumo per operazione
computation_cost = 10  # Costo computazionale per il computing

def mpc_optimization():
    # Variabili decisionali
    x = cp.Variable((N, T))  # Livello di batteria per ogni robot nel tempo
    u = cp.Variable((N, T), boolean=True)  # 1 se il robot è operativo, 0 se è in carica
    o = cp.Variable((N, T), boolean=True)  # 1 se il robot sta offloadando, 0 altrimenti
    #w = cp.Variable((N, T))  # Variabile ausiliaria per eliminare il prodotto tra booleani
    
    constraints = []
    
    # Stato iniziale (vincolo invece di assegnazione diretta)
    x_init = np.random.randint(B_min, B_max, size=N)
    constraints.append(x[:, 0] == x_init)  # Vincolo per i livelli iniziali della batteria
    u_init = np.random.randint(0, 2, size=N)
    constraints.append(u[:, 0] == u_init)  # Vincolo per lo stato iniziale degli operatori
    
    for t in range(T - 1):
        for i in range(N):
            constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * u[i, t] - computation_cost * (u[i, t] - o[i, t]))
            constraints.append(x[i, t+1] >= 0)
            constraints.append(x[i, t+1] <= 100)
            
            constraints.append(o[i, t] <= u[i, t])
        
        constraints.append(cp.sum(o[:, t]) == N - cp.sum(u[:, t]))
            
            # constraints.append(u[i, t] <= (1/charge_rate) * (100 - x[i, t]))
            #if x[i, t+1] <= consume_rate:
            #constraints.append(u[i, t+1] ==  charge_rate * (i - u[i, t]) - consume_rate * u[i, t])
            
            # # Definizione della variabile ausiliaria w[i, t] = u[i, t] * (1 - o[i, t])
            # constraints.append(w[i, t] <= u[i, t])
            # constraints.append(w[i, t] <= 1 - o[i, t])
            # constraints.append(w[i, t] >= u[i, t] + (1 - o[i, t]) - 1)

            # # Dinamica della batteria con variabile ausiliaria
            # constraints.append(x[i, t+1] == x[i, t] + charge_rate * (1 - u[i, t]) - consume_rate * w[i, t])
            
            # # Vincoli di batteria
            # constraints.append(x[i, t+1] >= B_min)
            # constraints.append(x[i, t+1] <= B_max)
            
        # Bilanciamento 50%-50%
        #constraints.append(cp.sum(u[:, t]) == N / 2)
    
    # Funzione obiettivo: massimizzare il tempo operativo e l'offloading
    objective = cp.Maximize(cp.sum(u))
    
    # Risoluzione del problema
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.GUROBI if "GUROBI" in cp.installed_solvers() else cp.ECOS)
    
    if x.value is None:
        print(x_init)
        print(u_init)
    
    return x.value, u.value, o.value#, w.value

# Esegui l'ottimizzazione
battery_levels, operations, offloading = mpc_optimization()

if battery_levels is not None:
    # Stampa risultati
    for i in range(N):
        print("Robot", i)
        print("Battery Levels:", battery_levels[i])
        print("Operations:", operations[i])
        print("Offloading:", offloading[i])
        print()
    #print("Auxiliary Variable:\n", aux_var)

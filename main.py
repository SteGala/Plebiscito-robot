from src.mpc import AllocationPolicy
from src.utils import MoveComputationPolicy
from src.simulator import Simulator

# Configurations from Chatgpt

small_config = {
    "n_robots": 8,
    "charge_rate": 17, # 17Wh
    "discharge_rate": 15, # 15Wh 
    "total_battery": 35*60, # typically 35Wh then mutiply by 60 to get the total battery
    "AI_computation": 20
}

medium_config = {
    "n_robots": 15,
    "charge_rate": 65, # 65Wh
    "discharge_rate": 25, # 25Wh 
    #"discharge_rate": 0,
    "total_battery": 220*60, # typically 220Wh then mutiply by 60 to get the total battery
    "AI_computation": 20
}

# medium_config = {
#     "n_robots": 3,
#     "charge_rate": 65, # 65Wh
#     "discharge_rate": 25, # 25Wh 
#     #"discharge_rate": 0,
#     "total_battery": 300, # typically 220Wh then mutiply by 60 to get the total battery
#     "AI_computation": 20
# }

# Configurations from Gemini

config = {
    "n_robots": 10,
    "charge_rate": 150, # 65Wh
    "discharge_rate": 100, # 25Wh 
    "total_battery": 250*60, # typically 220Wh then mutiply by 60 to get the total battery
    "AI_computation": 20
}

# Note: every time instant represent one minute of simulation, therefore, the total_battery is multiplied by 60 to get the total battery

if __name__ == "__main__":
    n_run = 1
    duration = 1000
    
    for i in range(n_run):
        # Run the simulation to get the values for the battery optimmization
        s = Simulator(i, f"reference", config=medium_config, allocation_policy=AllocationPolicy.MOVE1)
        s.run(duration)
        
        s = Simulator(i, f"battery-l", config=medium_config, allocation_policy=AllocationPolicy.MOVE1, move_computation_policy=MoveComputationPolicy.LARGEST_BATTERY)
        s.run(duration)

        s = Simulator(i, "mpc", config=medium_config, allocation_policy=AllocationPolicy.MPC, move_computation_policy=MoveComputationPolicy.LARGEST_BATTERY, optimize_computation_frequency=1, optimize_computation_window=1000)
        s.run(duration)
        # for j in [1000]:
        #     s = Simulator(i, f"battery-opt", move_computation_enabled=True, optimize_computation_frequency=1, optimize_computation_window=j, config=medium_config, allocation_policy=AllocationPolicy.MOVE1, num_processes=22)
        #     s.run(duration)
        
       
        
        
        
        
                
    
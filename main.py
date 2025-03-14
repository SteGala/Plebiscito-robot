from src.mpc import AllocationPolicy
from src.utils import MoveComputationPolicy
from src.simulator import Simulator
from datetime import datetime
import os

# Configurations from Chatgpt

small_config = {
    "n_robots": 8,
    "charge_rate": 17, # 17Wh
    "discharge_rate": 15, # 15Wh 
    "total_battery": 35*60, # typically 35Wh then mutiply by 60 to get the total battery
    "AI_computation": 20,
    "charging_threshold": 0.05, 
    "operating_threshold": 0.95
}

medium_config = {
    "n_robots": 25,
    "charge_rate": 65, # 65Wh
    "discharge_rate": 25, # 25Wh 
    #"discharge_rate": 0,
    "total_battery": 220*60, # typically 220Wh then mutiply by 60 to get the total battery
    "AI_computation": 20,
    "charging_threshold": 0.05, 
    "operating_threshold": 0.95
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
    "AI_computation": 20,
    "charging_threshold": 0.05, 
    "operating_threshold": 0.95
}

# Note: every time instant represent one minute of simulation, therefore, the total_battery is multiplied by 60 to get the total battery

if __name__ == "__main__":
    duration = 1000
    n_run = 100    

    # Run the simulation to get the values for the battery optimmization
    s = Simulator(run_number=n_run, config=medium_config, move_computation_policies=[MoveComputationPolicy.LARGEST_BATTERY, MoveComputationPolicy.SMALLEST_BATTERY, MoveComputationPolicy.RANDOM], report_dir="results")
    s.run(duration)
            
        
       
        
        
        
        
                
    
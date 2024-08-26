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
    "n_robots": 7,
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
    "n_robots": 8,
    "charge_rate": 150, # 65Wh
    "discharge_rate": 100, # 25Wh 
    "total_battery": 250*60, # typically 220Wh then mutiply by 60 to get the total battery
    "AI_computation": 20
}

# Note: every time instant represent one minute of simulation, therefore, the total_battery is multiplied by 60 to get the total battery

if __name__ == "__main__":
    n_run = 1
    duration = 10000
    
    for i in range(n_run):
        # Run the simulation to get the values for the battery optimmization
        s = Simulator(i, f"reference", move_computation_enabled=False, config=medium_config)
        s.run(duration)
        
        s = Simulator(i, f"battery", move_computation_enabled=True, config=medium_config)
        s.run(duration)
        
        # s = Simulator(i, f"delay-1-9", move_computation_enabled=True, config=medium_config, delay_operation_enabled=True)
        # s.run(duration)
        
        for j in [100, 200, 300, 750]:
            s = Simulator(i, f"battery-opt-{str(j)}", move_computation_enabled=True, optimize_computation_frequency=j, optimize_computation_window=j, config=medium_config)
            s.run(duration)
        
       
        
        
        
        
                
    
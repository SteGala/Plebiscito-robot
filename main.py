import numpy as np
import pandas as pd
from src.mpc import AllocationPolicy, AllocationStrategy
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
    "n_robots": 15,
    "charge_rate": 20, # 65Wh
    "discharge_rate": 20, # 25Wh 
    #"discharge_rate": 0,
    "total_battery": 220*60, # typically 220Wh then mutiply by 60 to get the total battery
    "AI_computation": 120,
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

def compute_value(x, y, epochs, tot_battery, n_robot):
    offload = epochs*(tot_battery/y)/((tot_battery/y) + (tot_battery/x))
    return offload * n_robot

# Note: every time instant represent one minute of simulation, therefore, the total_battery is multiplied by 60 to get the total battery

if __name__ == "__main__":
    duration = 30000
    n_run = 1

    results = []

    charge = np.linspace(0.004, 0.016, 30)
    discharge = np.linspace(0.004, 0.016, 30)
    computation = np.linspace(0.006, 0.015, 4)

    for ch in charge:
        for dis in discharge:
            for comp in computation:
                medium_config["charge_rate"] = medium_config["total_battery"] * ch 
                medium_config["discharge_rate"] = medium_config["total_battery"] * dis
                medium_config["AI_computation"] = medium_config["total_battery"] * comp

                # Run the simulation to get the values for the battery optimmization
                s = Simulator(
                    run_number=n_run,
                    config=medium_config,
                    move_computation_policies=[MoveComputationPolicy.SMALLEST_BATTERY],
                )
                # s = Simulator(run_number=n_run, config=medium_config, move_computation_policies=[MoveComputationPolicy.LARGEST_BATTERY], report_dir="results")

                res = s.run(duration)
                d = {}
                d["wasted_computation_no_offload"] = res["wasted_computation_no_offload"]
                d["wasted_computation_offload"] = res["wasted_computation_offload"]
                d["charge_rate"] = ch
                d["discharge_rate"] = dis
                d["AI_computation"] = comp
                d["offload"] = res["offload"]
                d["no_offload"] = res["no_offload"]
                d["best_offload"] = compute_value(medium_config["charge_rate"], medium_config["discharge_rate"], duration, 11880, 15)

                results.append(d)

    pd.DataFrame(results).to_csv("results.csv", index=False)

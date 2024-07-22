from src.simulator import Simulator

if __name__ == "__main__":
    s = Simulator("reference", 8, move_computation_enabled=False, optimize_computation_frequency=None)
    s.run(500)
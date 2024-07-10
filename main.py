from src.simulator import Simulator

if __name__ == "__main__":
    s = Simulator("no-offload", 10, move_computation_enabled=False)
    s.run(200)
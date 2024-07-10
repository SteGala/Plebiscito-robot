class Task:
    def __init__(self, from_robot, consumption):
        self.from_robot = from_robot
        self.consumption = consumption
        
    def __str__(self) -> str:
        return f"{self.from_robot} ({self.consumption})"
    
    def get_consumption(self):
        return self.consumption
    
    def get_from(self):
        return self.from_robot

from src.task import Task

class Robot:
    def __init__(self, name, battery_level=50, total_battery=100, status="operating", charge_rate=5, disharge_rate=1):
        self.name = name
        self.battery_level = battery_level
        self.total_battery = total_battery
        self.status = status
        self.charge_rate = charge_rate
        self.discharge_rate = disharge_rate
        self.hosted_task = None
        self.self_task = Task(name, 1)
        self.self_task_offload = False
        
    def __str__(self) -> str:
        return f"{self.name} ({self.battery_level})"
        
    def host(self, task):
        if self.hosted_task is not None:
            return False
        self.hosted_task = task
        return True 
    
    def is_hosting(self):
        return self.hosted_task is not None
      
    def get_status(self):
        return self.status
    
    def charge(self):
        self.status = "charging"
        
    def operate(self):  
        self.status = "operating"
        self.hosted_task = None
        
    def offload(self):
        self.self_task_offload = True
        
    def unoffload(self):
        self.self_task_offload = False

    def tick(self):
        if self.status == "charging":
            # charge the battery
            self.battery_level += self.charge_rate
            
            # if the robot is hosting a task, consume the battery
            if self.hosted_task is not None:
                self.battery_level -= self.hosted_task.get_consumption()
                
            # check if the battery level is greater than the total battery
            if self.battery_level > self.total_battery:
                self.battery_level = self.total_battery
        elif self.status == "operating":
            # discharge the battery
            self.battery_level -= self.discharge_rate
            
            # if the robot is hosting its own task, consume the battery
            if not self.self_task_offload:
                self.battery_level -= self.self_task.get_consumption()
                
                
        return self.battery_level / self.total_battery 
        

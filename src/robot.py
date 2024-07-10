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
        self.self_task = Task(self, 1)
        self.self_task_offload = False
        
        self.initialize_stats()
        
    def __str__(self) -> str:
        return f"{self.name} ({self.battery_level})"
    
    def initialize_stats(self):
        self.stats = {}
        self.stats["offloaded_computing"] = 0 # done
        self.stats["operation_time"] = 0 # done
        self.stats["charging_time"] = 0 # done
        self.stats["n_charging"] = 0 # done
        self.stats["n_operating"] = 0 # done
        self.stats["n_offloaded"] = 0 # done
        self.stats["n_hosted"] = 0 # done
        
    def get_stats(self):
        return self.stats
        
    def host(self, task):
        if self.hosted_task is not None:
            return False
        self.hosted_task = task
        self.stats["n_hosted"] += 1
        return True 
    
    def get_hosted_task(self):
        return self.hosted_task
    
    def is_hosting(self):
        return self.hosted_task is not None
      
    def get_status(self):
        return self.status
    
    def charge(self):
        self.status = "charging"
        self.stats["n_charging"] += 1
        
    def operate(self):  
        self.status = "operating"
        self.hosted_task = None
        self.stats["n_operating"] += 1
        
    def offload(self):
        self.self_task_offload = True
        self.stats["n_offloaded"] += 1
        
    def get_self_task(self):
        return self.self_task
        
    def has_offloaded(self):
        return self.self_task_offload
        
    def unoffload(self):
        self.self_task_offload = False

    def tick(self):
        if self.status == "charging":
            self.stats["charging_time"] += 1
            # charge the battery
            self.battery_level += self.charge_rate
            
            # if the robot is hosting a task, consume the battery
            if self.hosted_task is not None:
                self.battery_level -= self.hosted_task.get_consumption()
                
            # check if the battery level is greater than the total battery
            if self.battery_level > self.total_battery:
                self.battery_level = self.total_battery
        elif self.status == "operating":
            self.stats["operation_time"] += 1
            # discharge the battery
            self.battery_level -= self.discharge_rate
            
            # if the robot is hosting its own task, consume the battery
            if not self.self_task_offload:
                self.battery_level -= self.self_task.get_consumption()
            else:
                self.stats["offloaded_computing"] += self.self_task.get_consumption()
                
        return self.battery_level / self.total_battery 
        

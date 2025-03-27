from src.task import Task

class Robot:
    def __init__(self, name, battery_level=50, total_battery=100, status="operating", charge_rate=5, disharge_rate=1, task_demand=1):
        self.name = int(name)
        self.battery_level = battery_level
        self.total_battery = total_battery
        self.status = status
        self.charge_rate = charge_rate
        self.discharge_rate = disharge_rate
        self.hosted_task = None
        self.self_task = Task(self, task_demand)
        self.self_task.assign_to(self)
        
        self.initialize_stats()
        
    def __str__(self) -> str:
        return f"{self.name} ({self.battery_level})"
    
    def __eq__(self, value: object) -> bool:
        return self.name == value.name
    
    def get_name(self):
        return self.name
    
    def get_battery_level(self):
        return self.battery_level
    
    def get_battery_percentage(self):
        return self.battery_level / self.total_battery
    
    def get_total_battery(self):
        return self.total_battery
    
    def get_battery_status(self):
        return self.status
    
    def get_discharge_rate(self):
        return self.discharge_rate
    
    def get_discharge_rate_percentage(self):
        return self.discharge_rate / self.total_battery
    
    def get_charge_rate_percentage(self):
        return self.charge_rate / self.total_battery
    
    def get_charge_rate(self):
        return self.charge_rate
    
    def initialize_stats(self):
        self.stats = {}
        self.stats["operation_time"] = 0 # done
        self.stats["charging_time"] = 0 # done
        self.stats["n_charging"] = 0 # done
        self.stats["n_operating"] = 0 # done
        self.stats["n_offloaded"] = 0 # done
        self.stats["n_hosted"] = 0 # done
        self.stats["computation"] = 0 # done
        self.stats["free_computing"] = 0 # done
        self.stats["self_computing"] = 0 # done
        self.stats["offload_computing"] = 0 # done
        
    def update_computation(self):
        if self.hosted_task is not None:
            self.stats["computation"] += 1
            
        if not self.has_offloaded():
            self.stats["computation"] += 1
            
    def get_stats(self):
        return self.stats
        
    def host(self, task):
        if self.hosted_task is not None:
            return False
        self.hosted_task = task
        self.stats["n_hosted"] += 1
        return True 
    
    def unhost(self):
        self.hosted_task = None
    
    def get_hosted_task(self):
        return self.hosted_task
    
    def is_hosting(self):
        return self.hosted_task is not None
      
    def get_status(self):
        return self.status
    
    def charge(self):
        self.status = "charging"
        
        # if the robot task was offloaded, unoffload it
        if self.self_task.get_to() != self:
            self.self_task.get_to().unhost()
            self.self_task.assign_to(self)
            
        # if self.hosted_task is not None:
        #     self.hosted_task.get_from().unoffload()
        #     self.hosted_task = None
        
        self.stats["n_charging"] += 1
        
    def operate(self):  
        self.status = "operating"
        
        # if the robot is hosting a task, unoffload it
        if self.hosted_task is not None:
            self.hosted_task.get_from().unoffload()
            self.hosted_task = None
            
        self.stats["n_operating"] += 1
        
    def offload(self, robot):
        self.self_task.assign_to(robot)
        self.stats["n_offloaded"] += 1
        
    def get_self_task(self):
        return self.self_task
        
    def has_offloaded(self):
        return self.self_task.get_to() != self
        
    def unoffload(self):
        self.self_task.assign_to(self)

    def tick(self):
        if self.status == "charging":
            self.stats["charging_time"] += 1
            # charge the battery
            self.battery_level += self.charge_rate
            
            # if the robot is hosting a task, consume the battery
            if self.hosted_task is not None:
                self.stats["free_computing"] += self.hosted_task.get_consumption()
                # self.battery_level -= self.hosted_task.get_consumption()
                                
            # TODO: might be removed in future. If the device is charging we can assume that the self task is not executed
            if not self.has_offloaded():
                self.stats["self_computing"] += self.self_task.get_consumption()
                
            # check if the battery level is greater than the total battery
            if self.battery_level > self.total_battery:
                self.battery_level = self.total_battery
        elif self.status == "operating":
            self.stats["operation_time"] += 1
            # discharge the battery
            self.battery_level -= self.discharge_rate
            
            # if the robot is hosting its own task, consume the battery
            if not self.has_offloaded():
                self.battery_level -= self.self_task.get_consumption()
                self.stats["self_computing"] += self.self_task.get_consumption()
                
            if self.hosted_task is not None:
                self.battery_level -= self.hosted_task.get_consumption()
                self.stats["offload_computing"] += self.hosted_task.get_consumption()
                
        return self.battery_level / self.total_battery 
        

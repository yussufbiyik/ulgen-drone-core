import numpy as np

class SafetyManager:
    def __init__(self, collision_distance, controllers):
        self.collision_distance, self.controllers = collision_distance, controllers
    
    def check_collisions(self):
        """Çarpışma kontrolü"""
        for i, controller1 in enumerate(self.controllers):
            for j, controller2 in enumerate(self.controllers[i+1:], start=i+1):
                pos1 = self.controllers[i].position
                pos2 = self.controllers[j].position
                distance = np.linalg.norm(pos1 - pos2)
                
                if distance < self.collision_distance:
                    collision_info = {
                        'drone1': f"Dron ${controller1.id}",
                        'drone2': f"Dron ${controller2.id}",
                        'distance': distance
                    }
                    return True, collision_info
        return False, None
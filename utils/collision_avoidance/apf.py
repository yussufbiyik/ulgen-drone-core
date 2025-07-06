import numpy as np

class APF:
    def __init__(self, 
                    repulsive_gain = 0.8, 
                    repulsive_range = 2.0
                ):
        self.repulsive_gain, self.repulsive_range = repulsive_gain, repulsive_range
    
    def calculate(self, current_pos, neighbors):
        """
        APF tabanlı çarpışma önleme kuvveti
        """
        repulsive_force = np.zeros(3)
        
        for neighbor_pos in neighbors:
            distance_vector = current_pos - neighbor_pos
            distance = np.linalg.norm(distance_vector)
            
            if 0 < distance < self.repulsive_range:
                force_direction = distance_vector / distance
                force_magnitude = self.repulsive_gain * (1.0/distance - 1.0/self.repulsive_range)
                individual_force = force_magnitude * force_direction
            # Çok yakın mesafe acil durumu
            if distance < self.collision_distance:
                individual_force *= 3.0  # Üç kat daha güçlü itme
        
            repulsive_force += individual_force
            # Z ekseninde itme kuvvetini sınırlama
            repulsive_force[2] = np.clip(repulsive_force[2], -1.0, 1.0)
        return repulsive_force
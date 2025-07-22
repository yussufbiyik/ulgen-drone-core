import logging

class APF:
    def __init__(self, repulsive_gain=0.0005, influence_radius=0.0002):
        """
        :param repulsive_gain: Strength of repulsive force
        :param influence_radius: Max lat/lon distance to consider neighbors (degrees)
        """
        self.repulsive_gain = repulsive_gain
        self.influence_radius_sq = influence_radius ** 2

    def compute_apf(self, current_position, neighbors):
        """
        Computes APF velocity in NED frame from lat/lon.
        :param current_position: object with `.lat`, `.lon`
        :param neighbors: list of objects with `.lat`, `.lon`
        :return: (vx, vy) in m/s
        """
        if not neighbors:
            logging.debug("APF disabled — no neighbors.")
            return 0.0, 0.0

        force_x = 0.0
        force_y = 0.0

        for neighbor in neighbors:
            dx = current_position.lat - neighbor.lat
            dy = current_position.lon - neighbor.lon
            dist_sq = dx ** 2 + dy ** 2

            if dist_sq < 1e-10 or dist_sq > self.influence_radius_sq:
                continue  # çok yakın veya çok uzakta ise görmezden gel

            fx = self.repulsive_gain * dx / dist_sq
            fy = self.repulsive_gain * dy / dist_sq

            force_x += fx
            force_y += fy

        # dereceden metreye dönüştür (yaklaşık 111,139 metre/derece)
        vx = force_x * 1e5
        vy = force_y * 1e5

        return vx, vy

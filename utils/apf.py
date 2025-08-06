import logging
from utils.formation_utilities import distance_meters, latlon_to_ned

class APF:
    def __init__(self, repulsive_gain=0.000005, influence_radius=1.0, weight=0.1):
        """
        :param repulsive_gain: İtme kuvveti katsayısı
        :param influence_radius: Komşuları dikkate almak için maksimum mesafe (metre)
        """
        self.repulsive_gain = repulsive_gain
        self.influence_radius = influence_radius

    def compute_apf(self, current_position, neighbors):
        """
        APF hesaplama fonksiyonu
        :param current_position: Dronun anlık konumu
        :param neighbors: Komşu dronların anlık konumlarının bir listesi
        :return: (vx, vy) in m/s
        """
        if not neighbors:
            logging.debug("APF devre dışı, komşu yok.")
            return 0.0, 0.0

        force_x = 0.0
        force_y = 0.0

        for neighbor in neighbors:
            neighbor_position = neighbor["data"]["gps_position"]
            dx, dy = latlon_to_ned(
                neighbor_position["latitude"],
                neighbor_position["longitude"],
                current_position["latitude"],
                current_position["longitude"],
            )
            distance = distance_meters(
                current_position["latitude"],
                current_position["longitude"],
                neighbor_position["latitude"],
                neighbor_position["longitude"],
            )

            if distance > self.influence_radius:
                logging.debug(f"Komşu {distance} metre uzakta, itme kuvveti hesaplanmıyor.")
                continue  # çok uzakta ise görmezden gel
            logging.debug(f"Komşu {distance} metre mesafede, itme kuvveti hesaplanıyor.")

            fx = self.repulsive_gain * dx / distance
            fy = self.repulsive_gain * dy / distance

            force_x += fx
            force_y += fy

        # dereceden metreye dönüştür (yaklaşık 111,139 metre/derece)
        vx = force_x * 1e5
        vy = force_y * 1e5

        return vx, vy
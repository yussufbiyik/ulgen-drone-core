import sys 
import asyncio
import numpy as np
import functools
import logging

from core.drone import Drone

from utils.formation_utilities import distance_meters, calculate_formation_weight_center, calculate_ideal_formation_positions, assign_position

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

def check_neighbors(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if not self.drone.neighbors or len(self.drone.neighbors) < 2:
            raise Exception("Başka dron yok, formasyon özelliği çalıştırılamaz.")
        return await func(self, *args, **kwargs)
    return wrapper

class DroneController:
    def __init__(self, drone: Drone):
        self.drone = drone
        self.time_waited_for_other_drones = 0

    # Sık kullanılan drone işlemleri
    async def wait_for_proper_data(self):
        """
        Drone'un geçerli verileri almasını bekler.
        """
        while True:
            general_info = await self.drone.mavsdk_controller.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                self.drone.pre_takeoff_location = gps_position
                logging.info(f"Geçerli veriler alındı.")
                break
            logging.info("Geçerli veriler henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)

    async def arm(self):
        """
        Drone'u arm eder
        """
        await self.drone.mavsdk_controller.mavsdk.action.arm()
    async def arm_check(self):
        """
        Drone'un arm durumunu kontrol eder.
        """
        is_armed = await self.drone.mavsdk_controller.mavsdk.telemetry.armed().__anext__()
        return is_armed
    
    async def wait_for_broadcast(self):
        """
        Drone'un diğer dronların broadcast mesajlarını beklediği adım fonksiyonu.
        """
        logging.info("Diğer dronların broadcast mesajları bekleniyor...")
    async def wait_for_broadcast_check(self, minimum_neighbor_count=1):
        """
        Drone'un diğer dronların broadcast mesajlarını alıp almadığını kontrol eden fonksiyon.
        """
        if len(self.drone.neighbors) > 0:
            logging.info(f"Şu anda {len(self.drone.neighbors)} tane komşu drone var.")
            logging.info("Daha başka dronların olma ihtimaline karşın biraz daha bekleniyor.")
            if self.time_waited_for_other_drones < 2:
                self.time_waited_for_other_drones += 1
                await asyncio.sleep(1)
                return False
            logging.info("Tüm dronların broadcast mesajları alındığı varsayılıyor, kontrol tamamlandı.")
            return True if len(self.drone.neighbors) >= minimum_neighbor_count else False
        return False
    
    async def set_pre_takeoff_location(self):
        """
        Drone'un kalkış öncesi konumunu belirler.
        Bu, kalkış yüksekliğini hesaplamak için kullanılır.
        """
        while True:
            general_info = await self.drone.mavsdk_controller.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                self.drone.pre_takeoff_location = gps_position
                logging.debug(f"Kalkış öncesi konum ayarlandı: {self.drone.pre_takeoff_location}")
                break
            logging.warning("GPS konum bilgisi henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)
    async def pre_takeoff_location_check(self):
        """
        Drone'un kalkış öncesi konum kontrol fonksiyonu.
        Bu, drone'un kalkış yapmadan önceki konumunu kontrol eder.
        """
        if self.drone.pre_takeoff_location is not None:
            logging.debug(f"Kalkış öncesi konum belirlenmesi başarılı: {self.drone.pre_takeoff_location}")
            return True
        logging.warning("Kalkış öncesi konum henüz ayarlanmamış.")
        return False
    
    async def takeoff(self, altitude): 
        """
        Drone'a kalkış komutu gönderir
        
        :param altitude: Kalkış yüksekliği
        """
        await self.drone.mavsdk_controller.mavsdk.action.set_takeoff_altitude(altitude)
        await self.drone.mavsdk_controller.mavsdk.action.takeoff()
    async def altitude_check(self, target_altitude):
        """
        Drone'un irtifasını kontrol eden fonksiyon.
        """
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        climbed = abs(gps_position["altitude"] - self.drone.pre_takeoff_location["altitude"])
        logging.debug(f"Drone hedef irtifa ile {climbed} metre mesafede.")
        if abs(target_altitude - climbed) <= 0.2:
            logging.info(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
            return True
        return False

    async def enable_offboard_controller(self):
        logging.info("OffboardController aktifleştiriliyor...")
        self.drone.offboard_status["is_active"] = True
        asyncio.create_task(
            self.drone.offboard_controller.background_offboard_controller()
        )
    async def enable_offboard_controller_check(self):
        if await self.drone.mavsdk_controller.mavsdk.offboard.is_active():
            logging.info("OffboardController etkin.")
            return True
        return False
    
    async def goto_location(self, target_location):
        """
        Drone'u belirli bir konuma götüren adım fonksiyonu.
        
        :param target_location: Hedef konum (latitude, longitude, altitude)
        """
        logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
        await self.drone.mavsdk_controller.mavsdk.action.goto_location(
            target_location["latitude"],
            target_location["longitude"],
            target_location["altitude"]+self.drone.pre_takeoff_location["altitude"],  # GPS yüksekliğine göre ayarlanır
            0,  # yaw
        )
    async def goto_location_with_offboard(self, target_location):
        logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
        self.drone.offboard_status["target_position"] = target_location
        self.drone.offboard_status["altitude_to_keep"] = target_location["altitude"]
    async def goto_location_check(self, target_location):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        logging.debug(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
        # Hedefe ulaşma durumunda True döndür
        if (distance_meters(gps_position, target_location) <= self.drone.waypoint_threshold):
            logging.info("Drone hedef konuma ulaştı.")
            return True
        return False
    
    async def land(self): 
        """
        Drone'a iniş komutu gönderir
        """
        logging.info("Drone iniş yapıyor...")
        self.drone.offboard_status["is_active"] = False
        await self.drone.mavsdk_controller.mavsdk.action.land()

    async def disarm_pre_check(self):
        """
        Drone'un disarm edilmeden önceki durumunu kontrol eder.
        Şartın sağlanması için drone'un havada olmaması gerekir.
        """
        is_in_air = await self.drone.mavsdk_controller.mavsdk.telemetry.in_air().__anext__()
        return not is_in_air
    async def disarm(self):
        """
        Drone'u disarm eder
        """
        await self.drone.mavsdk_controller.mavsdk.action.disarm()
    async def disarm_check(self):
        """
        Drone'un disarm durumunu kontrol eder.
        """
        is_armed = await self.drone.mavsdk_controller.mavsdk.telemetry.armed().__anext__()
        return not is_armed
    
    # Formasyon ile alakalı fonksiyonlar
    async def neighbor_altitude_check(self, target_altitude):
        """
        Diğer dronların irtifalarını kontrol eder.
        """
        if len(self.drone.neighbors) < 2:
            logging.warning("Formasyon için yeterli komşu drone yok.")
            return False
        for neighbor in self.drone.neighbors:
            if "altitude" in neighbor["data"]["gps_position"] and (neighbor["data"]["gps_position"]["altitude"] - target_altitude) > 0.5:
                return False
        logging.info("Tüm komşu dronlar yeterli irtifaya sahip.")
        return True
    async def neighbor_formation_check(self):
        """
        Diğer dronların formasyon durumunu kontrol eder.
        """
        if len(self.drone.neighbors) < 2:
            logging.warning("Formasyon için yeterli komşu drone yok.")
            return True
        for neighbor in self.drone.neighbors:
            if distance_meters(neighbor["data"]["gps_position"], self.drone.neighbor_formation_positions[neighbor["sender"]]) > self.drone.waypoint_threshold:
                logging.warning(f"{neighbor['sender']} drone formasyon konumunda değil.")
                return False
        logging.info("Tüm komşu dronlar formasyon konumunda.")
        return True
    async def get_drone_formation_position(self, formation_type, formation_distance, gps_position):
        """
        Drone'u formasyon konumuna taşır.
        """
        if len(self.drone.neighbors) < 2:
            logging.warning("Formasyon için yeterli komşu drone yok.")
            return
        # Drone'un ideal pozisyonunu belirle
        center_position = calculate_formation_weight_center(gps_position, self.drone.neighbors)
        ideal_positions = calculate_ideal_formation_positions(formation_type, center_position, formation_distance)

        assigned_position, position_assignments = assign_position(ideal_positions, gps_position, self.drone.xbee_id, self.drone.neighbors)
        position_assignments.pop(self.drone.xbee_id)  # Kendi pozisyonunu kaldır
        self.drone.neighbor_formation_positions = position_assignments
        return assigned_position
    
    async def goto_formation_location_with_offboard(self, formation_type, formation_distance):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        target_location = await self.get_drone_formation_position(formation_type, formation_distance, gps_position)
        self.drone.formation_position = target_location
        self.drone.offboard_status["target_position"] = target_location
        self.drone.offboard_status["altitude_to_keep"] = gps_position["altitude"]
    async def goto_formation_location_check(self):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        logging.debug(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
        # Hedefe ulaşma durumunda True döndür
        if (distance_meters(gps_position, self.drone.formation_position) <= self.drone.waypoint_threshold) and await self.neighbor_formation_check():
            logging.info("Drone hedef konuma ulaştı.")
            return True
        return False
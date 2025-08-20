import sys 
import asyncio
import math
import random
import functools
import logging

from core.drone import Drone

from utils.formation_utilities import distance_meters, calculate_formation_weight_center, calculate_ideal_formation_positions, assign_position, latlon_to_ned, ned_to_latlon, rotate_position

from mavsdk.action import OrbitYawBehavior

from mavsdk.offboard import OffboardError, VelocityNedYaw, VelocityBodyYawspeed

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

def check_neighbors(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if not self.drone.neighbors or len(self.drone.neighbors) < 2:
            raise Exception("Başka dron yok, formasyon özelliği çalıştırılamaz.")
        return await func(self, *args, **kwargs)
    return wrapper

def normalize_angle_deg(angle):
    """Normalize angle to [0, 360) degrees."""
    return angle % 360
    
class DroneController:
    def __init__(self, drone: Drone):
        self.drone = drone
        self.time_waited_for_other_drones = 0
        self.start_angle = 0.0  # Başlangıç açısı, formasyon döndürme işlemlerinde kullanılacak

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
            logging.warning("Geçerli veriler henüz alınamadı, bekleniyor...")
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
    async def wait_for_broadcast_check(self, minimum_neighbor_count=1, try_count=2):
        """
        Drone'un diğer dronların broadcast mesajlarını alıp almadığını kontrol eden fonksiyon.
        """
        if len(self.drone.neighbors) >= minimum_neighbor_count:
            logging.info(f"Şu anda {len(self.drone.neighbors)} tane komşu drone var.")
            logging.info("Daha başka dronların olma ihtimaline karşın biraz daha bekleniyor.")
            if self.time_waited_for_other_drones < try_count:
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
        await self.drone.mavsdk_controller.mavsdk.action.takeoff()
        await self.drone.mavsdk_controller.mavsdk.action.set_takeoff_altitude(altitude)
    async def altitude_check(self, target_altitude):
        """
        Drone'un irtifasını kontrol eden fonksiyon.
        """
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        raw_altitude = await self.drone.mavsdk_controller.get_altitude()
        climbed = abs(gps_position["altitude"] - self.drone.pre_takeoff_location["altitude"])
        climbed_fallback = abs(raw_altitude.altitude_relative_m - self.drone.pre_takeoff_location["altitude"])
        logging.debug(f"Drone hedef irtifa ile {climbed} metre mesafede.")
        if (abs(target_altitude - climbed) <= 0.5) or (abs(target_altitude - climbed_fallback) <= 0.5):
            logging.info(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
            return True
        return False

    async def enable_offboard_controller(self):
        logging.info("OffboardController aktifleştiriliyor...")
        self.drone.offboard_status["is_active"] = True
        if not await self.drone.mavsdk_controller.mavsdk.offboard.is_active():
            try:
                yaw = await self.drone.mavsdk_controller.get_yaw()
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw)
                )
                await self.drone.mavsdk_controller.mavsdk.offboard.start()
                logging.debug("Offboard modu başlatıldı.")
            except OffboardError as e:
                logging.warning(f"Offboard moduna geçiş başarısız: {e}")
        asyncio.create_task(
            self.drone.offboard_controller.background_offboard_controller()
        )
    async def enable_offboard_controller_check(self):
        if await self.drone.mavsdk_controller.mavsdk.offboard.is_active():
            logging.info("OffboardController etkin.")
            return True
        return False

    async def disable_offboard_controller(self):
        logging.info("OffboardController devre dışı bırakılıyor...")
        self.drone.offboard_status["is_active"] = False
        await self.drone.mavsdk_controller.mavsdk.offboard.stop()
    async def disable_offboard_controller_check(self):
        if not await self.drone.mavsdk_controller.mavsdk.offboard.is_active() and not self.drone.offboard_status["is_active"]:
            logging.info("OffboardController devre dışı.")
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
        await self.drone.mavsdk_controller.mavsdk.action.set_current_speed(self.drone.speed_limit)
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
            self.drone.offboard_status["target_position"] = None
            return True
        logging.info(f"Drone hedef konuma ulaşamadı, mesafe: {distance_meters(gps_position, target_location)} metre.")
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
    async def neighbor_formation_check(self):
        """
        Diğer dronların formasyon durumunu kontrol eder.
        """
        if len(self.drone.neighbors) < 2:
            logging.warning("Formasyon için yeterli komşu drone yok.")
            return True
        for neighbor in self.drone.neighbors:
            neighbor_distance_to_formation = distance_meters(neighbor["data"]["gps_position"], self.drone.formation["neighbor_positions"][neighbor["sender"]])
            if neighbor_distance_to_formation > self.drone.waypoint_threshold:
                logging.info(f"{neighbor['sender']} drone formasyon konumuna {neighbor_distance_to_formation} metre uzaklıkta.")
                return False
        logging.info("Tüm komşu dronlar formasyon konumunda.")
        return True
    
    async def get_drone_formation_position(self, formation_type, formation_distance, gps_position, center_position=None):
        """
        Drone'u formasyon konumuna taşır.
        """
        if len(self.drone.neighbors) < 2:
            logging.warning("Formasyon için yeterli komşu drone yok.")
            return
        # Drone'un ideal pozisyonunu belirle
        center_position = center_position or calculate_formation_weight_center(gps_position, self.drone.neighbors)
        ideal_positions = calculate_ideal_formation_positions(formation_type, center_position, formation_distance)

        assigned_position, position_assignments = assign_position(ideal_positions, gps_position, self.drone.xbee_id, self.drone.neighbors)
        position_assignments.pop(self.drone.xbee_id)  # Kendi pozisyonunu kaldır
        self.drone.formation["neighbor_positions"] = position_assignments
        return assigned_position, position_assignments

    async def resolve_position_conflicts(self, position_assignments, target_location):
        loop_count = 0
        # Mevcut hedef pozisyonu diğer dronlara yayınla
        position_string = f"mt,{target_location['latitude']:.6f},{target_location['longitude']:.6f}".replace('.', '')
        self.drone.broadcast_message(position_string)
        # ACK ile hedef konumu gönder
        await self.drone.send_message_with_ack(position_string)
        while True:
            # Fazla döngüden kaçınmak için güvenlik kontrolü
            if loop_count > 2:
                logging.warning("Çok fazla döngüde kaldı, formasyon kabul edildi.")
                await self.drone.send_message_with_ack("mts,1")
                # Tüm komşu dronlar hedefi kabul etti mi kontrol et
                did_others_complete = all(
                    neighbor["data"].get("target_status", True)
                    for neighbor in self.drone.neighbors
                )
                if did_others_complete:
                    logging.info("Tüm komşu dronlar formasyon konumlarını kabul etti.")
                    return target_location
            # Komşuların hedef pozisyonlarını topla
            neighbor_target_positions = [
                {"sender": neighbor["sender"], "target": neighbor["data"]["target_position"], "current_position": neighbor["data"]["gps_position"]}
                for neighbor in self.drone.neighbors
                if "target_position" in neighbor["data"]
            ]
            # Çakışan pozisyonları bul (1 metre içinde)
            conflicting_positions = [
                neighbor
                for neighbor in neighbor_target_positions
                if round(distance_meters(neighbor["target"], target_location), 3) < 1.000
            ]
            # Çakışma yoksa devam et
            if (len(neighbor_target_positions) >= 2 and not conflicting_positions) or len(conflicting_positions) == 0:
                logging.info("Formasyon konumu sorunsuz.")
                # return
            else:
                # Çakışma varsa deterministik olarak önce alacağı yol en kısa olan dronaa,
                # mesafeler eşit ise en küçük ID'li drona öncelik ver
                conflicting_positions.sort(key=lambda n: int(str(n["sender"]), 16))
                lowest_id_drone = conflicting_positions[0]
                lowest_id = lowest_id_drone["sender"]
                # Mesafe bazlı ilk kararı ver
                general_info = await self.drone.mavsdk_controller.get_general_info()
                gps_position = general_info["gps_position"]
                if round(distance_meters(lowest_id_drone["target"], lowest_id_drone["current_position"]), 3) < round(distance_meters(self.drone.formation["position"], gps_position), 3):
                    logging.info(f"Drone {lowest_id} pozisyona daha yakın, öncelik ona ait.")
                    swap_position = position_assignments[lowest_id]
                    if swap_position:
                        target_location = {
                            "latitude": swap_position["latitude"],
                            "longitude": swap_position["longitude"]
                        }
                    else:
                        logging.warning("Tüm komşu dronlar formasyon konumunu işgal ediyor, bekleniyor...")
                # Mesafeler eşit ise en küçük ID'li drona öncelik ver
                if int(str(self.drone.xbee_id), 16) > int(str(lowest_id), 16):
                    # Daha büyük ID'ye sahip olan dron konumunu değiştirsin
                    swap_position = position_assignments[lowest_id]
                    logging.info(f"Değişim pozisyonu: {swap_position}")
                    if swap_position:
                        target_location = {
                            "latitude": swap_position["latitude"],
                            "longitude": swap_position["longitude"]
                        }
                    else:
                        logging.warning("Tüm komşu dronlar formasyon konumunu işgal ediyor, bekleniyor...")
                else:
                    logging.info(f"Dron {self.drone.xbee_id} çakışmada öncelikli, konumunu koruyor.")
            loop_count += 1
            await asyncio.sleep(1.5 + random.uniform(0, 0.5))

    async def goto_formation_location_with_offboard(self, formation_type, formation_distance):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        target_location, position_assignments = await self.get_drone_formation_position(formation_type, formation_distance, gps_position)
        self.drone.formation["position"] = target_location
        self.drone.formation["position"] = await self.resolve_position_conflicts(position_assignments, target_location)
        self.drone.offboard_status["target_position"] = target_location
        self.drone.offboard_status["altitude_to_keep"] = self.drone.pre_takeoff_location["altitude"] + self.drone.altitude_target

    async def goto_formation_location(self, formation_type, formation_distance):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        target_location, position_assignments = await self.get_drone_formation_position(formation_type, formation_distance, gps_position)
        self.drone.formation["position"] = target_location
        self.drone.formation["position"] = await self.resolve_position_conflicts(position_assignments, target_location)
        # Ardından hedef konuma ilerle
        await self.drone.mavsdk_controller.mavsdk.action.goto_location(
            target_location["latitude"],
            target_location["longitude"],
            self.drone.pre_takeoff_location["altitude"] + self.drone.altitude_target,
            0,  # yaw
        )
        await self.drone.mavsdk_controller.mavsdk.action.set_current_speed(self.drone.speed_limit)

    async def goto_formation_location_check(self):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        distance_to_target = distance_meters(gps_position, self.drone.formation["position"])
        # Hedefe ulaşma durumunda True döndür
        if (distance_to_target <= self.drone.waypoint_threshold):
            logging.info(f"Drone hedef konuma ulaştı, anlık konum: {gps_position}")
            return True
        return False
    
    # Formasyon ile navigasyon ile alakalı fonksiyonlar
    async def goto_location_with_formation(self, target_location, isOffboard=True):
        """
        Drone'u sürüyü de dikkate alarak belirli bir konuma götüren adım fonksiyonu.

        :param target_location: Hedef konum (latitude, longitude, altitude)
        """
        logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        center_position = calculate_formation_weight_center(gps_position, self.drone.neighbors)
        self.drone.formation["weight_center"] = center_position
        self_offset_north, self_offset_east = latlon_to_ned(gps_position, center_position)
        target_lat_offset, target_lon_offset = ned_to_latlon(
            self_offset_north, self_offset_east,
            target_location["latitude"],
            target_location["longitude"]
        )
        drone_formation_position_at_target = {
            "latitude": target_lat_offset,
            "longitude": target_lon_offset,
            "altitude": gps_position["altitude"]  # GPS yüksekliğine göre ayarlanır
        }
        self.drone.formation["position"] = drone_formation_position_at_target
        if not isOffboard:
            await self.drone.mavsdk_controller.mavsdk.action.goto_location(
                drone_formation_position_at_target["latitude"],
                drone_formation_position_at_target["longitude"],
                drone_formation_position_at_target["altitude"],  # GPS yüksekliğine göre ayarlanır
                0,  # yaw
            )
            await self.drone.mavsdk_controller.mavsdk.action.set_current_speed(self.drone.speed_limit)
            return
        self.drone.offboard_status["target_position"] = drone_formation_position_at_target
        self.drone.offboard_status["altitude_to_keep"] = drone_formation_position_at_target["altitude"]

    async def goto_location_with_formation_check(self, target_location):
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        distance_to_target = distance_meters(gps_position, self.drone.formation["position"])
        # Hedefe ulaşma durumunda True döndür
        if (distance_to_target <= self.drone.waypoint_threshold):
            self.drone.offboard_status["target_position"] = None
            logging.info(f"Drone hedef konuma ulaştı, anlık konum: {gps_position}")
            return True
        return False

    async def go_home(self):
        """
        Drone'u ev konumuna geri döndürür, 
        ilk başta irtifayı yarıya indirir,
        ardından iniş yapar.
        """
        logging.info("Drone ev konumuna dönüyor...")
        current_data = await self.drone.mavsdk_controller.get_general_info()
        current_gps = current_data["gps_position"]
        self.drone.offboard_status["is_active"] = False
        await self.drone.mavsdk_controller.mavsdk.action.goto_location(
            current_gps["latitude"],
            current_gps["longitude"],
            current_gps["altitude"]-self.drone.altitude_target/2,
            0,  # yaw
        )
        while abs(current_gps["altitude"] - self.drone.pre_takeoff_location["altitude"]) > (self.drone.altitude_target/2)+self.drone.waypoint_threshold:
            current_data = await self.drone.mavsdk_controller.get_general_info()
            current_gps = current_data["gps_position"]
            logging.info(f"Drone ev konumuna dönüyor, anlık irtifa: {current_gps['altitude']}")
            logging.info(abs(current_gps["altitude"] - self.drone.pre_takeoff_location["altitude"]))
            await asyncio.sleep(0.5)
        await self.drone.mavsdk_controller.mavsdk.action.return_to_launch()
        await self.drone.send_message_with_ack("mh1")

    async def wait_for_neighbor_home(self):
        """
        Komşu dronların ev konumuna dönmesini bekler.
        """
        neighbors_at_home = next((n for n in self.drone.neighbors if n["data"]["is_home"] == True), None)
        if not neighbors_at_home:
            logging.debug("Hiçbir komşu drone ev konumuna dönmedi.")
            return False
        return True
    
    async def wait_for_neighbor_swap(self):
        """
        Komşu dronların pozisyon değişimini bekler.
        """
        neighbors_at_swapped = next((n for n in self.drone.neighbors if n["data"]["is_on_position"] == True), None)
        if not neighbors_at_swapped:
            logging.debug("Hiçbir komşu drone pozisyon değişimi yapmadı.")
            return False
        return True

    async def goto_homed_neighbor_position(self):
        """
        Ev konumuna dönen bir komşunun eski pozisyonuna gider.
        """
        homed_neighbors = [n for n in self.drone.neighbors if n["data"]["is_home"] == True]
        if not homed_neighbors:
            logging.debug("Ev konumuna dönen komşu drone bulunamadı.")
        # İlk ev konumuna dönen komşu ile pozisyon değiştir
        neighbor = homed_neighbors[0]
        current_data = await self.drone.mavsdk_controller.get_general_info()
        current_gps = current_data["gps_position"]
        logging.info(f"Ev konumuna dönen komşu drone ile pozisyon değiştiriliyor: {neighbor['sender']}")
        await self.drone.mavsdk_controller.mavsdk.action.goto_location(
            neighbor["data"]["target_position"]["latitude"],
            neighbor["data"]["target_position"]["longitude"],
            current_gps["altitude"]+self.drone.altitude_target/2, # Başta yarım irtifa ile gidip, lat, lon oturunca irtifayı yükseltecek
            0,  # yaw
        )
        is_on_homed_neighbor_position = self.goto_location_check(neighbor["data"]["target_position"])
        while not is_on_homed_neighbor_position:
            is_on_homed_neighbor_position = await self.goto_location_check(neighbor["data"]["target_position"])
            await asyncio.sleep(0.5)
        await self.drone.mavsdk_controller.mavsdk.action.goto_location(
            neighbor["data"]["target_position"]["latitude"],
            neighbor["data"]["target_position"]["longitude"],
            current_gps["altitude"]+self.drone.altitude_target, # İrtifayı yükseltecek
            0,  # yaw
        )
    
    async def goto_homed_neighbor_position_check(self):
        homed_neighbors = [n for n in self.drone.neighbors if n["data"]["is_home"] == True]
        if not homed_neighbors:
            logging.debug("Ev konumuna dönen komşu drone bulunamadı.")
        # İlk ev konumuna dönen komşu ile pozisyon değiştir
        neighbor = homed_neighbors[0]
        is_on_homed_neighbor_position = await self.goto_location_check(neighbor["data"]["target_position"])
        if is_on_homed_neighbor_position:
            logging.info("Drone ev konumuna dönen komşu dronun formasyon konumu ile aynı konumda.")
            await self.drone.send_message_with_ack("ms1")
        return is_on_homed_neighbor_position
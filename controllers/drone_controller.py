import os
import sys 
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import time
import asyncio
import numpy as np
import json
import logging
import math
import pymap3d

from controllers import mavsdk_controller
from controllers import xbee_controller

from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import Position

from step_controller import StepController, Step

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

def format_broadcast_message(message):
    """
    Mesajı kompakt formda broadcast'e uygun şekilde hazırlar.
    
    :param message: Gönderilecek mesaj
    :return: Formatlanmış mesaj
    """
    is_armable = 1 if message["health"] else 0
    is_armed = 1 if message["armed"] else 0
    flight_mode = message["flight_mode"]
    battery = int(message["battery"])
    gps_string = f"{message['gps_position']['latitude']:.6f},{message['gps_position']['longitude']:.6f},{format(message['gps_position']['altitude'], '.4f')}"
    velocity = f"{message['velocity']['north']},{message['velocity']['east']},{message['velocity']['down']}"
    new_message = f"{is_armable},{is_armed},{flight_mode},{battery},{gps_string}"
    logging.debug(f"Broadcast mesajı hazırlandı: {new_message}")
    return new_message

def deg_to_rad(degrees):
    return degrees * math.pi / 180

def calculate_distance(coord1, coord2):
    """
    İki koordinat arasındaki mesafeyi hesaplar.
    
    :param coord1: İlk koordinat (latitude, longitude)
    :param coord2: İkinci koordinat (latitude, longitude)
    :return: Mesafe (metre cinsinden)
    """
    R = 6371  # Dünya'nın yarıçapı (kilometre cinsinden)
    dist_lat = deg_to_rad(coord2["latitude"] - coord1["latitude"])
    dist_lon = deg_to_rad(coord2["longitude"] - coord1["longitude"])

    lat1 = deg_to_rad(coord1["latitude"])
    lat2 = deg_to_rad(coord2["latitude"])
    # Haversine formülü ile mesafe hesaplama
    a = (math.sin(dist_lat / 2) ** 2 +
         math.sin(dist_lon / 2) ** 2 * math.cos(lat1) * math.cos(lat2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c * 1000  # Mesafeyi metre cinsine çevir
    logging.debug(f"Koordinatlar arasındaki mesafe: {distance} metre")
    return distance

class DroneController:
    def __init__(self, xbee_port = None, mavsdk_port="udpin://0.0.0.0:14540"):
        
        if xbee_port is not None:
            logging.info(f"XBee portu: {xbee_port} olarak ayarlandı.")
            self.XBeeController = xbee_controller.XBeeController(
                port=xbee_port,
                message_received_callback=self.handle_message_received
            )
            self.XBeeController.listen()
            logging.info("XBeeController başlatıldı.")
            asyncio.create_task(self.broadcast_drone_status())
        self.xbee_id = self.XBeeController.address if xbee_port is not None else "TESTING"
        self.MAVSDKController = mavsdk_controller.MAVSDKController(system_address=mavsdk_port)
        self.drone = self.MAVSDKController.drone
        self.neighbors = []

    
    def handle_message_received(self, recieved_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu.

        :param message: XBee'den alınan mesaj
        """
        # Örnek data
        # 1,1,6,50,47.3977058,8.5460053,1.3350
        if recieved_message.sender not in self.neighbors:
            message_raw = recieved_message.data.split(',')
            if len(message_raw) < 10:
                logging.error(f"Beklenmeyen mesaj formatı: {recieved_message.data}")
                return
            message_data = {
                "sender": recieved_message.sender,
                "timestamp": recieved_message.timestamp,
                "data": {
                    "armable": bool(int(message_raw[0])),
                    "armed": bool(int(message_raw[1])),
                    "flight_mode": int(message_raw[2]),
                    "battery": int(message_raw[3]),
                    "gps_position": {
                        "latitude": float(message_raw[4]),
                        "longitude": float(message_raw[5]),
                        "altitude": float(message_raw[6])
                    },
                    "velocity": {
                        "north": float(message_raw[7]),
                        "east": float(message_raw[8]),
                        "down": float(message_raw[9])
                    }
                }
            }
            self.neighbors.append(message_data)
            logging.info(f"Yeni komşu eklendi: {recieved_message.sender}.\nKomşu ile aradaki gecikme: {recieved_message.timestamp - time.time()} saniye.")

    async def broadcast_drone_status(self):
        """
        Bu fonksiyon, dronun genel durumunu alır ve XBee üzerinden broadcast eder

        param XBeeController: XBee kontrolcüsü
        param MAVSDKController: MAVSDK kontrolcüsü

        """
        while True:
            if not self.MAVSDKController.is_connected or not self.XBeeController.device.is_open():
                logging.info(self.MAVSDKController.is_connected)
                logging.info(self.XBeeController.device.is_open())
                logging.warning("MAVSDKController veya XBee henüz bağlı değil, durum broadcast edilemiyor, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            data = await self.MAVSDKController.get_general_info()
            if data["health"] is None:
                logging.warning("Drone'un durumu henüz alınamadı, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            message = format_broadcast_message(data)
            try:
                self.XBeeController.send_broadcast_message(message)
            except Exception as e:
                logging.error(f"Broadcast mesajı gönderilirken hata oluştu: {e}")
            logging.info(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1.5)  # Her saniyede bir güncel durumu broadcast et
    # Sık kullanılan drone işlemleri
    async def arm(self):
        """
        Drone'u arm eder
        """
        await self.drone.action.arm()

    async def disarm(self):
        """
        Drone'u disarm eder
        """
        await self.drone.action.disarm()
    
    async def takeoff(self, altitude): 
        """
        Drone'a kalkış komutu gönderir
        
        :param altitude: Kalkış yüksekliği
        """
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()

    async def land(self): 
        """
        Drone'a iniş komutu gönderir
        """
        await self.drone.action.land()
    
    # PID & APF Mekanizmaları
    # async def background_offboard_controller(self):



async def main():
    """
    DroneController temel işlemlerini test eden ana fonksiyon.
    Bu fonksiyon, drone'u arm eder, kalkış yapar, belirli bir yüksekliğe çıkar, iniş yapar ve disarm eder.
    """
    drone_controller = DroneController(
            # xbee_port="/dev/ttyUSB0", 
            # mavsdk_port="serial:///dev/ttyACM0:57600"
        )
    step_controller = StepController()
    # Waypointler
    target_locations = [
        {
            "latitude": 40.325763, 
            "longitude": 36.473505,
            "altitude": 10,
        },
        {
            "latitude": 40.325672,
            "longitude": 36.473580,
            "altitude": 10,
        },
        {
            "latitude": 40.325460,
            "longitude": 36.473591,
            "altitude": 10,
        },
    ]
    target_locations2 = [
        {
            "latitude": 47.397851,
            "longitude": 8.546990,
            "altitude": 10,
        },
        {
            "latitude": 47.397372,
            "longitude": 8.546582,
            "altitude": 10,
        },
        {
            "latitude": 47.397713,
            "longitude": 8.546003,
            "altitude": 10,
        },
    ]

    await drone_controller.MAVSDKController.connect()
    while not drone_controller.MAVSDKController.is_connected:
        logging.info(drone_controller.MAVSDKController.is_connected)
        logging.info("DroneController henüz bağlı değil, bağlanmaya çalışılıyor...")
        await asyncio.sleep(1)
    logging.info("DroneController bağlı.")
    pre_takeoff_altitude = None
    while True:
        _general_info = await drone_controller.MAVSDKController.get_general_info()
        _gps_position = _general_info["gps_position"]
        if _gps_position and "altitude" in _gps_position:
            pre_takeoff_altitude = _gps_position["altitude"]
            break
        logging.info("GPS yükseklik bilgisi henüz alınamadı, bekleniyor...")
        await asyncio.sleep(0.5)
    # Örnek kullanım
    # Arm eder
    async def arm():
        """0
        Drone'u arm eden adım fonksiyonu.
        """
        logging.info("Drone arm ediliyor...")
        await drone_controller.arm()
        # Offboard moduna geçer
        logging.info("Drone offboard moduna geçiyor...")
    async def arm_check():
        """
        Drone'un arm durumunu kontrol eden fonksiyon.
        """
        return await drone_controller.MAVSDKController.is_armed()
    step_controller.add_step(Step("arm", arm, arm_check))
    # Diğer dronların broadcast mesajlarını bekle
    async def wait_for_broadcast():
        """
        Drone'un diğer dronların broadcast mesajlarını beklediği adım fonksiyonu.
        """
        logging.info("Drone diğer dronların broadcast mesajlarını bekliyor...")
        logging.info("Diğer bir drone keşfedildi.")
    async def wait_for_broadcast_pre_check():
        """
        Drone'un diğer dronların broadcast mesajlarını alıp almadığını kontrol eden fonksiyon.
        
        :return: True eğer en az bir komşu varsa; aksi halde False
        """
        if len(drone_controller.neighbors) > 0:
            logging.info(f"Komşular: {drone_controller.neighbors}")
            return True
        return False
    # Offboard hazırlıklarını yapar ve başlatır
    async def switch_to_offboard():
        """
        Drone'u offboard moduna geçiren adım fonksiyonu.
        """
        logging.info("Drone offboard moduna geçiyor...")
        try:
            await drone_controller.drone.telemetry_server.publish_home(
                Position(
                    latitude=pre_takeoff_altitude,  # GPS yüksekliğine göre ayarlanır
                    longitude=0,
                    altitude=0  
                )
            )
            await drone_controller.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await drone_controller.drone.offboard.start()
            logging.info("Drone offboard moduna geçti.")
        except OffboardError as e:
            logging.error(f"Offboard moduna geçilirken hata oluştu: {e}")
    # Kalkış yapar
    target_altitude = 10  # Kalkış yüksekliği
    async def takeoff():
        """
        Drone'u kalkışa hazırlayan adım fonksiyonu.
        """
        logging.info("Drone kalkış yapıyor...")
        await drone_controller.takeoff(target_altitude)
    async def altitude_check(target_altitude=target_altitude):
        """
        Drone'un kalkış durumunu kontrol eden fonksiyon.
        """
        general_info = await drone_controller.MAVSDKController.get_general_info()
        gps_position = general_info["gps_position"]
        climbed = abs(gps_position["altitude"] - pre_takeoff_altitude)
        logging.info(f"Drone hedef irtifa ile {climbed} metre mesafede.")
        if abs(target_altitude - climbed) <= 0.2:
            logging.debug(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
            return True
    step_controller.add_step(Step("takeoff", takeoff, altitude_check))
    # Waypoint'lere ilerler
    async def goto_location(target_location):
        """
        Drone'u belirli bir konuma götüren adım fonksiyonu.
        
        :param target_location: Hedef konum (latitude, longitude, altitude)
        """
        logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
        await drone_controller.drone.action.goto_location(
            target_location["latitude"],
            target_location["longitude"],
            target_location["altitude"]+pre_takeoff_altitude,  # GPS yüksekliğine göre ayarlanır
            0,  # yaw
        )
        
    async def goto_location_check(target_location):
        """
        Drone'un belirli bir konuma ulaşıp ulaşmadığını kontrol eden fonksiyon.
        
        :param target_location: Hedef konum (latitude, longitude, altitude)
        :return: True eğer drone hedef konuma ulaştıysa; aksi halde False
        """
        general_info = await drone_controller.MAVSDKController.get_general_info()
        gps_position = general_info["gps_position"]
        logging.info(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
        if (calculate_distance(gps_position, target_location) <= 0.5):
            logging.info("Drone hedef konuma ulaştı.")
            return True
        return False
    for i, target_location in enumerate(target_locations2):
        step_name = f"goto_location_{i+1}"
        step_controller.add_step(
            Step(
                step_name, 
                lambda loc=target_location: goto_location(loc), 
                lambda loc=target_location: goto_location_check(loc)
            )
        )
        logging.info(f"{step_name} adımı eklendi.")
    # İniş yapar
    async def land():
        """
        Drone'u inişe hazırlayan adım fonksiyonu.
        """
        logging.info("Drone iniş yapıyor...")
        await drone_controller.land()
    step_controller.add_step(Step("land", land,
                lambda alt=0: altitude_check(alt)
            ))
    # Disarm eder
    async def disarm_pre_check():
        """
        Drone'un disarm durumunu kontrol eden fonksiyon.
        """
        async for is_in_air in drone_controller.MAVSDKController.drone.telemetry.in_air():
            return not is_in_air
    async def disarm():
        """
        Drone'u disarm eden adım fonksiyonu.
        """
        logging.info("Drone disarm ediliyor...")
        await drone_controller.disarm()
    async def disarm_check():
        """
        Drone'un disarm durumunu kontrol eden fonksiyon.
        """
        is_armed = await drone_controller.MAVSDKController.is_armed()
        if not is_armed:
            return True
        return False
    step_controller.add_step(Step("disarm", disarm, disarm_check, disarm_pre_check))
    logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
    await step_controller.run_steps()
    while not step_controller.is_all_done:
        logging.debug("Adımlar hala çalışıyor, bekleniyor...")
        await asyncio.sleep(1)
    logging.info("DroneController testinin tüm adımları tamamlandı.")

if __name__ == "__main__":
    logging.info("DroneController başlatıldı.")
    asyncio.run(main())
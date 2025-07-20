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

from utils import pid
from utils.collision_avoidance import apf

from controllers import mavsdk_controller
from controllers import xbee_controller

from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

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

class DroneController:
    def __init__(self, xbee_port = None, mavsdk_port="udpin://0.0.0.0:14540"):
        # Kontrolcüler
        self.pid = pid.PID()      
        self.apf = apf.APF()
        
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
        # 1,1,6,50,47.3977058,8.5460053,1.3350,-0.03999999910593033,0.0,0.6800000071525574
        # 47.3977058,8.5460053,1.3350
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
    
    def compute_control_velocity(self, target_position, neighbor_positions):
        """
        Kontrol hızı hesapla
        
        :param target_position: Hedef konum
        :param neighbor_positions: Diğer dronların konumları ([x,y,z] şeklinde)
        :return: Hesaplanan hız vektörü
        """
        current_pos = self.get_position()
        v_pid = self.pid.compute(target_position, current_pos)
        f_apf = self.apf.calculate(current_pos, neighbor_positions)
        total_velocity = v_pid + f_apf
        return total_velocity
    
    def send_velocity_command(self, velocity): 
        """
        Hız komutu gönder

        :param velocity: Hız vektörü (x, y, z)
        """
        self.drone.offboard.set_velocity_ned(velocity)

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

async def main():
    """
    DroneController temel işlemlerini test eden ana fonksiyon.
    Bu fonksiyon, drone'u arm eder, kalkış yapar, belirli bir yüksekliğe çıkar, iniş yapar ve disarm eder.
    """
    drone_controller = DroneController(
            xbee_port="/dev/ttyUSB0", 
            mavsdk_port="serial:///dev/ttyACM0:115200"
        )
    await drone_controller.MAVSDKController.connect()
    while not drone_controller.MAVSDKController.is_connected:
        logging.info(drone_controller.MAVSDKController.is_connected)
        logging.info("DroneController henüz bağlı değil, bağlanmaya çalışılıyor...")
        await asyncio.sleep(1)
    
    # Örnek kullanım
    target_altitude = 10
    await drone_controller.arm()
    logging.debug("arm() komutu verildi.")
    await asyncio.sleep(1)
    await drone_controller.drone.param_server.provide_param_float("MPC_XY_CRUISE", 1.0)
    # await drone_controller.drone.offboard.set_velocity_body(
    #     VelocityBodyYawspeed(
    #         1.0,  # 1 m/s hızla kuzeye hareket
    #         0.0,  # Doğu yönünde hareket yok
    #         0.0,  # Aşağı yönünde hareket yok
    #         0.0  # Yaw hızı yok
    #     )
    # )
    logging.info("Hız 1m/sn ayarlandı.")
    # await drone_controller.drone.action_server.set_flight_mode(drone_controller.drone.action_server.FlightMode.OFFBOARD)
    await drone_controller.takeoff(target_altitude)
    while True:
        _general_info = await drone_controller.MAVSDKController.get_general_info()
        _gps_position = _general_info["gps_position"]
        if _gps_position and "altitude" in _gps_position:
            pre_takeoff_altitude = _gps_position["altitude"]
            break
        logging.info("GPS yükseklik bilgisi henüz alınamadı, bekleniyor...")
        await asyncio.sleep(0.5)
    logging.debug("takeoff() komutu verildi.")
    while True:
        general_info = await drone_controller.MAVSDKController.get_general_info()
        gps_position = general_info["gps_position"]
        climbed = gps_position["altitude"] - pre_takeoff_altitude
        logging.info(f"Drone {climbed} metre yükseldi.")
        if abs(target_altitude - climbed) <= 0.2:
            logging.info(f"Drone {target_altitude} metreye yeterince yakınlaştı, land() komutu veriliyor.")
            break
        await asyncio.sleep(1)
    # Waypoint'e ilerle
    target_locations = [
        {
            "latitude": 40.325763,
            "longitude": 36.473505,
            "altitude": 10,
        },
        {
            "latitude": 40.325672,
            "longitude": 36.43802,
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
            "latitude": 47.398309,
            "longitude": 8.5408438,
            "altitude": 10,
        },
        {
            "latitude": 47.397190,
            "longitude": 8.547258,
            "altitude": 10,
        },
        {
            "latitude": 47.397161,
            "longitude": 8.544898,
            "altitude": 10,
        },
    ]
    for target_location in target_locations:
        await drone_controller.drone.action.goto_location(
            target_location["latitude"],
            target_location["longitude"],
            target_location["altitude"],
            0,  # yaw
        )
        while True:
            general_info = await drone_controller.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            logging.info(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
            if (calculate_distance(gps_position, target_location) <= 0.5):
                logging.info("Drone hedef konuma ulaştı.")
                await asyncio.sleep(1)
                break
            await asyncio.sleep(1)
    # İniş yapar
    await drone_controller.land()
    logging.debug("land() komutu verildi.")
    async for is_in_air in drone_controller.MAVSDKController.drone.telemetry.in_air():
        if not is_in_air:
            logging.info("Drone zeminde, disarm ediliyor.")
            break
        logging.info("Drone hala havada, bekleniyor...")
        await asyncio.sleep(1)
    await drone_controller.disarm()
    logging.info("Drone disarm edildi.")
    logging.info("DroneController işlemleri tamamlandı.")

if __name__ == "__main__":
    logging.info("DroneController başlatıldı.")
    asyncio.run(main())

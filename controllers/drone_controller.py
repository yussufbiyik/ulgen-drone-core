import os
import sys 
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import time
import uuid
import asyncio
import numpy as np
import json
import logging

from utils import pid
from utils.collision_avoidance import apf

from controllers import mavsdk_controller
from controllers import xbee_controller

log_name = "./logs/DroneController.log"
os.makedirs("./logs", exist_ok=True)

sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
fh = logging.FileHandler(log_name, mode='w')
fh.setLevel(logging.DEBUG)
logging.basicConfig(
        format='[%(asctime)s | %(levelname)s]\n\t⤷ %(message)s',
        handlers=[fh, sh]
    )

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
    gps_string = f"{message['gps_position']['latitude']},{message['gps_position']['longitude']},{format(message['gps_position']['altitude'], '.4f')}"
    timestamp = int(time.time() * 1000)
    new_message = f"{is_armable},{is_armed},{flight_mode},{battery},{gps_string},{timestamp}"
    logging.debug(f"Broadcast mesajı hazırlandı: {new_message}")
    return new_message

def handle_message_received(message):
    """
    XBee'den gelen mesajları işleyen callback fonksiyonu.

    :param message: XBee'den alınan mesaj
    """
    logging.info(f"Başka bir XBee'den mesaj alındı: {message.data.decode('utf-8', errors='replace')}")
    return NotImplemented

class DroneController:
    def __init__(self, xbee_port):
        # Kontrolcüler
        self.pid = pid.PID()      
        self.apf = apf.APF()
        
        if xbee_port is not None:
            self.XBeeController = xbee_controller.XBeeController(
                port=xbee_port,
                message_received_callback=handle_message_received
            )
            self.XBeeController.listen()
            asyncio.create_task(self.broadcast_drone_status())
        self.xbee_id = self.XBeeController.address if xbee_port is not None else "TESTING"
        self.MAVSDKController = mavsdk_controller.MAVSDKController(self.xbee_id, system_address="udpin://0.0.0.0:14540")
        self.drone = self.MAVSDKController.drone
        
        # Temel Özellikler
        self.relative_position = np.array([0.0, 0.0, 0.0])

    
    async def broadcast_drone_status(self):
        """
        Bu fonksiyon, dronun genel durumunu alır ve XBee üzerinden broadcast eder

        param XBeeController: XBee kontrolcüsü
        param MAVSDKController: MAVSDK kontrolcüsü

        """
        while True:
            if not self.MAVSDKController.is_connected or not self.XBeeController.device.is_open():
                logging.warning("MAVSDKController veya XBee henüz bağlı değil, durum broadcast edilemiyor, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            data = await self.MAVSDKController.get_general_info()
            if data["health"] is None:
                logging.warning("Drone'un durumu henüz alınamadı, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            message = format_broadcast_message(data)
            self.XBeeController.send_broadcast_message(message)
            logging.info(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1)  # Her saniyede bir güncel durumu broadcast et

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


async def main():
    """
    DroneController temel işlemlerini test eden ana fonksiyon.
    Bu fonksiyon, drone'u arm eder, kalkış yapar, belirli bir yüksekliğe çıkar, iniş yapar ve disarm eder.
    """
    drone_controller = DroneController(xbee_port="/dev/ttyUSB0")  # XBee portunu uygun şekilde ayarlayın
    await drone_controller.MAVSDKController.connect()
    while not drone_controller.MAVSDKController.is_connected:
        logging.info(drone_controller.MAVSDKController.is_connected)
        logging.info("DroneController henüz bağlı değil, bağlanmaya çalışılıyor...")
        await asyncio.sleep(1)
    
    # Örnek kullanım
    target_altitude = 5
    await drone_controller.arm()
    logging.debug("arm() komutu verildi.")
    while True:
        _general_info = await drone_controller.MAVSDKController.get_general_info()
        _gps_position = _general_info["gps_position"]
        if _gps_position and "altitude" in _gps_position:
            pre_takeoff_altitude = _gps_position["altitude"]
            break
        logging.info("GPS konum bilgisi henüz alınamadı, bekleniyor...")
        await asyncio.sleep(0.5)
    await drone_controller.takeoff(target_altitude)
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
    # İniş yap
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
    asyncio.run(main())
    logging.info("DroneController başlatıldı.")
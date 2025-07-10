import os
import time
import uuid
import asyncio
import numpy as np
import json
import threading
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

def broadcast_drone_status(XBeeController, MAVSDKController):
    """
    Dron durumunu yayınlayan fonksiyon.
    """
    while True:
        data = MAVSDKController.get_all()
        message = XBeeController.construct_message(data)
        XBeeController.send_broadcast_message(message)
        logging.info(f"Güncel durum broadcast edildi: {message}")
        time.sleep(5)  # Her 5 saniyede bir güncelleme yap

def handle_message_received(message):
    """
    XBee'den gelen mesajları işleyen callback fonksiyonu.
    """
    logging.info(f"Başka bir XBee'den mesaj alındı: {message.data.decode('utf-8', errors='replace')}")
    return NotImplemented

class DroneController:
    def __init__(self):
        self.uuid = str(uuid.uuid4())
        
        # Kontrolcüler
        self.pid = pid.PID()
        self.apf = apf.APF()
        
        # self.XBeeController = xbee_controller.XBeeController(self.uuid, "PORT", message_received_callback=handle_message_received)
        self.MAVSDKController = mavsdk_controller.MAVSDKController(self.uuid)
        
        # Temel Özellikler
        self.relative_position = np.array([0.0, 0.0, 0.0])
        
        # Rutin operasyonlar
        # threading.Thread(target=broadcast_drone_status(self.XBeeController, self.MAVSDKController), daemon=True).start()

    async def arm(self):
        """
        Drone'u arm eder
        """
        await self.MAVSDKController.drone.action.arm()

    async def disarm(self):
        """
        Drone'u disarm eder
        """
        await self.MAVSDKController.drone.action.disarm()
    
    async def takeoff(self, altitude): 
        """
        Drone'a kalkış komutu gönderir
        
        :param altitude: Kalkış yüksekliği
        """
        await self.MAVSDKController.drone.action.set_takeoff_altitude(altitude)
        await self.MAVSDKController.drone.action.takeoff()

    async def land(self): 
        """
        Drone'a iniş komutu gönderir
        """
        await self.MAVSDKController.drone.action.land()
    
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
        """
        return NotImplemented
    
async def main():
    """
    DroneController temel işlemlerini gerçekleştirir.
    """
    drone_controller = DroneController()
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
        _gps_position = _general_info["status"].get("gps_position")
        if _gps_position and "altitude" in _gps_position:
            pre_takeoff_altitude = _gps_position["altitude"]
            break
        logging.info("GPS konum bilgisi henüz alınamadı, bekleniyor...")
        await asyncio.sleep(0.5)
    await drone_controller.takeoff(target_altitude)
    logging.debug("takeoff() komutu verildi.")
    while True:
        general_info = await drone_controller.MAVSDKController.get_general_info()
        gps_position = general_info["status"].get("gps_position")
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
import os
import time
import uuid
import numpy as np
import threading
import logging

from ..utils.pid import PID
from ..utils.collision_avoidance.apf import APF

from mavsdk_controller import MAVSDKController
from xbee_controller import XBeeController

logging.basicConfig(
        level=logging.INFO, 
        format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
        filename=f"../logs/DRONE_{int(time.time()*1000)}.log",
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
        self.id = str(uuid.uuid4())
        
        # Kontrolcüler
        self.pid = PID()
        self.apf = APF()
        
        self.XBeeController = XBeeController(self.uuid, "PORT", message_received_callback=handle_message_received)
        self.MAVSDKController = MAVSDKController()
        
        # Temel Özellikler
        self.relative_position = np.array([0.0, 0.0, 0.0])
        current_properties = self.MAVSDKController.get_all()
        self.all_properties = {
            "id": self.id,
            "status": {
            "state": {
                "armed": current_properties.is_armed,
                "mode": current_properties.flight_mode,
            },
            "battery": current_properties.battery,
            "position": current_properties.gps_position,
            "attitude": current_properties.attitude,
            "velocity": current_properties.velocity,
            }
        }
        
        # Rutin operasyonlar
        threading.Thread(target=broadcast_drone_status(self.XBeeController, self.MAVSDKController), daemon=True).start()

    def arm(self):
        """
        Drone'u arm eder
        """
        return NotImplemented
    
    def takeoff(self, altitude): 
        """
        Drone'a kalkış komutu gönderir
        
        :param altitude: Kalkış yüksekliği
        """
        return NotImplemented
    
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
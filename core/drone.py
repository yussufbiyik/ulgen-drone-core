import threading
import asyncio
import logging
import socket
import time
import random
import json
import math

from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController
from controllers.offboard_controller import OffboardController

from utils.pid import PID
from utils.apf import APF

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

SERVER_IP = "127.0.0.1"
SERVER_PORT = 5005

class Drone:
    def __init__(self, xbee_controller: XBeeController, mavsdk_controller: MAVSDKController, isTesting=False):
        self.isTesting = isTesting

        # Test modunda soket üzerinden iletişim kurmak için
        self.socket = None
        self.fake_id = random.randint(10000, 99999) if isTesting else None
        # XBee
        self.xbee_controller = xbee_controller
        self.xbee_id = self.xbee_controller.address if not self.isTesting else self.fake_id
        # MAVSDK
        self.mavsdk_controller = mavsdk_controller
        
        if not self.isTesting:
            self.xbee_controller.message_received_callback = self.handle_message_received
            self.xbee_controller.listen()
            logging.info("XBee iletişimi başlatıldı.")
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            threading.Thread(target=self.listen_to_socket).start()
            logging.info("Soket iletişimi başlatıldı.")
        asyncio.create_task(self.broadcast_drone_status())
        self.pre_takeoff_location = {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0
        }  # Aslında home gibi
        self.speed_limit = 1.0  # m/s olarak varsayılan hız sınırı
        self.waypoint_threshold = 0.5  # m olarak varsayılan waypoint eşiği
        self.offboard_controller = OffboardController(self)
        self.offboard_status = {
            "is_active": False,
            "altitude_to_keep": 0.0,
            "target_position": None,
            "navigation_method": "pid", # "standard" veya "pid"
        }
        # Formasyon için gerekli değişkenler
        self.formation_position = None
        self.formation_weight_center = None
        self.neighbor_formation_positions = []
        # Yatay eksen için PID kontrolcüsü
        self.pid_ne = PID(
            Kp=0.6, Ki=0.005, Kd=0.4,
            max_output=self.speed_limit, min_output=-self.speed_limit, error_threshold=self.waypoint_threshold,
            slowing_minimum=0.5
        )
        self.apf = APF()
        
        self.neighbors = []

    # XBee ve simülasyon içi iletişim ile alakalı işlemler
    def listen_to_socket(self):
        """
        Soket üzerinden gelen verileri dinler ve işler.
        """
        while True:
            data, _ = self.socket.recvfrom(2048)
            try:
                msg = json.loads(data.decode())
                logging.debug(f"{msg['sender']} adresindeki drondan mesaj alındı, zaman: {msg['timestamp']}, içerik: {msg['data']}")
                self.handle_message_received(msg)
            except Exception as e:
                logging.exception(f"Mesaj çözümlenemedi: {e}")

    def handle_message_received(self, recieved_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu.

        :param message: XBee'den alınan mesaj
        """
        # Örnek data
        # 1,1,6,50,47.3977058,8.5460053,1.3350
        if not recieved_message["sender"]:
            logging.error(f"Mesajın göndereni belirtilmemiş, geçiliyor.")
            return
        message_raw = recieved_message['data'].split(',')
        if len(message_raw) < 7:
            logging.error(f"Beklenmeyen mesaj formatı: {recieved_message['data']}")
            return
        logging.debug("Komşu drone mesajı alındı, işleniyor...")
        if not any(neighbor["sender"] == recieved_message["sender"] for neighbor in self.neighbors):
            logging.debug(f"Yeni komşu drone bulundu: {recieved_message['sender']}")
            message_data = {
                "sender": recieved_message["sender"],
                "timestamp": recieved_message["timestamp"],
                "data": {
                    "armable": bool(int(message_raw[0])),
                    "armed": bool(int(message_raw[1])),
                    "flight_mode": int(message_raw[2]),
                    "battery": int(message_raw[3]),
                    "gps_position": {
                        "latitude": float(message_raw[4]),
                        "longitude": float(message_raw[5]),
                        "altitude": float(message_raw[6])
                    }
                }
            }
            self.neighbors.append(message_data)
            logging.info(f"Yeni komşu eklendi: {recieved_message['sender']} ({recieved_message['timestamp'] - time.time()}ms).")
        else:
            logging.debug(f"Komşu zaten mevcut:, güncelleniyor ({recieved_message['timestamp'] - time.time()}ms).")
            for neighbor in self.neighbors:
                if neighbor["sender"] == recieved_message["sender"]:
                    neighbor["data"]["armable"] = bool(int(message_raw[0]))
                    neighbor["data"]["armed"] = bool(int(message_raw[1]))
                    neighbor["data"]["flight_mode"] = int(message_raw[2])
                    neighbor["data"]["battery"] = int(message_raw[3])
                    neighbor["data"]["gps_position"]["latitude"] = float(message_raw[4])
                    neighbor["data"]["gps_position"]["longitude"] = float(message_raw[5])
                    neighbor["data"]["gps_position"]["altitude"] = float(message_raw[6])
                    break

    async def broadcast_drone_status(self):
        """
        Bu fonksiyon, dronun genel durumunu alır ve XBee üzerinden broadcast eder
        """
        while True:
            # MAVSDKController ve XBeeController'ın bağlı olup olmadığını kontrol et
            if not self.mavsdk_controller.is_connected:
                logging.warning("MAVSDKController bağlı değil, durum broadcast edilemiyor, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            data = await self.mavsdk_controller.get_general_info()
            if data["health"] is None:
                logging.warning("Drone'un durumu henüz alınamadı, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            message = format_broadcast_message(data)
            try:
                if self.isTesting:
                    test_message = json.dumps({
                        "sender": self.fake_id,
                        "data": message,
                    })
                    self.socket.sendto(
                        test_message.encode('utf-8'),
                        (SERVER_IP, SERVER_PORT)
                    )
                else:
                    if not self.xbee_controller.device.is_open():
                        logging.warning("XBeeController bağlı değil, durum broadcast edilemiyor, broadcast beklemede.")
                        await asyncio.sleep(1)
                        continue
                    self.xbee_controller.send_broadcast_message(message)
            except Exception as e:
                logging.error(f"Broadcast mesajı gönderilirken hata oluştu: {e}")
                continue
            logging.debug(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1.5)  # Her saniyede bir güncel durumu broadcast et
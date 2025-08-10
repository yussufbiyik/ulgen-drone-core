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
    mission = f"{message['mission']['current_step']['index']},{message['mission']['current_step']['status']}"
    new_message = f"{is_armable},{is_armed},{flight_mode},{battery},{gps_string},{mission}"
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
        self.speed_limit = 3.0  # m/s olarak varsayılan hız sınırı
        self.waypoint_threshold = 1.0  # m olarak varsayılan waypoint eşiği
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
        self.formation_type = None
        self.formation_distance = None
        self.neighbor_formation_positions = []
        self.mission_info = {
            "current_step": {
                "index": None,
                "status": 0
            }
        }
        # Yatay eksen için PID kontrolcüsü
        self.pid_ne = PID(
            Kp=0.45, Ki=0.005, Kd=0.1,
            max_output=self.speed_limit, min_output=-self.speed_limit, error_threshold=self.waypoint_threshold
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

    def process_drone_status_message(self, message):
        """
        Diğer dronlardan gelen sıradan konum vb. verileri içeren statü mesajlarını işler.
        """
        sender = message["sender"]
        message_data = message['data'].split(',')
        if len(message_data) < 7:
            logging.warning("Mesaj verisi eksik, geçiliyor.")
            return
        neighbor = next((n for n in self.neighbors if n["sender"] == sender), None)
        if not neighbor:
            logging.debug(f"Yeni komşu drone bulundu: {sender}, ekleniyor...")
            message_data = {
                "sender": sender,
                "timestamp": message["timestamp"],
                "data": {
                    "armable": bool(int(message_data[0])),
                    "armed": bool(int(message_data[1])),
                    "flight_mode": int(message_data[2]),
                    "battery": int(message_data[3]),
                    "gps_position": {
                        "latitude": float(message_data[4]),
                        "longitude": float(message_data[5]),
                        "altitude": float(message_data[6])
                    },
                    "mission": {
                        "current_step": {
                            "index": int(message_data[7]),
                            "status": int(message_data[8]),
                        }
                    },
                }
            }
            self.neighbors.append(message_data)
            logging.info(f"Yeni komşu eklendi: {sender} ({(message['timestamp'] - time.time()):.2f}ms).")
        else:
            logging.debug(f"{sender} zaten mevcut:, güncelleniyor ({(message['timestamp'] - time.time()):.2f}ms).")
            data = neighbor["data"]
            gps = data["gps_position"]
            mission_step = data["mission"]["current_step"]
            data["armable"] = bool(int(message_data[0]))
            data["armed"] = bool(int(message_data[1]))
            data["flight_mode"] = int(message_data[2])
            data["battery"] = int(message_data[3])
            gps["latitude"] = float(message_data[4])
            gps["longitude"] = float(message_data[5])
            gps["altitude"] = float(message_data[6])
            mission_step["index"] = int(message_data[7])
            mission_step["status"] = int(message_data[8])

    def process_mission_message(self, message):
        """
        Diğer dronlardan gelen görev mesajlarını işler.
        """
        sender = message["sender"]
        neighbor= next((n for n in self.neighbors if n["sender"] == sender), None)
        if not neighbor:
            logging.warning(f"Komşu drone bulunamadı: {sender}, mesaj işlenemiyor.")
            return
        message_data = message['data'].split(',')
        if message_data[1] == "t":
            # Formasyon konumuna gitme mesajı
            if len(message_data) < 3:
                logging.warning("Formasyon konumu mesajı eksik, geçiliyor.")
                return
            latitude = float(message_data[2])
            longitude = float(message_data[3])
            neighbor["data"]["target_position"] = {
                "latitude": latitude,
                "longitude": longitude
            }
            # neighbor["data"]["target_status"] = bool(int(message_data[4]))
            logging.debug(f"{sender} drone'u, {latitude}, {longitude} hedef konumuna gidiyor.")
        elif message_data[1] == "ts":
            neighbor["data"]["target_status"] = bool(int(message_data[2]))
            logging.debug(f"{sender} drone'u, formasyon hesaplarını tamamladı.")

    def handle_message_received(self, recieved_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu.

        :param message: XBee'den alınan mesaj
        """
        # Örnek data
        # 1,1,6,50,47.3977058,8.5460053,1.3350
        sender = recieved_message["sender"]
        if not sender:
            logging.error(f"Mesajın göndereni belirtilmemiş, geçiliyor.")
            return
        try:
            message_raw = recieved_message['data'].split(',')
        except Exception as e:
            logging.error("Desteklenmeyen mesaj formatı, geçiliyor.")
            return
        logging.debug("Komşu drone mesajı alındı, işleniyor...")
        is_drone_status_message = message_raw[0].isdigit()
        if is_drone_status_message and int(message_raw[0]):
            self.process_drone_status_message(recieved_message)
        elif message_raw[0] == "m":
            self.process_mission_message(recieved_message)

    async def broadcast_message(self, message):
        """
        Drondan bir mesaj broadcast eder.

        :param message: Broadcast edilecek mesaj
        """
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
            self.xbee_controller.send_broadcast_message(message)

    async def broadcast_drone_status(self):
        """
        Bu fonksiyon, dronun genel durumunu alır ve XBee üzerinden broadcast eder.
        """
        while True:
            # MAVSDKController ve XBeeController'ın bağlı olup olmadığını kontrol et
            if not self.mavsdk_controller.is_connected:
                logging.warning("MAVSDKController bağlı değil, durum broadcast edilemiyor, broadcast beklemede.")
                await asyncio.sleep(1)
                continue
            data = await self.mavsdk_controller.get_general_info()
            data.update(
                {
                    "mission": {
                        "current_step": {
                            "index": self.mission_info["current_step"]["index"],
                            "status": self.mission_info["current_step"]["status"],
                        }
                    }
                }
            )
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
            await asyncio.sleep(1.5)  # Her 1.5 saniyede bir güncel durumu broadcast et
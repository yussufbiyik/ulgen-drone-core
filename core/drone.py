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
    gps_string = f"{message['gps_position']['latitude']:.6f},{message['gps_position']['longitude']:.6f}".replace('.', '')
    velocity = f"{message['velocity']['north']:.1f},{message['velocity']['east']:.1f},{message['velocity']['down']:.1f}".replace('.', '')
    mission = f"{message['mission']['current_step']['index']}{message['mission']['current_step']['status']}"
    new_message = f"{gps_string},{mission}"
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
        self.xbee_id = self.xbee_controller.address if self.xbee_controller else self.fake_id
        # MAVSDK
        self.mavsdk_controller = mavsdk_controller
        
        if not self.isTesting:
            self.xbee_controller.subscribe(self.handle_message_received)
            logging.info("XBee mesajlarına abone olundu.")
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
        self.altitude_target = 0.0  # m olarak varsayılan irtifa hedefi
        self.waypoint_threshold = 1.0  # m olarak varsayılan waypoint eşiği
        self.offboard_controller = OffboardController(self)
        self.offboard_status = {
            "is_active": False,
            "altitude_to_keep": 0.0,
            "target_position": None,
            "navigation_method": "pid", # "standard" veya "pid"
            "is_on_target": False,
        }
        # Formasyon için gerekli değişkenler
        self.formation = {
            "position": None,
            "weight_center": None,
            "type": None,
            "distance": None,
            "neighbor_positions": [],
            "leave": False
        }
        self.mission_info = {
            "current_step": {
                "index": 0,
                "status": 0
            }
        }
        # Yatay eksen için PID kontrolcüsü
        self.pid_ne = PID(
            Kp=0.45, Ki=0.005, Kd=0.1,
            max_output=self.speed_limit, min_output=-self.speed_limit, error_threshold=self.waypoint_threshold
        )
        # Dikey eksen için PID kontrolcüsü
        self.pid_d = PID(
            Kp=0.20, Ki=0.0005, Kd=0.05,
            max_output=0.5, min_output=-0.5, error_threshold=self.waypoint_threshold
        )
        self.apf = APF()
        
        self.neighbors = []
        self.inactive_neighbors = []

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
        if len(message_data) < 3:
            logging.warning("Mesaj verisi eksik, geçiliyor.")
            return
        neighbor = next((n for n in self.neighbors if n["sender"] == sender), None)
        gps_position = {
            "latitude": int(float(message_data[0]))/10**6,
            "longitude": int(float(message_data[1]))/10**6,
        }
        mission = {
            "current_step": {
                "index": int(message_data[2][:-1]),
                "status": int(message_data[2][-1]),
            }
        }
        is_in_inactive_neighbors = any(n["sender"] == sender for n in self.inactive_neighbors)
        if not neighbor and not is_in_inactive_neighbors:
            logging.info(f"Yeni komşu drone bulundu: {sender}, ekleniyor...")
            message_data = {
                "sender": sender,
                "timestamp": message["timestamp"],
                "data": {
                    "gps_position": gps_position,
                    "mission": mission,
                    "is_synced": True,
                    "is_formation_drone": True
                }
            }
            self.neighbors.append(message_data)
            logging.info(f"Yeni komşu eklendi: {sender} ({(message['timestamp'] - time.time()):.2f}ms).")
        elif is_in_inactive_neighbors:
            neighbor = next((n for n in self.inactive_neighbors if n["sender"] == sender), None)
            if neighbor:
                neighbor_data = neighbor["data"]
                neighbor_data["gps_position"] = gps_position
                neighbor_data["mission"]["current_step"] = mission["current_step"]
            logging.debug(f"{sender} drone'u, pasif komşular listesinde, güncellendi.")
        else:
            logging.debug(f"{sender} zaten mevcut:, güncelleniyor ({(message['timestamp'] - time.time()):.2f}ms).")
            data = neighbor["data"]
            data["gps_position"] = gps_position
            data["mission"]["current_step"] = mission["current_step"]

    def process_mission_message(self, message):
        """
        Diğer dronlardan gelen görev mesajlarını işler.
        """
        sender = message["sender"]
        all_neighbors = [
            *self.neighbors,
            *self.inactive_neighbors
        ]
        neighbor = next((n for n in all_neighbors if n["sender"] == sender), None)
        if not neighbor:
            logging.warning(f"Komşu drone bulunamadı: {sender}, mesaj işlenemiyor.")
            return
        message_data = message['data'].split(',')
        if message_data[0] == "mt":
            # Konuma gitme mesajı
            if len(message_data) < 2:
                logging.warning("Konum mesajı eksik, geçiliyor.")
                return
            latitude = int(message_data[1])/10**6
            longitude = int(message_data[2])/10**6
            neighbor["data"]["target_position"] = {
                "latitude": latitude,
                "longitude": longitude
            }
            logging.debug(f"{sender} drone'u, {latitude}, {longitude} hedef konumuna gidiyor.")
            self.send_private_message(sender, f"ACK")
        elif message_data[0] == "mts":
            neighbor["data"]["target_status"] = int(message_data[1])
            logging.debug(f"{sender} drone'u, formasyon hesaplarını tamamladı.")
            self.send_private_message(sender, f"ACK")
        elif message_data[0] == "mf0":
            self.neighbors.remove(neighbor)
            self.inactive_neighbors.append(neighbor)
            logging.debug(f"{sender} drone'u, formasyon dışı olarak işaretlendi.")
            self.send_private_message(sender, f"ACK")
        elif message_data[0] == "mf1":
            self.inactive_neighbors.remove(neighbor)
            self.neighbors.append(neighbor)
            neighbor["data"]["is_formation_drone"] = True
            logging.debug(f"{sender} drone'u, formasyona dahil edildi.")
            self.send_private_message(sender, f"ACK")
        elif message_data[0] == "mh1":
            neighbor["data"]["is_home"] = True
            logging.debug(f"{sender} drone'u, ev konumuna döndü.")
            self.send_private_message(sender, f"ACK")
        elif message_data[0] == "ms1":
            neighbor["data"]["is_synced"] = True
            neighbor["data"]["is_formation_drone"] = True
            logging.debug(f"{sender} drone'u, diğer bir dronun formasyon konumuna döndü.")
            self.send_private_message(sender, f"ACK")

    def process_formation_message(self, message):
        """
        Diğer dronlardan gelen formasyon mesajlarını işler.
        """
        message_data = message['data'].split(',')
        logging.info(message_data)
        if message_data[0] == "fl":
            # Formasyondan çıkma mesajı
            message_subject = int(message_data[1], 16)
            if self.xbee_id == message_subject:
                logging.debug("Formasyondan çıkış emri verildi, iniş yapılacak.")
                self.formation["leave"] = True
            else:
                neighbor = next((n for n in self.neighbors if n["sender"] == message_subject), None)
                neighbor["data"]["leave"] = True
        elif message_data[0] == "fj":
            # Formasyona katılma mesajı
            sender = message["sender"]
            if self.xbee_id == sender:
                return
            neighbor = next((n for n in self.neighbors if n["sender"] == sender), None)
            neighbor["data"]["leave"] = False

    def handle_message_received(self, recieved_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu.

        :param message: XBee'den alınan mesaj
        """
        # Örnek data
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
        elif message_raw[0].startswith("m"):
            self.process_mission_message(recieved_message)
            self.send_private_message(sender, "ACK")
        elif message_raw[0].startswith("f"):
            self.process_formation_message(recieved_message)
        elif message_raw[0] == "ACK":
            sender_in_neighbor_list = next((n for n in self.neighbors if n["sender"] == sender), None)
            if sender_in_neighbor_list:
                logging.info(f"{sender} ID'li drondan, ACK mesajı alındı ve güncellendi.")
                sender_in_neighbor_list["data"]["acknowledged"] = True

    def broadcast_message(self, message):
        """
        Drondan bir mesaj broadcast eder.

        :param message: Broadcast edilecek mesaj
        """
        if self.isTesting:
            message = json.dumps({
                "sender": self.fake_id,
                "data": message,
            })
            self.socket.sendto(
                message.encode('utf-8'),
                (SERVER_IP, SERVER_PORT)
            )
        else:
            self.xbee_controller.send_broadcast_message(message)

    def send_private_message(self, receiver, message):
        """
        Drondan bir mesaj özel olarak gönderir.

        :param receiver: Mesajın gönderileceği alıcı
        :param message: Gönderilecek mesaj
        """
        if self.isTesting:
            message = json.dumps({
                "sender": self.fake_id,
                "target": receiver,
                "data": message,
            })
            self.socket.sendto(
                message.encode('utf-8'),
                (SERVER_IP, SERVER_PORT)
            )
        else:
            self.xbee_controller.send_private_message(receiver, message)

    async def send_message_with_ack(self, message):
        """
        Drondan bir mesajı broadcast eder, teslim alındığının onayını bekler (ACK), 
        teslim almamış olan dronlara da mesajı tekrar gönderir.

        :param receiver: Mesajın gönderileceği alıcı
        :param message: Gönderilecek mesaj
        """
        if self.isTesting:
            message = json.dumps({
                "sender": self.fake_id,
                "data": message,
            })
            self.socket.sendto(
                message.encode('utf-8'),
                (SERVER_IP, SERVER_PORT)
            )
        else:
            self.xbee_controller.send_broadcast_message(message)
        while True:
            not_acknowledged_drones = [
                neighbor for neighbor in self.neighbors
                if not neighbor["data"].get("acknowledged", False)
            ]
            # Bu kontrol tamamlanınca ACK işlemleri için değer false verilir ki,
            # sonraki ACK kontrolünde tüm değerler True olup hatalı sonuç vermesin
            if len(not_acknowledged_drones) == 0:
                logging.info("Tüm dronlar mesajı aldı.")
                for drone in not_acknowledged_drones:
                    drone_in_neighbor_list = next((n for n in self.neighbors if n["sender"] == drone["sender"]), None)
                    drone_in_neighbor_list["data"]["acknowledged"] = False
                return
            else:
                for drone in not_acknowledged_drones:
                    self.send_private_message(drone["sender"], message)
            await asyncio.sleep(1.5 + random.uniform(0, 0.5))

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
                logging.exception(f"Broadcast mesajı gönderilirken hata oluştu: {e}")
                continue
            logging.debug(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1.5 + random.uniform(0, 0.5))  # Her 1.5-2 saniyede bir güncel durumu broadcast et
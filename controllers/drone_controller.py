import sys 

import threading
import time
import asyncio
import numpy as np
import json
import logging
import math
import socket

from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController

from utils.pid import PID
from utils.apf import APF

from mavsdk.offboard import OffboardError, VelocityNedYaw

from controllers.step_controller import StepController, Step

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

class DroneController:
    def __init__(self, xbee_port = None, mavsdk_port="udpin://0.0.0.0:14540", isTesting=False):
        self.isTesting = isTesting
        self.XBeeController = None
        self.socket = None
        if not self.isTesting:
            logging.info(f"XBee portu: {xbee_port} olarak ayarlandı.")
            self.XBeeController = XBeeController(
                port=xbee_port,
                message_received_callback=self.handle_message_received
            )
            self.XBeeController.listen()
            logging.info("XBeeController başlatıldı.")
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            threading.Thread(target=self.listen_to_socket).start()
        asyncio.create_task(self.broadcast_drone_status())
        self.xbee_id = self.XBeeController.address if xbee_port is not None else "TESTING"
        self.MAVSDKController = MAVSDKController(system_address=mavsdk_port)
        self.drone = self.MAVSDKController.drone
        self.pre_takeoff_location = None  # Aslında home gibi
        self.offboardController = {
            "isActive": False,
            "altitudeToKeep": 0.0,
            "targetPosition": None,
        }
        # Her eksen üzerinde kontrol sahibi olmak için
        # PID kontrolörleri eksen başına ayrı ayrı tanımlanır.
        # Yatay eksen
        self.pid_n = PID(Kp=0.15, Ki=0.0, Kd=0.15)
        self.pid_e = PID(Kp=0.15, Ki=0.0, Kd=0.15)
        # Yükseklik ekseni
        self.pid_z = PID(Kp=0.3, Ki=0.0, Kd=0.3)
        self.apf = APF()
        
        self.neighbors = []

    # XBee ile alakalı işlemler
    def listen_to_socket(self):
        while True:
            data, _ = self.socket.recvfrom(2048)
            try:
                msg = json.loads(data.decode())
                logging.info(f"{msg['sender']}. Drone'dan mesaj alındı, zaman: {msg['timestamp']}, içerik: {msg['data']}")
                self.handle_message_received(msg)
            except Exception as e:
                logging.error(f"Mesaj çözümlenemedi: {e}")

    def handle_message_received(self, recieved_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu.

        :param message: XBee'den alınan mesaj
        """
        # Örnek data
        # 1,1,6,50,47.3977058,8.5460053,1.3350
        if not recieved_message.sender:
            logging.error(f"Mesajın göndereni belirtilmemiş, es geçiliyor: {recieved_message.data}")
            return
        message_raw = recieved_message.data.split(',')
        if len(message_raw) < 10:
            logging.error(f"Beklenmeyen mesaj formatı: {recieved_message.data}")
            return
        if recieved_message.sender not in self.neighbors:
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
                    }
                }
            }
            self.neighbors.append(message_data)
            logging.info(f"Yeni komşu eklendi: {recieved_message.sender}.\nKomşu ile aradaki gecikme: {recieved_message.timestamp - time.time()} saniye.")
        else:
            logging.info(f"Komşu zaten mevcut: {recieved_message.sender}, güncelleniyor.")
            for neighbor in self.neighbors:
                if neighbor["sender"] == recieved_message.sender:
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

        param XBeeController: XBee kontrolcüsü
        param MAVSDKController: MAVSDK kontrolcüsü

        """
        while True:
            # Test modunda XBee yerine socket kullanılıyor
            if self.isTesting:
                if not self.MAVSDKController.is_connected:
                    logging.warning("Test modunda, MAVSDKController bağlı değil.")
                    await asyncio.sleep(1)
                    continue
                logging.info("Test modunda, broadcast işlemi için XBee yerine socket kullanılıyor.")
                data = await self.MAVSDKController.get_general_info()
                message = format_broadcast_message(data)
                try:
                    self.socket.sendto(
                        message.encode('utf-8'),
                        (SERVER_IP, SERVER_PORT)
                    )
                except Exception as e:
                    logging.error(f"Broadcast mesajı gönderilirken hata oluştu: {e}")
                logging.info(f"Güncel durum broadcast edildi: {message}")
                await asyncio.sleep(1)
                continue
            # MAVSDKController ve XBeeController'ın bağlı olup olmadığını kontrol et
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
                continue
            logging.info(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1.5)  # Her saniyede bir güncel durumu broadcast et
    
    # PID & APF Mekanizmaları
    def distance_meters(self, lat1, lon1, lat2, lon2):
        # Haversine formülü ile mesafe hesaplama
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def latlon_to_ned(self, target_lat, target_lon, current_lat, current_lon):
        # GPS koordinatlarını NED düzlemine çevir
        d_north = self.distance_meters(current_lat, current_lon, target_lat, current_lon)
        d_east = self.distance_meters(current_lat, current_lon, current_lat, target_lon)
        if target_lat < current_lat:
            d_north *= -1
        if target_lon < current_lon:
            d_east *= -1
        return d_north, d_east

    def clamp_velocity(self, v, limit=1.0):
        """
        Hızı sınırlar.
        :param v: Hız değeri
        :param limit: Sınır değeri
        :return: Sınırlanmış hız
        """
        return max(-limit, min(limit, v))

    async def apf_controller(self):
        """
        APF: Komşu dronelardan kaçınmak için hız vektörü üretir.
        """
        current_data = await self.MAVSDKController.get_general_info()
        current_position = current_data["gps_position"]

        vx, vy = self.apf.compute_apf(current_position, self.neighbors)
        return vx, vy

    async def pid_controller(self, target_position):
        """
        PID kontrolü: hedef pozisyona yönelmek için hız vektörü üretir.
        """
        current_data = await self.MAVSDKController.get_general_info()
        current_position = current_data["gps_position"]

        d_north, d_east = self.latlon_to_ned(
            target_position["latitude"], target_position["longitude"],
            current_position["latitude"], current_position["longitude"]
        )

        dt = 0.1  # Sabit güncelleme süresi
        vx = self.pid_n.compute(d_north, dt)
        vy = self.pid_e.compute(d_east, dt)
        return vx, vy

    async def background_offboard_controller(self):
        while True:
            if not self.offboardController["isActive"]:
                logging.debug("OffboardController kapalı, kontrol döngüsü atlanıyor.")
                await asyncio.sleep(0.1)
                continue

            if not await self.drone.offboard.is_active():
                try:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                    await self.drone.offboard.start()
                    logging.info("Offboard modu başlatıldı.")
                except OffboardError as e:
                    logging.error(f"Offboard moduna geçiş başarısız: {e}")
                    await asyncio.sleep(0.5)
                    continue

            target_pos = self.offboardController.get("targetPosition")
            alt_to_keep = self.offboardController.get("altitudeToKeep")

            if target_pos is None:
                logging.warning("Hedef konum ayarlanmamış. Hover moduna geçiliyor.")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.1)
                continue

            # Güncel konum bilgisi
            current_data = await self.MAVSDKController.get_general_info()
            current_position = current_data["gps_position"]

            # PID ve APF ile hızları hesapla
            pid_vx, pid_vy = await self.pid_controller(target_pos)
            apf_vx, apf_vy = await self.apf_controller()

            # İrtifa kontrolü
            error_z = alt_to_keep - current_position["altitude"]
            vz = self.pid_z.compute(error_z, 0.1)

            # Hızları birleştir ve sınırla
            vx = self.clamp_velocity(pid_vx + apf_vx)
            vy = self.clamp_velocity(pid_vy + apf_vy)
            vz = self.clamp_velocity(vz)

            try:
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=-vz, yaw_deg=0.0)
                )
            except OffboardError as e:
                logging.error(f"Hız vektörü ayarlanamadı: {e}")

            await asyncio.sleep(0.1)

    # Sık kullanılan drone işlemleri
    async def wait_for_proper_data(self):
        """
        Drone'un doğru verileri almasını bekler.
        """
        while True:
            general_info = await self.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                logging.info(f"Doğru veriler alındı.")
                break
            logging.info("Doğru veriler henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)

    async def arm(self):
        """
        Drone'u arm eder
        """
        await self.drone.action.arm()
    async def arm_check(self):
        """
        Drone'un arm durumunu kontrol eder.
        
        :return: True eğer drone arm edildiyse; aksi halde False
        """
        async for is_armed in self.drone.telemetry.armed():
            return is_armed
    
    async def wait_for_broadcast(self):
        """
        Drone'un diğer dronların broadcast mesajlarını beklediği adım fonksiyonu.
        """
        logging.info("Drone diğer dronların broadcast mesajlarını bekliyor...")
        logging.info("Diğer bir drone keşfedildi.")
    async def wait_for_broadcast_check(self):
        """
        Drone'un diğer dronların broadcast mesajlarını alıp almadığını kontrol eden fonksiyon.
        
        :return: True eğer en az bir komşu varsa; aksi halde False
        """
        if len(self.neighbors) > 0:
            logging.info(f"Komşular: {self.neighbors}")
            return True
        return False
    
    async def set_pre_takeoff_location(self):
        """
        Drone'un kalkış öncesi konumunu belirler.
        Bu, kalkış yüksekliğini hesaplamak için kullanılır.
        """
        while True:
            general_info = await self.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                self.pre_takeoff_location = gps_position
                logging.info(f"Kalkış öncesi konum ayarlandı: {self.pre_takeoff_location}")
                break
            logging.info("GPS konum bilgisi henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)
    async def pre_takeoff_location_check(self):
        """
        Drone'un kalkış öncesi konum kontrol fonksiyonu.
        Bu, drone'un kalkış yapmadan önceki konumunu kontrol eder.

        :return: True eğer drone'un kalkış öncesi konumu ayarlandıysa; aksi halde False
        """
        if self.pre_takeoff_location is not None:
            logging.info(f"Kalkış öncesi konum belirlenmesi başarılı: {self.pre_takeoff_location}")
            return True
        logging.warning("Kalkış öncesi konum henüz ayarlanmamış.")
        return False
    
    async def takeoff(self, altitude): 
        """
        Drone'a kalkış komutu gönderir
        
        :param altitude: Kalkış yüksekliği
        """
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
    async def altitude_check(self, target_altitude):
        """
        Drone'un irtifasını kontrol eden fonksiyon.
        """
        general_info = await self.MAVSDKController.get_general_info()
        gps_position = general_info["gps_position"]
        climbed = abs(gps_position["altitude"] - self.pre_takeoff_location["altitude"])
        logging.info(f"Drone hedef irtifa ile {climbed} metre mesafede.")
        if abs(target_altitude - climbed) <= 0.2:
            logging.debug(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
            return True
        return False

    async def enable_offboard_controller(self):
        logging.info("OffboardController aktifleştiriliyor...")
        self.offboardController["isActive"] = True
        asyncio.create_task(
            self.background_offboard_controller()
        )
    async def enable_offboard_controller_check(self):
        if await self.drone.offboard.is_active():
            logging.info("OffboardController etkin.")
            return True
        return False
    
    async def goto_location_with_offboard(self, target_location):
        logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
        self.offboardController["targetPosition"] = target_location
        self.offboardController["altitudeToKeep"] = target_location["altitude"]
    async def goto_location_check(self, target_location):
        general_info = await self.MAVSDKController.get_general_info()
        gps_position = general_info["gps_position"]
        logging.info(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
        if (self.distance_meters(gps_position["latitude"], gps_position["longitude"], target_location["latitude"], target_location["longitude"]) <= 0.5):
            logging.info("Drone hedef konuma ulaştı.")
            return True
        return False
    
    async def land(self): 
        """
        Drone'a iniş komutu gönderir
        """
        logging.info("Drone iniş yapıyor...")
        self.offboardController["isActive"] = False
        await self.drone.action.land()

    async def disarm_pre_check(self):
        """
        Drone'un disarm edilmeden önceki durumunu kontrol eder.
        Şartın sağlanması için drone'un havada olmaması gerekir.
        
        :return: True eğer drone havada değilse; aksi halde False
        """
        async for is_in_air in self.drone.telemetry.in_air():
            return not is_in_air
    async def disarm(self):
        """
        Drone'u disarm eder
        """
        await self.drone.action.disarm()
    async def disarm_check(self):
        """
        Drone'un disarm durumunu kontrol eder.
        
        :return: True eğer drone disarm edildiyse; aksi halde False
        """
        is_armed = await self.drone.action.is_armed()
        return not is_armed

async def main():
    """
    DroneController temel işlemlerini test eden ana fonksiyon.
    Bu fonksiyon, drone'u arm eder, kalkış yapar, belirli bir yüksekliğe çıkar, iniş yapar ve disarm eder.
    """
    isTesting = True
    xbee_port = lambda: None if isTesting else "/dev/ttyUSB0"
    mavsdk_port = lambda: "udp://0.0.0.0:14540" if isTesting else "serial:///dev/ttyACM0:57600"
    drone_controller = DroneController(
            xbee_port=xbee_port(),
            mavsdk_port=mavsdk_port(),
            isTesting=isTesting
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
        logging.error("MAVSDK henüz bağlı değil, bağlanmaya çalışılıyor...")
        await asyncio.sleep(1)
    logging.info("MAVSDK bağlı.")
    # Örnek kullanım
    await drone_controller.wait_for_proper_data()
    # Kalkış öncesi konumu ayarla
    step_controller.add_step(Step("Kalkış Öncesi Konum Ayarı", drone_controller.set_pre_takeoff_location, drone_controller.pre_takeoff_location_check))
    # Arm eder
    step_controller.add_step(Step("Arm Et", drone_controller.arm, drone_controller.arm_check))
    # Diğer dronların broadcast mesajlarını bekle
    step_controller.add_step(Step(
        "Broadcastleri bekle", 
        drone_controller.wait_for_broadcast,
        drone_controller.wait_for_broadcast_check,
        drone_controller.wait_for_broadcast_check,
        timeout=30))
    # Kalkış yapar
    target_altitude = 10  # Kalkış yüksekliği
    step_controller.add_step(Step(
                "Takeoff", 
                lambda: drone_controller.takeoff(target_altitude), 
                lambda: drone_controller.altitude_check(target_altitude)
            ))
    # OffboardController arka planda çalışır
    step_controller.add_step(Step(
        "OffboardController'ı Aktifleştir", 
        drone_controller.enable_offboard_controller, 
        drone_controller.enable_offboard_controller_check 
        ))
    # Waypoint'lere ilerler
    for i, target_location in enumerate(target_locations2):
        step_name = f"{i+1} Numaralı Hedefe İlerle"
        step_controller.add_step(
            Step(
                step_name, 
                lambda loc=target_location: drone_controller.goto_location_with_offboard(loc), 
                lambda loc=target_location: drone_controller.goto_location_check(loc)
            )
        )
        logging.info(f"{step_name} adımı eklendi.")
    # İniş yapar
    step_controller.add_step(Step("land", drone_controller.land, lambda: drone_controller.altitude_check(0)))
    # Disarm eder
    step_controller.add_step(Step("disarm", drone_controller.disarm, drone_controller.disarm_check, drone_controller.disarm_pre_check))
    logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
    await step_controller.run_steps()
    while not step_controller.is_all_done:
        logging.debug("Adımlar hala çalışıyor, bekleniyor...")
        await asyncio.sleep(1)

if __name__ == "__main__":
    logging.info("DroneController başlatıldı.")
    asyncio.run(main())
    logging.info("DroneController testinin tüm adımları tamamlandı, çıkılıyor.")
    sys.exit(0)
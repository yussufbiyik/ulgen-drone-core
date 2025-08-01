import sys 

import threading
import time
import asyncio
import numpy as np
import random
import json
import logging
import math
import socket

from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController

from utils.pid import PID
from utils.apf import APF
from utils.formation_utililties import distance_meters, latlon_to_ned, detect_pose

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
    def __init__(self, xbee_controller: XBeeController, mavsdk_controller: MAVSDKController, isTesting=False):
        self.isTesting = isTesting

        # Test modunda soket üzerinden iletişim kurmak için
        self.socket = None
        self.fake_id = random.randint(10000, 99999) if isTesting else None
        # XBee
        self.xbee_controller = xbee_controller
        self.xbee_id = self.xbee_controller.address if not self.isTesting else self.fake_id
        self.time_waited_for_other_drones = 0 
        # MAVSDK
        self.mavsdk_controller = mavsdk_controller
        self.drone = self.mavsdk_controller.drone
        
        if not self.isTesting:
            self.xbee_controller.message_received_callback = self.handle_message_received
            self.xbee_controller.listen()
            logging.info("XBee iletişimi başlatıldı.")
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            threading.Thread(target=self.listen_to_socket).start()
            logging.info("Soket iletişimi başlatıldı.")
        asyncio.create_task(self.broadcast_drone_status())
        self.pre_takeoff_location = None  # Aslında home gibi
        self.offboard_controller = {
            "is_active": False,
            "altitude_to_keep": 0.0,
            "target_position": None,
        }
        # Her eksen üzerinde kontrol sahibi olmak için
        # PID kontrolörleri eksen başına ayrı ayrı tanımlanır.
        # Yatay eksen
        self.pid_n = PID(Kp=0.1, Ki=0.0, Kd=0.1)
        self.pid_e = PID(Kp=0.1, Ki=0.0, Kd=0.1)
        # Yükseklik ekseni
        self.pid_z = PID(Kp=0.3, Ki=0.0, Kd=0.3)
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
                logging.error(f"Mesaj çözümlenemedi: {e}")

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
            logging.info(f"Komşu zaten mevcut:, güncelleniyor ({recieved_message['timestamp'] - time.time()}ms).")
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
                    self.socket.sendto(
                        message.encode('utf-8'),
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
            logging.info(f"Güncel durum broadcast edildi: {message}")
            await asyncio.sleep(1.5)  # Her saniyede bir güncel durumu broadcast et
    
    # PID & APF Mekanizmaları (192-280+ Çarpışma Önleyici)
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
        current_data = await self.mavsdk_controller.get_general_info()
        current_position = current_data["gps_position"]

        vx, vy = self.apf.compute_apf(current_position, self.neighbors)
        return vx, vy

    async def pid_controller(self, target_position):
        """
        PID kontrolü: hedef pozisyona yönelmek için hız vektörü üretir.
        """
        current_data = await self.mavsdk_controller.get_general_info()
        current_position = current_data["gps_position"]

        d_north, d_east = latlon_to_ned(
            target_position["latitude"], target_position["longitude"],
            current_position["latitude"], current_position["longitude"]
        )

        dt = 0.1  # Sabit güncelleme süresi
        vx = self.pid_n.compute(d_north, dt)
        vy = self.pid_e.compute(d_east, dt)
        return vx, vy

    async def background_offboard_controller(self):
        while True:
            if not self.offboard_controller["is_active"]:
                logging.debug("OffboardController kapalı, kontrol döngüsü atlanıyor.")
                await asyncio.sleep(0.1)
                continue

            if not await self.drone.offboard.is_active():
                try:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                    await self.drone.offboard.start()
                    logging.debug("Offboard modu başlatıldı.")
                except OffboardError as e:
                    logging.error(f"Offboard moduna geçiş başarısız: {e}")
                    await asyncio.sleep(0.5)
                    continue

            target_pos = self.offboard_controller.get("target_position")
            alt_to_keep = self.offboard_controller.get("altitude_to_keep")

            if target_pos is None:
                logging.warning("Hedef konum ayarlanmamış. Hover moduna geçiliyor.")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.1)
                continue

            # Güncel konum bilgisi
            current_data = await self.mavsdk_controller.get_general_info()
            current_position = current_data["gps_position"]

            # PID ve APF ile hızları hesapla
            pid_vx, pid_vy = await self.pid_controller(target_pos)
            apf_vx, apf_vy = await self.apf_controller()

            # İrtifa kontrolü
            error_z = alt_to_keep - current_position["altitude"]
            vz = self.pid_z.compute(error_z, 0.1)

            # Hızları birleştir ve sınırla
            vx = self.clamp_velocity(pid_vx) + apf_vx
            vy = self.clamp_velocity(pid_vy) + apf_vy
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
        Drone'un geçerli verileri almasını bekler.
        """
        while True:
            general_info = await self.mavsdk_controller.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                logging.info(f"Geçerli veriler alındı.")
                break
            logging.info("Geçerli veriler henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)

    async def arm(self):
        """
        Drone'u arm eder
        """
        await self.drone.action.arm()
    async def arm_check(self):
        """
        Drone'un arm durumunu kontrol eder.
        """
        async for is_armed in self.drone.telemetry.armed():
            return is_armed
    
    async def wait_for_broadcast(self):
        """
        Drone'un diğer dronların broadcast mesajlarını beklediği adım fonksiyonu.
        """
        logging.info("Diğer dronların broadcast mesajları bekleniyor...")
    async def wait_for_broadcast_check(self):
        """
        Drone'un diğer dronların broadcast mesajlarını alıp almadığını kontrol eden fonksiyon.
        """
        if len(self.neighbors) > 0:
            logging.info(f"Şu anda {len(self.neighbors)} tane komşu drone var.")
            logging.info("Daha başka dronların olma ihtimaline karşın biraz daha bekleniyor.")
            if self.time_waited_for_other_drones < 100:
                self.time_waited_for_other_drones += 1
                await asyncio.sleep(1)
                return False
            logging.info("Tüm dronların broadcast mesajları alındığı varsayılıyor, kontrol tamamlandı.")
            return True
        return False
    
    async def set_pre_takeoff_location(self):
        """
        Drone'un kalkış öncesi konumunu belirler.
        Bu, kalkış yüksekliğini hesaplamak için kullanılır.
        """
        while True:
            general_info = await self.mavsdk_controller.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                self.pre_takeoff_location = gps_position
                logging.debug(f"Kalkış öncesi konum ayarlandı: {self.pre_takeoff_location}")
                break
            logging.warning("GPS konum bilgisi henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)
    async def pre_takeoff_location_check(self):
        """
        Drone'un kalkış öncesi konum kontrol fonksiyonu.
        Bu, drone'un kalkış yapmadan önceki konumunu kontrol eder.
        """
        if self.pre_takeoff_location is not None:
            logging.debug(f"Kalkış öncesi konum belirlenmesi başarılı: {self.pre_takeoff_location}")
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
        general_info = await self.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        climbed = abs(gps_position["altitude"] - self.pre_takeoff_location["altitude"])
        logging.debug(f"Drone hedef irtifa ile {climbed} metre mesafede.")
        if abs(target_altitude - climbed) <= 0.2:
            logging.info(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
            return True
        return False

    async def enable_offboard_controller(self):
        logging.info("OffboardController aktifleştiriliyor...")
        self.offboard_controller["is_active"] = True
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
        self.offboard_controller["target_position"] = target_location
        self.offboard_controller["altitude_to_keep"] = target_location["altitude"]
    async def goto_location_check(self, target_location):
        general_info = await self.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        logging.debug(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
        if (distance_meters(gps_position["latitude"], gps_position["longitude"], target_location["latitude"], target_location["longitude"]) <= 0.5):
            logging.info("Drone hedef konuma ulaştı.")
            return True
        return False
    
    async def land(self): 
        """
        Drone'a iniş komutu gönderir
        """
        logging.info("Drone iniş yapıyor...")
        self.offboard_controller["is_active"] = False
        await self.drone.action.land()

    async def disarm_pre_check(self):
        """
        Drone'un disarm edilmeden önceki durumunu kontrol eder.
        Şartın sağlanması için drone'un havada olmaması gerekir.
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
        """
        is_armed = await self.drone.action.is_armed()
        return not is_armed

async def main():
    """
    DroneController temel işlemlerini test eden ana fonksiyon.
    Bu fonksiyon, drone'u arm eder, kalkış yapar, belirli bir yüksekliğe çıkar, iniş yapar ve disarm eder.
    """
    isTesting = True
    # Simülasyon ortamında hangi dronun kullanılacağını belirlemek için sim_instance değişkeni kullanılır,
    # bu değişken 0'dan başlayarak artar. Her sitl için birer arttırılır
    sim_instance = 0
    mavsdk_port = lambda: f"udp://0.0.0.0:1454{sim_instance}" if isTesting else "serial:///dev/ttyACM0:57600"
    mavsdk_controller = MAVSDKController(
        system_address=mavsdk_port(),
        port=50060+sim_instance,
    )
    xbee_port = lambda: None if isTesting else "/dev/ttyUSB0"
    xbee_controller = None
    # XBeeController test modunda None olarak ayarlanır, gerçek port kullanılmaz
    # Eğer test modunda değilsek, XBeeController'ı tanımlarız
    if not isTesting:
        xbee_controller = XBeeController(
            port=xbee_port(),
            message_received_callback=None # Başlangıçta None, daha sonra DroneController __init__ kısmında tanımlanacak
        )
    drone_controller = DroneController(
            xbee_controller,
            mavsdk_controller,
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

    await drone_controller.mavsdk_controller.connect()
    while not drone_controller.mavsdk_controller.is_connected:
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
        timeout=1000))
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
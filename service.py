import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import math
import logging
import asyncio
import argparse

# Temel Bağımlılıklar
from core.drone import Drone
from controllers.xbee_controller import XBeeController
from controllers.mavsdk_controller import MAVSDKController
from controllers.drone_controller import DroneController
from utils.formation_utilities import ned_to_latlon

# Ana Görevler
from missions.formasyon_normal import FormasyonMission as FormasyonNormal
from missions.formasyon_pid import FormasyonMission as FormasyonPID
from missions.suru_normal import SuruNavigasyonMission as SuruNavigasyonNormal
from missions.suru_pid import SuruNavigasyonMission as SuruNavigasyonPID
# Test Görevleri
from missions.tests.pid_navigasyon import UcusKanitMission

# Simülasyon ise dron numarasını al
parser = argparse.ArgumentParser(description="Ülgen Sürü IHA Takımı Dron Servisi")
parser.add_argument("--drone_id", type=int, default=0, help="Simülasyondaki dron numarası", required=False)
args = parser.parse_args()

real_mavsdk_address = "serial:///dev/ttyACM0:57600"
sim_mavsdk_address = f"udp://0.0.0.0:1454{args.drone_id}"
xbee_port = lambda num: f"/dev/ttyUSB{num}"

class DroneService:
    def __init__(self):
        self.mavsdk_controller = MAVSDKController(
            system_address=real_mavsdk_address,
            port=50060+args.drone_id
        )
        self.xbee_controller = None
        self.drone = None
        self.drone_controller = None
        self.activeMission = None

    async def connect_drone(self):
        if self.mavsdk_controller and self.xbee_controller:
            self.drone = Drone(
                mavsdk_controller=self.mavsdk_controller,
                xbee_controller=self.xbee_controller
            )
            self.drone_controller = DroneController(self.drone)
        else:
            logging.error("Drone bağlantısı kurulamadı, lütfen MAVSDK ve XBee bağlantılarını kontrol edin.")

    async def connect_mavsdk(self):
        try:
            await self.mavsdk_controller.connect()
        except Exception as e:
            logging.error(f"MAVSDK bağlantısı kurulurken hata oluştu: {e}")

    def connect_xbee(self):
        for port in range(5):
            try:
                self.xbee_controller = XBeeController(
                    port=xbee_port(port),
                    message_received_callback=self.handle_gcs_messages
                )
                logging.info(f"XBee {port} numaralı portta bulundu.")
                self.xbee_controller.listen()
                break
            except Exception as e:
                logging.exception(f"XBee {port} numaralı portta bulunamadı veya başka bir hata gerçekleşti.\nHata:{e}")
                continue

    def get_locations_from_parsed_message(self, gps_position, takeoff_altitude, parsed_message):
        location_input_type = int(parsed_message[6])  # 0:"gps" veya 1:"ned"
        locations_raw = parsed_message[7:]
        locations = []
        for index, raw_location in enumerate(locations_raw):
            if index in range(len(locations_raw)) and index+1 in range(len(locations_raw)):
                if location_input_type == 0:  # "gps"
                    lat = float(locations_raw[index])/math.pow(10, 6)
                    lon = float(locations_raw[index+1])/math.pow(10, 6)
                    locations.append({
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                    })
                else:
                    north_offset = float(locations_raw[index])
                    east_offset = float(locations_raw[index+1])
                    # Konumlar listesine eklenen ilk eleman dronun o anlık durumuna göre eklenir
                    # peşine eklenenler ise bir öncekinin konumuna göre hesaplanır
                    if len(locations) == 0: 
                        lat, lon = ned_to_latlon(north_offset, east_offset, gps_position["latitude"], gps_position["longitude"])
                        locations.append({
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                        })
                    else:
                        lat, lon = ned_to_latlon(north_offset, east_offset, locations[-1]["latitude"], locations[-1]["longitude"])
                        locations.append({
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                        })
        return locations

    async def handle_gcs_messages(self, raw_message):
        """
        Yer kontrol istasyonundan (GCS) gelen mesajları işler.
        """
        message = raw_message["data"]
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        if message == "ping":
            logging.info("GCS'den PING mesajı alındı.")
            await self.drone.mavsdk_controller.play_tune("success")
        elif message == "rst":
            logging.info("GCS'den RST (Reset) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.reboot()
            self.drone.pre_takeoff_location = gps_position
        elif message == "lnd":
            logging.info("GCS'den LND (Landing) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.land()
        elif message == "kill":
            logging.info("GCS'den KILL mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.kill()
        elif message == "home":
            logging.info("GCS'den HOME mesajı alındı.")
            await self.drone_controller.go_home()
            is_home_done = await self.drone_controller.go_home_check()
            while not is_home_done:
                is_home_done = await self.drone_controller.go_home_check()
                await asyncio.sleep(1)
            logging.info("Drone ev konumuna döndü.")
        elif message == "rb":
            logging.info("GCS'den RB (Reboot) mesajı alındı.")
        elif message == "abrt":
            logging.info("GCS'den ABRT (Abort) mesajı alındı.")
            if self.activeMission is not None and self.activeMission.status["is_running"] == True:
                self.activeMission.abort()
                self.activeMission = None
                logging.info("Aktif görev iptal edildi, drone iniş yapıyor...")
                await self.drone.mavsdk_controller.mavsdk.action.land()
        elif message.startswith("g"):
            logging.info("GCS'den görev mesajı alındı.")
            if self.activeMission is not None and self.activeMission.status["is_running"] == True:
                logging.warning("Zaten bir görev çalışıyor, yeni görev başlatılamaz.")
                return
            parsed_message = message.split(",")
            mission_id = int(parsed_message[1])
            takeoff_altitude = int(parsed_message[2])
            drone_distance = int(parsed_message[3])
            hold_seconds = int(parsed_message[4])*math.pow(10, 3)  # milisaniye cinsinden
            if mission_id == 0: # Formasyon (Normal GO-TO Versiyonu)
                formation_type = parsed_message[5]
                self.activeMission = FormasyonNormal(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    formation_duration=hold_seconds,

                    user_selected_formation_type=formation_type,
                )
            elif mission_id == 1: # Navigasyon (Normal GO-TO Versiyonu)
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message)
                formation_type = parsed_message[5]
                self.activeMission = SuruNavigasyonNormal(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    formation_duration=hold_seconds,

                    user_selected_formation_type=formation_type,
                    target_locations=mission_parameters,
                )
            elif mission_id == 2: # Birey Ekleme Çıkarma
                mission_parameters = ""
            elif mission_id == 3: # Keşif
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message)
            elif mission_id == 4: # Tek dron test görevi
                self.activeMission = UcusKanitMission(
                    drone=self.drone, drone_controller=self.drone_controller,
                    target_locations = [
                        {
                            "latitude": 47.397970,
                            "longitude": 8.546641,
                            "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                        },
                        {
                            "latitude": 47.397742,
                            "longitude": 8.546451,
                            "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                        },
                        {
                            "latitude": 47.397890,
                            "longitude": 8.546217,
                            "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                        },
                    ],
                    hold_time=hold_seconds,
                    takeoff_altitude=takeoff_altitude,
                )
            asyncio.create_task(self.activeMission.run())

async def main():
    drone_service = DroneService()
    await drone_service.connect_mavsdk()
    drone_service.connect_xbee()
    if drone_service.xbee_controller is None:
        logging.error("Hiçbir XBee cihazı bulunamadı.")
        return
    await drone_service.connect_drone()
    if drone_service.drone is None:
        logging.error("Drone bağlantısı kurulamadı.")
        return
    await drone_service.drone_controller.wait_for_proper_data()
    await drone_service.drone.mavsdk_controller.play_tune("success")
    while True:
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())
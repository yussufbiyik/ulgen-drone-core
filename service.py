import os
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
from utils.formation_utilities import ned_to_latlon, calculate_formation_weight_center

# Ana Görevler
from missions.formasyon_normal import FormasyonMission as FormasyonNormal
from missions.formasyon_pid import FormasyonMission as FormasyonPID
from missions.suru_normal import SuruNavigasyonMission as SuruNavigasyonNormal
from missions.suru_pid import SuruNavigasyonMission as SuruNavigasyonPID
from missions.birey_ekle_cikar_3_dron import BireyEkleCikar3DroneMission as BireyEkleCikarMission
from missions.birey_ekle_cikar_2_dron import BireyEkleCikar2DroneMission

# Test Görevleri
from missions.tests.pid_navigasyon import UcusKanitMission

# Simülasyon ise dron numarasını al
parser = argparse.ArgumentParser(description="Ülgen Sürü IHA Takımı Dron Servisi")
parser.add_argument("--drone_id", type=int, default=0, help="Simülasyondaki dron numarası", required=False)
parser.add_argument("--is_sim", type=int, default=False, help="Simülasyonda mı?", required=False)
args = parser.parse_args()

real_mavsdk_address = "serial:///dev/ttyACM0:57600"
sim_mavsdk_address = f"udp://0.0.0.0:1454{args.drone_id}"
xbee_port = lambda num: f"/dev/ttyUSB{num}"

class DroneService:
    def __init__(self):
        self.mavsdk_controller = MAVSDKController(
            system_address=sim_mavsdk_address if args.is_sim else real_mavsdk_address,
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
                    baudrate=57600,
                    message_received_callback=self.handle_gcs_messages
                )
                logging.info(f"XBee {port} numaralı portta bulundu.")
                self.xbee_controller.listen()
                break
            except Exception as e:
                logging.exception(f"XBee {port} numaralı portta bulunamadı veya başka bir hata gerçekleşti.\nHata:{e}")
                continue

    def get_locations_from_parsed_message(self, gps_position, takeoff_altitude, parsed_message, startIndex):
        location_input_type = int(parsed_message[startIndex])  # 0:"gps" veya 1:"ned"
        locations_raw = parsed_message[startIndex+1:]
        locations = []
        if location_input_type == 0:  # "gps"
            lat = float(locations_raw[index])/math.pow(10, 6)
            lon = float(locations_raw[index+1])/math.pow(10, 6)
            locations.append({
                "latitude": lat,
                "longitude": lon,
                "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
            })
        else:
            swarm_center = calculate_formation_weight_center(gps_position, self.drone.neighbors)
            ref_lat, ref_lon = swarm_center["latitude"], swarm_center["longitude"]
            for index in range(0, len(locations_raw), 2):
                if index + 1 >= len(locations_raw):
                    break
                north_offset = float(locations_raw[index])
                east_offset = float(locations_raw[index+1])
                lat, lon = ned_to_latlon(north_offset, east_offset, ref_lat, ref_lon)
                locations.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                })
                ref_lat, ref_lon = lat, lon  # Son konumu referans olarak güncelle
        logging.info(f"Alınan konumlar: {locations}")
        return locations

    def abortActiveMission(self):
        """
        O an çalışan görevi iptal eder.
        """
        if self.activeMission is not None and self.activeMission.status["is_running"] == True:
            self.activeMission.abort()
            self.activeMission = None

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
            self.abortActiveMission()
            await self.drone.mavsdk_controller.mavsdk.action.land()
        elif message == "kill":
            logging.info("GCS'den KILL mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.kill()
        elif message == "rtl":
            logging.info("GCS'den RTL (Return to Launch) mesajı alındı.")
            self.abortActiveMission()
            logging.info("Aktif görev iptal edildi.")
            await self.drone_controller.go_home()
            logging.info("Drone ev konumuna dönüyor.")
        elif message == "rb":
            logging.info("GCS'den RB (Reboot) mesajı alındı.")
            os.system("sudo reboot")
        elif message == "abrt":
            logging.info("GCS'den ABRT (Abort) mesajı alındı.")
            self.abortActiveMission()
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
                formation_types = parsed_message[5:]
                self.activeMission = FormasyonNormal(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    formation_duration=hold_seconds,

                    user_selected_formation_types=formation_types,
                )
            elif mission_id == 1: # Navigasyon (Normal GO-TO Versiyonu)
                formation_type = parsed_message[5]
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message, 6)
                self.activeMission = SuruNavigasyonNormal(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    formation_duration=hold_seconds,

                    user_selected_formation_type=formation_type,
                    target_locations=mission_parameters,
                )
            elif mission_id == 2: # Birey Ekleme Çıkarma (2 Dron)
                is_main_member = bool(int(parsed_message[5]))
                is_leaving_member = bool(int(parsed_message[6]))
                is_joining_member = not is_leaving_member and not is_main_member
                formation_types = parsed_message[7:9]
                print(parsed_message)
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message, 9)
                print(mission_parameters)
                self.activeMission = BireyEkleCikar2DroneMission(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    user_selected_formation_types=formation_types,
                    target_positions=mission_parameters,
                    formasyon_suresi=hold_seconds,
                    is_joining_drone=is_joining_member,
                    is_leaving_drone=is_leaving_member,
                    waypoint_durma_suresi=8,
                )
            elif mission_id == 3: # Birey Ekleme Çıkarma (3 Dron)
                formation_type = parsed_message[5]
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message, 6)
                self.activeMission = BireyEkleCikarMission(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formasyon_suresi=hold_seconds,
                    waypoint_durma_suresi=10,
                    formation_distance=drone_distance,
                    user_selected_formation_type=formation_type,
                    target_positions=mission_parameters
                )
            elif mission_id == 4: # Birey Ekleme Çıkarma (4 Dron)
                is_main_member = int(parsed_message[5])
                formation_type = parsed_message[6]
                mission_parameters = self.get_locations_from_parsed_message(gps_position, takeoff_altitude, parsed_message, 8)
                self.activeMission = BireyEkleCikarMission(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formasyon_suresi=hold_seconds,
                    waypoint_durma_suresi=10,
                    formation_distance=drone_distance,
                    user_selected_formation_type=formation_type,
                    target_positions=mission_parameters
                )
            elif mission_id == 5: # Tek dron test görevi
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
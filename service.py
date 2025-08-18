import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import math
import logging
import asyncio

from core.drone import Drone
from controllers.xbee_controller import XBeeController
from controllers.mavsdk_controller import MAVSDKController
from controllers.drone_controller import DroneController

from missions.formasyon_normal import FormasyonMission as FormasyonNormal
from missions.formasyon_pid import FormasyonMission as FormasyonPID
from missions.suru_normal import SuruNavigasyonMission as SuruNavigasyonNormal
from missions.suru_pid import SuruNavigasyonMission as SuruNavigasyonPID

from missions.tests.pid_navigasyon import UcusKanitMission

real_mavsdk_address = "serial:///dev/ttyACM0:57600"
sim_mavsdk_address = "udp://0.0.0.0:14540"
xbee_port = lambda num: f"/dev/ttyUSB{num}"

class DroneService:
    def __init__(self):
        self.mavsdk_controller = MAVSDKController(system_address=sim_mavsdk_address)
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

    def get_locations_from_parsed_message(self, parsed_message):
        location_input_type = parsed_message[5]  # "gps" veya "ned"
        locations_raw = parsed_message[6:-1]
        locations = []
        for index, raw_location in enumerate(locations_raw):
            if locations_raw[index] and locations_raw[index+1]:
                if location_input_type == "gps":
                    lat = float(locations_raw[index])/math.pow(10, 6)
                    lon = float(locations_raw[index+1])/math.pow(10, 6)
                    locations.append({
                        "latitude": lat,
                        "longitude": lon,
                    })
                else:
                    x = float(locations_raw[index])
                    y = float(locations_raw[index+1])
                    locations.append({
                        "north": x,
                        "east": y,
                    })
        return locations

    async def handle_gcs_messages(self, raw_message):
        """
        Yer kontrol istasyonundan (GCS) gelen mesajları işler.
        """
        message = raw_message["data"]
        if message == "ping":
            logging.debug("GCS'den PING mesajı alındı.")
            await self.drone.mavsdk_controller.play_tune("success")
        elif message == "rst":
            logging.info("GCS'den RST (Reset) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.reboot()
        elif message == "lnd":
            logging.debug("GCS'den LND (Landing) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.land()
        elif message == "kill":
            logging.info("GCS'den KILL mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.kill()
        elif message == "rb":
            logging.info("GCS'den RB (Reboot) mesajı alındı.")
        elif message.startswith("g"):
            logging.debug("GCS'den görev mesajı alındı.")
            if self.activeMission is not None and self.activeMission.status["is_running"] == True:
                logging.warning("Zaten bir görev çalışıyor, yeni görev başlatılamaz.")
                return
            parsed_message = message.split(",")
            mission_id = int(parsed_message[1])
            takeoff_altitude = int(parsed_message[2])
            drone_distance = int(parsed_message[3])
            hold_seconds = int(parsed_message[4])*math.pow(10, 3)  # milisaniye cinsinden
            if mission_id == 0: # Formasyon
                formation_type = parsed_message[5]
                self.activeMission = FormasyonNormal(
                    self.drone, self.drone_controller,
                    takeoff_altitude=takeoff_altitude,
                    formation_distance=drone_distance,
                    formation_duration=hold_seconds,

                    user_selected_formation_type=formation_type,
                )
                # self.activeMission = UcusKanitMission(
                #     drone=self.drone, drone_controller=self.drone_controller,
                #     target_locations = [
                #         {
                #             "latitude": 47.397970,
                #             "longitude": 8.546641,
                #             "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                #         },
                #         {
                #             "latitude": 47.397742,
                #             "longitude": 8.546451,
                #             "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                #         },
                #         {
                #             "latitude": 47.397890,
                #             "longitude": 8.546217,
                #             "altitude": self.drone.pre_takeoff_location["altitude"] + takeoff_altitude,
                #         },
                #     ],
                #     hold_time=hold_seconds,
                #     takeoff_altitude=takeoff_altitude,
                # )
            elif mission_id == 1: # Navigasyon
                mission_parameters = self.get_locations_from_parsed_message(parsed_message)
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
                mission_parameters = self.get_locations_from_parsed_message(parsed_message)
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
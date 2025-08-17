import warnings
warnings.filterwarnings("ignore", category=UserWarning)

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

real_mavsdk_address = "serial:///dev/ttyACM0:57600"
sim_mavsdk_address = "udp://0.0.0.0:14540"
xbee_port = lambda num: f"/dev/ttyUSB{num}"

class DroneService:
    def __init__(self):
        self.mavsdk_controller = MAVSDKController(system_address=sim_mavsdk_address)
        self.xbee_controller = None
        self.drone = None
        self.drone_controller = None

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

    async def handle_gcs_messages(self, raw_message):
        """
        Yer kontrol istasyonundan (GCS) gelen mesajları işler.
        """
        message = raw_message["data"]
        if message == "ping":
            logging.debug("GCS'den PING mesajı alındı.")
            await self.drone.mavsdk_controller.play_tune("success")
        elif message == "rst":
            logging.debug("GCS'den RST (Reset) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.reboot()
        elif message == "lnd":
            logging.debug("GCS'den LND (Landing) mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.land()
        elif message == "kill":
            logging.debug("GCS'den KILL mesajı alındı.")
            await self.drone.mavsdk_controller.mavsdk.action.kill()
        elif message == "rb":
            logging.debug("GCS'den RB (Reboot) mesajı alındı.")
        elif message.startswith("g"):
            logging.debug("GCS'den görev mesajı alındı.")

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
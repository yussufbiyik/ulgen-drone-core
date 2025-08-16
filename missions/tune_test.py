import sys
import logging
import asyncio

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController
from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController

from core.drone import Drone

class UcusKanitMission(Mission):
    def __init__(self, drone: Drone, drone_controller: DroneController, **kwargs):
        super().__init__("Tune test", drone, **kwargs)
        self.drone_controller = drone_controller

    async def run(self):
        # Görev modül olarak çağırıldığında
        # Dronun tüm bağlantılarının ideal olduğu varsayılır.
        logging.info("Tune test görevi başlatılıyor...")
        logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
        self.step_controller.add_step(
            Step(
                "Tune Test Adımı",
                lambda: print("test"),
                lambda: True  # Bu adımın tamamlandığını kontrol etmek için True döndür
            )
        )
        await super().run()

# Simülasyon ortamında hangi dronun kullanılacağını belirlemek için sim_instance değişkeni kullanılır,
# bu değişken 0'dan başlayarak artar. Her sitl için birer arttırılır
async def main(sim_instance=0):
    logging.basicConfig(level=logging.INFO)
    isTesting = True
    mavsdk_port = lambda: "serial:///dev/ttyACM0:57600" # f"udp://0.0.0.0:1454{sim_instance}" if isTesting else "serial:///dev/ttyACM0:57600"
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
    drone = Drone(
        mavsdk_controller=mavsdk_controller,
        xbee_controller=xbee_controller,
        isTesting=isTesting
    )
    drone_controller = DroneController(drone)
    await drone.mavsdk_controller.connect()
    while not drone.mavsdk_controller.is_connected:
        logging.info("Drone bağlantısı kuruluyor...")
        await asyncio.sleep(1)
    logging.info("Drone bağlantısı kuruldu.")
    # await drone_controller.wait_for_proper_data()
    mission = UcusKanitMission(drone, drone_controller)
    await mission.run()
    drone.mavsdk_controller.disconnect()
    sys.exit(0)
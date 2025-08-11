import math
import sys
import logging
import asyncio
import time

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController
from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController

from core.drone import Drone

time_waited = 0
async def sleep_for(milliseconds):
    """
    Asenkron olarak belirtilen milisaniye kadar bekler.
    """
    global time_waited
    time_waited = time.time()
async def sleep_for_check(milliseconds):
    """
    Asenkron olarak belirtilen milisaniye kadar beklenip beklenilmediğini kontrol eder.
    """
    global time_waited
    current_time = time.time()
    if current_time - time_waited >= milliseconds / 1000:
        return True
    else:
        return False

async def print_message(message):
    """
    Mesajı yazdırır.
    ( neden lazım diye sormayın {-_-} )
    """
    logging.info(message)

class FormasyonMission(Mission):
    def __init__(self, drone: Drone, drone_controller: DroneController, **kwargs):
        super().__init__("Formasyon Navigasyon", drone, **kwargs)
        self.drone_controller = drone_controller
        self.step_controller.wait_for_neighbors = True
        self.drone.mavsdk_controller.mavsdk.action.set_current_speed(self.drone.speed_limit)

    async def run(self):
        # Görev modül olarak çağırıldığında
        # Dronun tüm bağlantılarının ideal olduğu varsayılır.
        # Parametreleri Al
        user_selected_formation_type = self.parameters.get("user_selected_formation_type", "v")
        formation_distance = self.parameters.get("formation_distance", 10.0)
        formasyon_suresi = self.parameters.get("formasyon_suresi", 100.0)
        takeoff_altitude = self.parameters.get("takeoff_altitude", 10.0) + self.drone.pre_takeoff_location["altitude"]

        logging.info("Formasyon navigasyon görevi başlatılıyor...")
        # Diğer dronlardan broadcast bekle
        self.step_controller.add_step(Step("Diğer Dronlardan Broadcast Bekle", self.drone_controller.wait_for_broadcast, lambda: self.drone_controller.wait_for_broadcast_check(2)))
        # Arm et
        self.step_controller.add_step(Step("Arm Et", self.drone_controller.arm, self.drone_controller.arm_check))
        # Kalkış öncesi konumu ayarla
        self.step_controller.add_step(Step("Kalkış Öncesi Konumu Ayarla", self.drone_controller.set_pre_takeoff_location, self.drone_controller.pre_takeoff_location_check))
        # Takeoff yap
        self.step_controller.add_step(
            Step("Takeoff",
                 lambda: self.drone_controller.takeoff(takeoff_altitude),
                 lambda: self.drone_controller.altitude_check(takeoff_altitude)
                )
            )
        formation_step = lambda formation_name, distance: Step(
                "Formasyona Gir",
                lambda: self.drone_controller.goto_formation_location(formation_name, distance),
                lambda: self.drone_controller.goto_formation_location_check()
            )
        formation_hold_step = lambda hold_time: Step(
                "Formasyonda Kal",
                lambda: sleep_for(hold_time),
                lambda: sleep_for_check(hold_time)
            )
        # Formasyona gir
        self.drone.formation_type = user_selected_formation_type
        self.drone.formation_distance = formation_distance
        self.step_controller.add_step(formation_step(user_selected_formation_type, formation_distance))
        # Bir süre formasyonda kalarak çıkış emrini bekle
        self.step_controller.add_step(formation_hold_step(formasyon_suresi))
        # Land yap
        self.step_controller.add_step(Step("Land", self.drone_controller.land, lambda alt=0: self.drone_controller.altitude_check(alt)))
        # Disarm et
        self.step_controller.add_step(Step("Disarm Et", self.drone_controller.disarm, self.drone_controller.disarm_check, self.drone_controller.disarm_pre_check))
        logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
        await super().run()

# Simülasyon ortamında hangi dronun kullanılacağını belirlemek için sim_instance değişkeni kullanılır,
# bu değişken 0'dan başlayarak artar. Her sitl için birer arttırılır
async def main(sim_instance=0):
    logging.basicConfig(level=logging.INFO)
    isTesting = True
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
            message_received_callback=None
        )
    drone = Drone(
        mavsdk_controller=mavsdk_controller,
        xbee_controller=xbee_controller,
        isTesting=isTesting
    )
    drone.waypoint_threshold = 0.5
    drone_controller = DroneController(drone)
    takeoff_altitude = 10.0
    await drone.mavsdk_controller.connect()
    while not drone.mavsdk_controller.is_connected:
        logging.info("Drone bağlantısı kuruluyor...")
        await asyncio.sleep(1)
    logging.info("Drone bağlantısı kuruldu.")
    await drone_controller.wait_for_proper_data()
    mission = FormasyonMission(drone, drone_controller, 
                                takeoff_altitude=takeoff_altitude,
                                user_selected_formation_type="v",
                                formation_distance=10.0,
                                formasyon_suresi=5000.0,
                            )
    await mission.run()
    drone.mavsdk_controller.disconnect()
    sys.exit(0)
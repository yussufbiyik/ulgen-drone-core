import sys
import time
import logging
import asyncio

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
    
class UcusKanitMission(Mission):
    def __init__(self, drone: Drone, drone_controller: DroneController, **kwargs):
        super().__init__("PID Uçuş Kanıt", drone, **kwargs)
        self.drone_controller = drone_controller

    async def run(self):
        # Görev modül olarak çağırıldığında
        takeoff_altitude = self.parameters.get("takeoff_altitude", 10.0)
        hold_time = self.parameters.get("hold_time", 100.0)
        self.drone.altitude_target = takeoff_altitude  # Dronun irtifa hedefini kalkış irtifasına ayarla
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
        # OffboardController'ı aktifleştir
        self.step_controller.add_step(
            Step("Offboard Moda Geç",
                self.drone_controller.enable_offboard_controller, 
                self.drone_controller.enable_offboard_controller_check
            )
        )
        # Hedef noktalara ilerle
        for i, target_location in enumerate(self.parameters.get("target_locations", [])):
            step_name = f"{i+1} Numaralı Hedefe İlerle"
            self.step_controller.add_step(
                Step(
                    step_name, 
                    lambda loc=target_location: self.drone_controller.goto_location_with_offboard(loc), 
                    lambda loc=target_location: self.drone_controller.goto_location_check(loc)
                )
            )
            self.step_controller.add_step(
                Step(
                    "Konumda Kal",
                    lambda: sleep_for(hold_time),
                    lambda: sleep_for_check(hold_time)
                )
            )
            logging.info(f"{step_name} adımı eklendi.")
        self.step_controller.add_step(
                Step("Offboard Modu Kapat",
                    self.drone_controller.disable_offboard_controller,
                    self.drone_controller.disable_offboard_controller_check
                )
            )
        # Hedef konumlara ulaşıldıktan sonra zemine in
        # Kontrol fonksiyonu olarak altitude_check fonksiyonu kullanılabilir
        self.step_controller.add_step(Step("Land", self.drone_controller.land,
                    lambda alt=0: self.drone_controller.altitude_check(alt)
                ))
        # Disarm et
        self.step_controller.add_step(Step("Disarm Et", self.drone_controller.disarm, self.drone_controller.disarm_check, self.drone_controller.disarm_pre_check))
        logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
        await super().run()

# Simülasyon ortamında hangi dronun kullanılacağını belirlemek için sim_instance değişkeni kullanılır,
# bu değişken 0'dan başlayarak artar. Her sitl için birer arttırılır
async def main(sim_instance=0):
    logging.basicConfig(level=logging.INFO)
    isTesting = False
    mavsdk_port = lambda: f"udp://0.0.0.0:1454{sim_instance}" # if isTesting else "serial:///dev/ttyACM0:57600"
    mavsdk_controller = MAVSDKController(
        system_address=mavsdk_port(),
        port=50060+sim_instance,
    )
    xbee_port = "/dev/ttyUSB0"
    xbee_controller = None
    # XBeeController test modunda None olarak ayarlanır, gerçek port kullanılmaz
    # Eğer test modunda değilsek, XBeeController'ı tanımlarız
    if not isTesting:
        xbee_controller = XBeeController(
            port=xbee_port,
            message_received_callback=None # Başlangıçta None, daha sonra DroneController __init__ kısmında tanımlanacak
        )
    drone = Drone(
        mavsdk_controller=mavsdk_controller,
        xbee_controller=xbee_controller,
        isTesting=isTesting
    )
    drone_controller = DroneController(drone)
    takeoff_altitude = 10.0
    target_locations1 = [
        {
            "latitude": 40.326037, 
            "longitude": 36.473655,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
        {
            "latitude": 40.325634,
            "longitude": 36.473806,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
        {
            "latitude": 40.325428,
            "longitude": 36.473451,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
    ]
    target_locations2 = [
        {
            "latitude": 47.397970,
            "longitude": 8.546641,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
        {
            "latitude": 47.397742,
            "longitude": 8.546451,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
        {
            "latitude": 47.397890,
            "longitude": 8.546217,
            "altitude": drone.pre_takeoff_location["altitude"]+takeoff_altitude,
        },
    ]
    await drone.mavsdk_controller.connect()
    while not drone.mavsdk_controller.is_connected:
        logging.info("Drone bağlantısı kuruluyor...")
        await asyncio.sleep(1)
    logging.info("Drone bağlantısı kuruldu.")
    await drone_controller.wait_for_proper_data()
    mission = UcusKanitMission(drone, drone_controller, takeoff_altitude=takeoff_altitude, target_locations=target_locations2, hold_time=10000)
    await mission.run()
    drone.mavsdk_controller.disconnect()
    sys.exit(0)
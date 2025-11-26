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

class BireyEkleCikar2DroneMission(Mission):
    def __init__(self, drone: Drone, drone_controller: DroneController, **kwargs):
        super().__init__("2 Dron ile Birey Ekle Çıkar", drone, **kwargs)
        self.drone_controller = drone_controller
        self.step_controller.wait_for_neighbors = True

    async def broadcast_self_formation_state_check(self):
        """
        Drone'un kendi formasyon durumunu diğer drone'lara bildirdiği kontrol fonksiyonu.
        """
        await self.drone.send_message_with_ack("mf0")
        return True
    
    async def send_self_waypoint_position(self):
        """
        Drone'un kendi waypoint konumunu diğer bir drona bildirdiği kontrol fonksiyonu.
        """
        general_info = await self.drone.mavsdk_controller.get_general_info()
        gps_position = general_info["gps_position"]
        position_string = f"mt,{gps_position['latitude']:.6f},{gps_position['longitude']:.6f}".replace('.', '')
        joining_drone = self.drone.inactive_neighbors[0]["sender"]
        # self.drone.send_private_message(joining_drone, position_string)
        await self.drone.send_message_with_ack(position_string)
        return True

    async def return_true(self):
        return True

    async def run(self):
        # Parametreleri Al
        user_selected_formation_types = self.parameters.get("user_selected_formation_types", [])
        print(user_selected_formation_types)
        formation_distance = self.parameters.get("formation_distance", 12.0)
        formasyon_suresi = self.parameters.get("formasyon_suresi", 4000.0)
        waypoint_durma_suresi = self.parameters.get("waypoint_durma_suresi", 8000.0)
        takeoff_altitude = self.parameters.get("takeoff_altitude", 10.0)
        target_positions = self.parameters.get("target_positions", [])
        is_joining_drone = self.parameters.get("is_joining_drone", False)
        is_leaving_drone = self.parameters.get("is_leaving_drone", False)
        print(is_leaving_drone, is_joining_drone)
        self.drone.altitude_target = takeoff_altitude  # Dronun irtifa hedefini kalkış irtifasına ayarla
        health = await self.drone.mavsdk_controller.mavsdk.telemetry.health().__anext__()
        is_armable = health.is_armable
        if not is_armable:
            self.step_controller.abort_steps()
        self.step_controller.add_step(Step(
            "Diğer Dronlardan Broadcast Bekle",
            self.drone_controller.wait_for_broadcast, lambda: self.drone_controller.wait_for_broadcast_check(2)
        ))
        self.step_controller.add_step(Step(
            "Sürüde Olma Durumunu Bildir",
            lambda: print_message("Sürüde olma durumu bildiriliyor..."), lambda: self.broadcast_self_formation_state_check(),
            should_skip=not is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Kalkış Öncesi Konumu Ayarla",
            self.drone_controller.set_pre_takeoff_location, self.drone_controller.pre_takeoff_location_check
        ))
        self.step_controller.add_step(Step(
            "Arm",
            self.drone_controller.arm, self.drone_controller.arm_check,
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Takeoff",
            lambda: self.drone_controller.takeoff(takeoff_altitude), lambda: self.drone_controller.altitude_check(takeoff_altitude),
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Formasyona Gir",
            lambda: self.drone_controller.goto_formation_location(user_selected_formation_types[0], formation_distance),
            lambda: self.drone_controller.goto_formation_location_check(),
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Formasyonda Bekle",
            lambda: sleep_for(formasyon_suresi),
            lambda: sleep_for_check(formasyon_suresi),
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "1. Hedef Konuma İlerle",
            lambda loc=target_positions[0]: self.drone_controller.goto_location_with_formation(loc, False), 
            lambda loc=target_positions[0]: self.drone_controller.goto_location_with_formation_check(loc),
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "2. Formasyona Gir",
            lambda: self.drone_controller.goto_formation_location(user_selected_formation_types[1], formation_distance),
            lambda: self.drone_controller.goto_formation_location_check(),
            should_skip=is_joining_drone
        ))
        # self.step_controller.add_step(Step(
        #     "1. Hedef Konumda Bekle",
        #     lambda: sleep_for(waypoint_durma_suresi),
        #     lambda: sleep_for_check(waypoint_durma_suresi),
        #     should_skip=is_joining_drone
        # ))
        self.step_controller.add_step(Step(
            "2. Hedef Konuma İlerle",
            lambda loc=target_positions[1]: self.drone_controller.goto_location_with_formation(loc, False), 
            lambda loc=target_positions[1]: self.drone_controller.goto_location_with_formation_check(loc),
            should_skip=is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Katılacak drona konumu bildir",
            self.send_self_waypoint_position,
            self.return_true,
            should_skip=not is_leaving_drone
        ))
        self.step_controller.add_step(Step(
            "Ayrılacak Dronun İnişini Bekle",
            lambda: self.drone_controller.leave_formation(is_leaving_drone),
            lambda: self.drone_controller.leave_formation_check(is_leaving_drone, is_joining_drone)
        ))
        self.step_controller.add_step(Step(
            "Arm",
            self.drone_controller.arm, self.drone_controller.arm_check,
            should_skip=not is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Takeoff",
            lambda: self.drone_controller.takeoff(takeoff_altitude), lambda: self.drone_controller.altitude_check(takeoff_altitude),
            should_skip=not is_joining_drone
        ))
        self.step_controller.add_step(Step(
            "Katılacak Dronun Gelişini Bekle",
            lambda: self.drone_controller.join_formation(is_joining_drone),
            lambda: self.drone_controller.join_formation_check(is_joining_drone, is_leaving_drone),
            should_skip=is_leaving_drone,
        ))
        # self.step_controller.add_step(Step(
        #     "2. Hedef Konumda Bekle",
        #     lambda: sleep_for(waypoint_durma_suresi),
        #     lambda: sleep_for_check(waypoint_durma_suresi),
        #     should_skip=is_leaving_drone
        # ))
        self.step_controller.add_step(Step(
            "3. Hedef Konuma İlerle",
            lambda loc=target_positions[2]: self.drone_controller.goto_location_with_formation(loc, False), 
            lambda loc=target_positions[2]: self.drone_controller.goto_location_with_formation_check(loc),
            should_skip=is_leaving_drone
        ))
        self.step_controller.add_step(Step(
            "İniş Yap",
            self.drone_controller.land, lambda alt=0: self.drone_controller.altitude_check(alt),
            should_skip=is_leaving_drone
        ))
        self.step_controller.add_step(Step(
            "Disarm Et",
            self.drone_controller.disarm, self.drone_controller.disarm_check, self.drone_controller.disarm_pre_check
        ))
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
    xbee_port = lambda: "/dev/ttyUSB0"
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
    await drone.mavsdk_controller.connect()
    while not drone.mavsdk_controller.is_connected:
        logging.info("Drone bağlantısı kuruluyor...")
        await asyncio.sleep(1)
    logging.info("Drone bağlantısı kuruldu.")
    await drone_controller.wait_for_proper_data()
    mission = BireyEkleCikar2DroneMission(
            drone, 
            drone_controller, 
            takeoff_altitude=5.0, 
            user_selected_formation_types=["cizgi"], 
            formation_distance=10.0, 
            formation_duration=5000,
            is_main_group_drone=True
        )
    await mission.run()
    drone.mavsdk_controller.disconnect()
    sys.exit(0)
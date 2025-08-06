import sys
import logging
import asyncio

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController
from controllers.mavsdk_controller import MAVSDKController
from controllers.xbee_controller import XBeeController

from core.drone import Drone

def pick_formation():
    """
    Kullanıcıdan formasyon tipini seçmesini ister.
    """
    formation_type = input("Formasyon tipi (v/cizgi/ok): ").strip().lower()
    if formation_type not in ["v", "cizgi", "ok"]:
        raise ValueError("Geçersiz formasyon tipi. Lütfen 'v', 'cizgi' veya 'ok' girin.")
    return formation_type

class FormasyonMission(Mission):
    def __init__(self, drone: Drone, drone_controller: DroneController, **kwargs):
        super().__init__("Formasyon", drone, **kwargs)
        self.drone_controller = drone_controller

    async def run(self):
        # Görev modül olarak çağırıldığında
        # Dronun tüm bağlantılarının ideal olduğu varsayılır.
        logging.info("Formasyon görevi başlatılıyor...")
        # Diğer dronlardan broadcast bekle
        self.step_controller.add_step(Step("Diğer Dronlardan Broadcast Bekle", self.drone_controller.wait_for_broadcast, self.drone_controller.wait_for_broadcast_check))
        # Arm et
        self.step_controller.add_step(Step("Arm Et", self.drone_controller.arm, self.drone_controller.arm_check))
        # Kalkış öncesi konumu ayarla
        self.step_controller.add_step(Step("Kalkış Öncesi Konumu Ayarla", self.drone_controller.set_pre_takeoff_location, self.drone_controller.pre_takeoff_location_check))
        # Takeoff yap
        takeoff_altitude = self.parameters.get("takeoff_altitude", 10.0)
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
                self.drone_controller.enable_offboard_controller_check)
            )
        # Formasyon pozisyonunu hesapla
        formation_type = self.parameters.get("formation_type", "v")
        formation_distance = self.parameters.get("formation_distance", 5.0)
        # Formasyona gir
        self.step_controller.add_step(
            Step(
                "Formasyona Gir",
                lambda: self.drone_controller.goto_formation_location_with_offboard(formation_type, formation_distance),
                lambda: self.drone_controller.goto_formation_location_check()
            )
        )
        # Bir süre formasyonda kal
        formasyon_suresi = self.parameters.get("formasyon_suresi", 100.0)
        self.step_controller.add_step(
            Step(
                "Formasyonda Kal",
                lambda: asyncio.sleep(formasyon_suresi),
                lambda: True  # Formasyonda kalma kontrolü, her zaman True döner
            )
        )
        # Diğer formasyonlara geçiş için kullanıcı girdisi al
        #! Daha ayarlanmadı!!
        # Hedef konumlara ulaşıldıktan sonra zemine in
        # Kontrol fonksiyonu olarak altitude_check fonksiyonu kullanılabilir
        self.step_controller.add_step(Step("Land", self.drone_controller.land,
                    lambda alt=0: self.drone_controller.altitude_check(alt)
                ))
        # Disarm et
        self.step_controller.add_step(Step("Disarm Et", self.drone_controller.disarm, self.drone_controller.disarm_check, self.drone_controller.disarm_pre_check))
        logging.info("Adımlar eklendi, adımlar çalıştırılıyor...")
        await super().run()

async def main():
    logging.basicConfig(level=logging.INFO)
    isTesting = True
    # Simülasyon ortamında hangi dronun kullanılacağını belirlemek için sim_instance değişkeni kullanılır,
    # bu değişken 0'dan başlayarak artar. Her sitl için birer arttırılır
    sim_instance = 2
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
    await drone_controller.wait_for_proper_data()
    mission = FormasyonMission(drone, drone_controller, takeoff_altitude=5.0, formation_type="v", formation_distance=5.0, formation_duration=60)
    await mission.run()
    drone.mavsdk_controller.disconnect()
    sys.exit(0)
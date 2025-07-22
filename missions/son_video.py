import logging
import asyncio

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController

class KTRVideoMission(Mission):
    def __init__(self, drone: DroneController, **kwargs):
        super().__init__("KTR Video", drone, **kwargs)

    async def run(self):
        # Görev modül olarak çağırıldığında
        # DroneController'ın tüm bağlantılarının ideal olduğu varsayılır.
        logging.info("KTR Video görevi başlatılıyor...")
        # Arm et
        self.step_controller.add_step(Step("Arm Et", self.drone_controller.arm, self.drone_controller.arm_check))
        # Kalkış öncesi konumu ayarla
        self.step_controller.add_step(Step("Kalkış Öncesi Konum Ayarı", self.drone_controller.set_pre_takeoff_location, self.drone_controller.pre_takeoff_location_check))
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
            logging.info(f"{step_name} adımı eklendi.")
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
    isTesting = True
    xbee_port = lambda: None if isTesting else "/dev/ttyUSB0"
    mavsdk_port = lambda: "udp://0.0.0.0:14540" if isTesting else "serial:///dev/ttyACM0:57600"
    drone_controller = DroneController(
            xbee_port=xbee_port(),
            mavsdk_port=mavsdk_port(),
            isTesting=isTesting,
        )
    await drone_controller.MAVSDKController.connect()
    while not drone_controller.MAVSDKController.is_connected:
        logging.info("Drone bağlantısı kuruluyor...")
        await asyncio.sleep(1)
    logging.info("Drone bağlantısı kuruldu.")
    await drone_controller.wait_for_proper_data()
    mission = KTRVideoMission(drone_controller, takeoff_altitude=10.0, target_locations=target_locations2)
    await mission.run()
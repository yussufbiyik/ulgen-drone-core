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
        pre_takeoff_location = None # Aslında home gibi
        while True:
            general_info = await self.drone.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                pre_takeoff_location = gps_position
                break
            logging.info("GPS yükseklik bilgisi henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)
        # Arm et
        async def arm():
            logging.info("Drone arm ediliyor...")
            await self.drone.arm()
        async def arm_check():
            async for is_armed in self.drone.telemetry.armed():
                return is_armed
        self.step_controller.add_step(Step("Arm", arm, arm_check))
        # Takeoff yap
        takeoff_altitude = self.parameters.get("takeoff_altitude", 10.0)
        async def takeoff():
            logging.info("Drone kalkış yapıyor...")
            await self.drone.takeoff(takeoff_altitude)
        async def altitude_check(target_altitude=takeoff_altitude):
            """
            Drone'un kalkış durumunu kontrol eden fonksiyon.
            """
            general_info = await self.drone.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            climbed = abs(gps_position["altitude"] - pre_takeoff_location["altitude"])
            logging.info(f"Drone hedef irtifa ile {climbed} metre mesafede.")
            if abs(target_altitude - climbed) <= 0.2:
                logging.debug(f"Drone {target_altitude} metreye yeterince yakınlaştı.")
                return True
        self.step_controller.add_step(Step("Takeoff",takeoff,altitude_check))
        # OffboardController'ı aktifleştir
        async def enable_offboard_controller():
            logging.info("OffboardController aktifleştiriliyor...")
            self.drone.offboardController["isActive"] = True
            asyncio.create_task(
                self.drone.background_offboard_controller()
            )
        async def enable_offboard_controller_check():
            if await self.drone.drone.offboard.is_active():
                logging.info("OffboardController etkin.")
                return True
            return False
        self.step_controller.add_step(Step("Offboard Mod", enable_offboard_controller, enable_offboard_controller_check))
        # Hedef noktalara ilerle
        async def goto_location(target_location):
            logging.info(f"Drone {target_location['latitude']}, {target_location['longitude']}, {target_location['altitude']} konumuna gidiyor...")
            self.drone.offboardController["targetPosition"] = target_location
            self.drone.offboardController["altitudeToKeep"] = target_location["altitude"]
        async def goto_location_check(target_location):
            general_info = await self.drone.MAVSDKController.get_general_info()
            gps_position = general_info["gps_position"]
            logging.info(f"Drone konumu: {gps_position['latitude']}, {gps_position['longitude']}, {gps_position['altitude']}")
            if (self.drone.distance_meters(gps_position["latitude"], gps_position["longitude"], target_location["latitude"], target_location["longitude"]) <= 0.5):
                logging.info("Drone hedef konuma ulaştı.")
                return True
            return False
        for i, target_location in enumerate(self.parameters.get("target_locations", [])):
            step_name = f"{i+1} Numaralı Hedefe İlerle"
            self.step_controller.add_step(
                Step(
                    step_name, 
                    lambda loc=target_location: goto_location(loc), 
                    lambda loc=target_location: goto_location_check(loc)
                )
            )
            logging.info(f"{step_name} adımı eklendi.")
        # Hedef konumlara ulaşıldıktan sonra zemine in
        async def land():
            """
            Drone'u inişe hazırlayan adım fonksiyonu.
            """
            logging.info("Drone iniş yapıyor...")
            self.drone.offboardController["isActive"] = False
            await self.drone.land()
        # Kontrol fonksiyonu olarak altitude_check fonksiyonu kullanılabilir
        self.step_controller.add_step(Step("Land", land,
                    lambda alt=0: altitude_check(alt)
                ))
        # Disarm et
        async def disarm_pre_check():
            async for is_in_air in self.drone.MAVSDKController.drone.telemetry.in_air():
                return not is_in_air
        async def disarm():
            logging.info("Drone disarm ediliyor...")
            await self.drone.disarm()
        async def disarm_check():
            is_armed = await self.drone.MAVSDKController.is_armed()
            if not is_armed:
                return True
            return False
        self.step_controller.add_step(Step("Disarm", disarm, disarm_check, disarm_pre_check))
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
    mission = KTRVideoMission(drone_controller, takeoff_altitude=10.0, target_locations=target_locations2)
    await mission.run()
import os
import time
import logging
import asyncio
import functools
from mavsdk import System

log_name = f"./logs/MAVSDKController_{int(time.time()*1000)}.log"
os.makedirs("./logs", exist_ok=True)
logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
        handlers=[
            logging.FileHandler(log_name),
            logging.StreamHandler()
        ]
    )

def check_connected(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if not self.is_connected:
            logging.error("Drone ile bağlantı yok.")
            return None
        return await func(self, *args, **kwargs)
    return wrapper

class MAVSDKController:
    def __init__(self, system_address="udpin://0.0.0.0:14540", telemetry_timeout=5, connection_timeout=1):
        self.drone = System()
        self.connection_url = system_address
        self.telemetry_timeout = telemetry_timeout
        self.connection_timeout = connection_timeout
        self.is_connected = False
    
    async def wait_for_connection(self):
        """
        Drone'un sistem bağlantısını bekler ve doğrular.
        """
        start_time = time.time()
        while time.time() - start_time < self.connection_timeout:
            try:
                state = await self.drone.core.connection_state().observe(timeout=0.5)
                if state.is_connected:
                    logging.info("Drone sistem bağlantısı doğrulandı.")
                    return True
                break
            except Exception as e:
                logging.debug(f"Bağlantı kontrolü: {e}")
            await asyncio.sleep(1)
        
        logging.error("Sistem bağlantısı zaman aşımı.")
        return False
    
    async def connect(self):
        """
        Drone ile bağlantı kurar.
        """
        try:
            await self.drone.connect(system_address=self.connection_url)
            if await self.wait_for_connection():
                self.is_connected = True
                logging.info(f"{self.connection_url} adresine bağlanıldı.")
            else:
                logging.error("Drone bağlantısı kurulamıyor.")
                self.is_connected = False
        except Exception as e:
            self.is_connected = False
            logging.error(f"Bağlanmaya çalışırken hata: {e}")
    
    @check_connected
    async def get_all(self):
        """
        Drone'un gerekli tüm bilgilerini döndürür.
        """
        try:
            results = await asyncio.gather(
                self.get_flight_mode(),
                self.get_health(),
                self.get_remaining_battery(),
                self.get_gps_info(),
                self.get_gps_position(),
                self.get_attitude(),
                self.get_velocity()
            )

            mode, health, battery, gps_info, gps_position, attitude, velocity = results
            
            return {
                "flight_mode": mode,
                "health": health,
                "battery": battery,
                "gps_info": gps_info,
                "gps_position": gps_position,
                "attitude": attitude,
                "velocity": velocity
            }
        except Exception as e:
            logging.error(f"Tüm telemetri bilgileri alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_health(self):
        """
        Drone'un kalibrasyon vb. bilgilerini döndürür.
        """
        try:
            health = await self.drone.telemetry.health().observe(timeout=0.5)
            return {
                "kalibrasyon": {
                    "manyetik": health.is_magnetometer_calibration_ok,
                    "ivmeölçer": health.is_accelerometer_calibration_ok,
                    "jiroskop": health.is_gyrometer_calibration_ok,
                    "gps": health.is_global_position_ok
                },
                "sistem": {
                    "armable": health.is_armable,
                }
            }
        except Exception as e:
            logging.error(f"Sağlık bilgisi alınırken hata: {e}")
            return None
        
    @check_connected
    async def is_armed(self):
        """
        Drone'un arm durumunu döndürür.
        """
        try:
            is_armed = await self.drone.telemetry.armed().observe(timeout=0.5)
            return is_armed
        except asyncio.TimeoutError:
            logging.error("Drone arm durumu alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"Drone arm durumu alınırken hata: {e}")
            return None
        
    @check_connected
    async def get_flight_mode(self):
        """
        Drone'un durumunu döndürür.
        """
        try:
            mode = await self.drone.telemetry.flight_mode().observe(timeout=0.5)
            return mode
        except asyncio.TimeoutError:
            logging.error("Drone durumu alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"Drone durumu alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_remaining_battery(self):
        """
        Drone'un kalan bataryasını döndürür.
        """
        try:
            battery = await self.drone.telemetry.battery().observe(timeout=0.5)
            return battery.remaining_percent
        except asyncio.TimeoutError:
            logging.error("Batarya bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"Batarya bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_gps_info(self):
        """
        Drone'un GPS bilgilerini döndürür.
        """
        try:
            gps_info = await self.drone.telemetry.gps_info().observe(timeout=0.5)
            return {
                        "Bağlı Uydular":gps_info.num_satellites, 
                        "Fix Type":gps_info.fix_type
                    }
        except asyncio.TimeoutError:
            logging.error("GPS bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"GPS bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_gps_position(self):
        """
        Drone'un GPS konumunu döndürür.
        """
        try:
            position = await self.drone.telemetry.position().observe(timeout=0.5)
            return {
                "latitude": position.latitude_deg,
                "longitude": position.longitude_deg,
                "altitude": position.absolute_altitude_m
            }
        except asyncio.TimeoutError:
            logging.error("GPS konum bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"GPS konum bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_attitude(self):
        """
        Drone'un yön bilgilerini döndürür (yaw, pitch, roll).
        """
        try:
            attitude_euler = await self.drone.telemetry.attitude_euler().observe(timeout=0.5)
            return {
                "yaw": attitude_euler.yaw_deg,
                "pitch": attitude_euler.pitch_deg,
                "roll": attitude_euler.roll_deg
            }
        except asyncio.TimeoutError:
            logging.error("Yön bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"Yön bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_velocity(self):
        """
        Drone'un anlık hız bilgisini döndürür.
        """
        try:
            velocity = await self.drone.telemetry.velocity_ned().observe(timeout=0.5)
            return {
                "north": velocity.north_m_s,
                "east": velocity.east_m_s,
                "down": velocity.down_m_s
            }
        except asyncio.TimeoutError:
            logging.error("Anlık hız bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"Anlık hız bilgisi alınırken hata: {e}")
            return None
    
    def disconnect(self):
        """
        Drone ile bağlantıyı keser.
        """
        if self.is_connected:
            self.drone = System()
            self.is_connected = False
            logging.info("Drone bağlantısı kesildi.")
        else:
            logging.warning("Drone zaten bağlı değil.")

async def main():
    controller = MAVSDKController(system_address="serial:///dev/ttyUSB0:57600")
    await controller.connect()
    if controller.is_connected:
        await asyncio.sleep(0.5)
        all_info = await controller.get_all()
        logging.info(f"Tüm bilgiler:\n\t{all_info}")
        controller.disconnect()
    else:
        logging.error("Drone ile bağlantı kurulamadı.")

if __name__ == "__main__":
    asyncio.run(main())
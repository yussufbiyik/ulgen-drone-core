import os
import time
import json
import logging
import asyncio
import functools
import mavsdk
from mavsdk import System

def check_connected(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if not self.is_connected:
            raise ConnectionError("Drone ile bağlantı kurulmadı. Lütfen önce connect() metodunu çağırın.")
        return await func(self, *args, **kwargs)
    return wrapper

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

class MAVSDKController:
    def __init__(self, system_address="udpin://0.0.0.0:14540", port=50050, connection_timeout=100):
        self.drone = System(
            port=port
        )
        self.connection_url = system_address
        self.connection_timeout = connection_timeout
        self.is_connected = False
        self.general_status = {
            "health": None,
            "armed": None,
            "flight_mode": None,
            "battery": None,
            "gps_info": None,
            "gps_position": None,
            "attitude": None,
            "velocity": None,
        }
    
    async def wait_for_connection(self):
        """
        Drone'un sistem bağlantısını bekler ve doğrular.
        """
        start_time = time.time()*1000
        while time.time()*1000 - start_time < self.connection_timeout:
            try:
                async for state in self.drone.core.connection_state():
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
            await self.drone.connect(
                system_address=self.connection_url)
            if await self.wait_for_connection():
                self.is_connected = True
                logging.info(f"{self.connection_url} adresine bağlanıldı.")
                # Arka planda tüm parametreleri almayı başlat
                asyncio.create_task(self.update_general_info())
            else:
                logging.error("Drone bağlantısı kurulamıyor.")
                self.is_connected = False
        except Exception as e:
            self.is_connected = False
            logging.error(f"Bağlanmaya çalışırken hata: {e}")

    async def update_general_info(self):
        """
        Drone'un tüm bilgilerini günceller.
        """
        while self.is_connected:
            try:
                health, is_armed, flight_mode, battery, gps_info, gps_position, attitude, velocity = await asyncio.gather(
                    self.get_health(),
                    self.is_armed(),
                    self.get_flight_mode(),
                    self.get_remaining_battery(),
                    self.get_gps_info(),
                    self.get_gps_position(),
                    self.get_attitude(),
                    self.get_velocity()
                )

                self.general_status["health"] = health
                self.general_status["armed"] = is_armed
                self.general_status["flight_mode"] = flight_mode
                self.general_status["battery"] = battery
                self.general_status["gps_info"] = gps_info
                self.general_status["gps_position"] = gps_position
                self.general_status["attitude"] = attitude
                self.general_status["velocity"] = velocity

                status_json = json.dumps(self.general_status, indent=2, ensure_ascii=False)
                logging.debug(f"Tüm parametreler güncellendi, parametrelerin son durumu:\n\t\t{status_json}")
            except Exception as e:
                logging.error(f"Drone bilgileri güncellenirken hata: {e}")
            await asyncio.sleep(0.5)
    
    @check_connected
    async def get_general_info(self):
        """
        Drone'un genel bilgilerini döndürür.
        """
        return self.general_status
    
    @check_connected
    async def get_health(self):
        """
        Drone'un kalibrasyon vb. bilgilerini döndürür.
        """
        try:
            async for health in self.drone.telemetry.health():
                if health.is_magnetometer_calibration_ok and \
                   health.is_accelerometer_calibration_ok and \
                   health.is_gyrometer_calibration_ok and \
                   health.is_global_position_ok:
                    logging.info("Drone kalibrasyon bilgileri başarılı.")
                    return True
                else:
                    logging.warning("Drone kalibrasyon bilgileri başarısız, lütfen kontrol edin.")
                    return False
        except Exception as e:
            logging.error(f"Sağlık bilgisi alınırken hata: {e}")
            return None
        
    @check_connected
    async def is_armed(self):
        """
        Drone'un arm durumunu döndürür.
        """
        try:
            async for arm in self.drone.telemetry.armed():
                return arm
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
            async for mode in self.drone.telemetry.flight_mode():
                return mode.value
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
            async for battery in self.drone.telemetry.battery():
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
            async for gps_info in self.drone.telemetry.gps_info():
                return {
                            "Bağlı Uydular":gps_info.num_satellites
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
            async for position in self.drone.telemetry.position():
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
            async for attitude_euler in self.drone.telemetry.attitude_euler():
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
            async for velocity in self.drone.telemetry.velocity_ned():
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
    """
    Test fonksiyonu, dosya doğrudan çalıştırıldığında çalışır.

    Beklenen davranış:
    - Drone ile bağlantı kurulur.
    - Bağlantı başarısız ise hata mesajı verir ve kapanır.
    - Bağlantı başarılı ise sürekli olarak log kayıtları yapılır.
    - Log kayıtları dosyasına ve konsola yazılır.
    - Main kapanana kadar devam eder.
    """
    controller = MAVSDKController(system_address="serial:///dev/ttyUSB0:115200")
    await controller.connect()
    if controller.is_connected:
        while True:
            logging.info("Log kayıtlarından tüm bilgileri görebilirsiniz.")
            await asyncio.sleep(1)
    else:
        logging.error("Drone ile bağlantı kurulamadı.")

if __name__ == "__main__":
    asyncio.run(main())
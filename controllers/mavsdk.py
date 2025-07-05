import os
import time
import logging
import asyncio
import functools
from mavsdk import System

os.makedirs("logs", exist_ok=True)
logging.basicConfig(
        level=logging.INFO, 
        format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
        filename=f"../logs/MAVSDKController_{int(time.time()*1000)}.log",
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
    def __init__(self, system_address="udp://:14540", telemetry_timeout=5, connection_timeout=30):
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
        Drone'un tüm gerekli bilgilerini döndürür.
        """
        try:
            health = await self.get_health()
            battery = await self.get_remaining_battery()
            gps_info = await self.get_gps_info()
            gps_position = await self.get_gps_position()
            orientation = await self.get_orientation()
            velocity = await self.get_velocity()
            
            return {
                "health": health,
                "battery": battery,
                "gps_info": gps_info,
                "gps_position": gps_position,
                "orientation": orientation,
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
            async for health in self.drone.telemetry.health():
                return health
        except Exception as e:
            logging.error(f"Sağlık bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_remaining_battery(self):
        """
        Drone'un kalan bataryasını döndürür.
        """
        try:
            async for battery in self.drone.telemetry.battery():
                return battery
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
                return gps_info
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
                return position
        except asyncio.TimeoutError:
            logging.error("GPS konum bilgisi alınırken zaman aşımına uğradı.")
            return None
        except Exception as e:
            logging.error(f"GPS konum bilgisi alınırken hata: {e}")
            return None
    
    @check_connected
    async def get_orientation(self):
        """
        Drone'un yön bilgilerini döndürür.
        """
        try:
            async for attitude_euler in self.drone.telemetry.attitude_euler():
                return attitude_euler
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
            async for velocity in self.drone.telemetry.velocity():
                return velocity
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
import logging
import asyncio

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController

from utils.formation_utililties import ned_to_latlon, latlon_to_ned, distance_meters

def calculate_ground_sampling_distance(
        altitude, 
        image_size=(1920, 1080),
        focal_length=(1.4*10**-6), 
        sensor_size=(4.75*10**-3), 
    ):
    """
    Yerden Örnekleme Mesafesini (GSD) hesaplar.
    GSD, dronun belirli bir yükseklikteki görüntüleme çözünürlüğünü
    belirler. GSD, dronun yüksekliğine, odak uzunluğuna, sensör boyutuna ve görüntü genişliğine bağlıdır.
    """
    gsd_width = (altitude * sensor_size) / (focal_length * image_size[0])
    gsd_height = (altitude * sensor_size) / (focal_length * image_size[1])
    viable_gsd = min(gsd_width, gsd_height)
    logging.info(f"GSD Hesaplandı: {viable_gsd} m/piksel")
    return viable_gsd
class FormationMission(Mission):
    def __init__(self, drone: DroneController, **kwargs):
        super().__init__("Sürü Keşif Görevi", drone, **kwargs)
        # Rol explorer (kaşif) veya follower (takipçi) olarak ayarlanabilir
        # Kaşif dronlar, kendi bölgelerini lawnmower pattern ile keşfeder
        # ve ArUco marker'ı bulmaya çalışır
        # bulduğunda diğer dronlara konumunu bildirir
        # Takipçi dronlar ise kaşiflerin gönderdiği konuma gider ve iniş yaparlar
        self.role = self.parameters.get("role", "explorer")
        self.altitude = self.parameters.get("altitude", 15)
        # Görev için drona ait keşif bölgesi
        self.district_boundaries = self.parameters.get("district_boundaries", None) if self.role == "explorer" else None
        self.district_to_explore = self.parameters.get("district_to_explore", None) if self.role == "explorer" else None
        # Görev durumunu takip etmek için
        self.mission_status = {
            "marker": {
                "is_found": False,
                "location": None,
            },
            "exploration_complete": False,
            "followers_active": False,
        }

    def determine_district_boundaries(self):
        """
        Drona ait keşif bölgesinin sınırlarını belirler.
        """
        # Lokal koordinat sistemine göre bölge boyutu gelir
        # GPS koordinatlarına dönüştürülür
        general_info = self.drone.MAVSDKController.get_general_info()
        location = general_info["gps_position"]
        district_lat, district_lon = latlon_to_ned(
            location["latitude"], location["longitude"],
        )
        # Alan 2'ye bölünür
        
        # bölge numarasına göre bölge sınırları belirlenir
        # ve belirlenen sınırların gps koordinatları döndürülür
        return NotImplemented
    
    async def discover_marker_in_district(self):
        """
        Dronun keşif bölgesinde ArUco marker'ı bulması için lawnmower pattern ile hareket etmesi
        """
        if not self.district_boundaries or not self.district:
            logging.error("Keşif bölgesi tanımlanmamış!")
            return
        
        # Lawn mower pattern ile keşif yap
        await self.drone.survey_area_with_lawnmower_pattern(self.district_boundaries, self.district)
        
        # Marker bulunduğunda konumu güncelle
        if self.mission_status["marker"]["is_found"]:
            self.mission_status["marker"]["location"] = self.drone.get_current_position()
            logging.info(f"Marker bulundu: {self.mission_status['marker']['location']}")

    async def run(self):
        await super().run()
import time
import logging
import asyncio

from controllers.drone_controller import DroneController
from controllers.step_controller import StepController

from core.drone import Drone

class Mission:
    """
    Temel görev sınıfı. Tüm görevler bu sınıftan türetilir.
    İçerisinde görevin adımlarını ve görev mantığını barındırır.
    Dron üzerinde olabildiğince az işlem yapmaya ve sadece görev ile ilgili işlemleri gerçekleştirmeye odaklanır.
    """
    def __init__(self, mission_name, drone: Drone, **kwargs):
        self.name = mission_name
        self.drone = drone
        self.parameters = kwargs
        self.step_controller = StepController(drone)
        self.status = {
            "is_running": False,
            "start_time": None,
            "end_time": None,
            "error": None
        }

    async def wait_for_drone_health(self):
        while True:
            general_info = await self.drone.mavsdk_controller.get_general_info()
            gps_position = general_info["gps_position"]
            if gps_position and "altitude" in gps_position:
                self.drone.pre_takeoff_location = gps_position
                break
            logging.info("GPS yükseklik bilgisi henüz alınamadı, bekleniyor...")
            await asyncio.sleep(0.5)
    
    async def run(self):
        """
        Görevi başlatır. Adımları kodda eklendikleri sırayla çalıştırır.
        Görev bitince konsola verileri yazar.
        """
        # Görev modül olarak çağırıldığında
        # Dronun tüm bağlantılarının ideal olduğu varsayılır.
        logging.info(f"{self.name} görevi başlatılıyor...")
        start_time = time.time() * 1000
        self.status["is_running"] = True
        self.status["start_time"] = start_time
        try:
            await self.step_controller.run_steps()
            while not self.step_controller.is_all_done:
                logging.debug("Tüm adımların tamamlanması bekleniyor...")
                await asyncio.sleep(1)
        except Exception as e:
            self.status["error"] = str(e)
            logging.error(f"{self.name} görevi sırasında hata oluştu: {e}")
        finally:
            end_time = time.time() * 1000
            self.status["is_running"] = False
            isSuccess = True if self.status["error"] is None else False
            log_message = f"{self.name} görevi {'başarıyla' if isSuccess else 'hatalı şekilde'} tamamlandı."
            if isSuccess:
                logging.info(log_message)
            else:
                logging.error(log_message)
            logging.info(f"Görevin tamamlanma süresi: {end_time - start_time:.2f} ms")

    def abort(self):
        """
        Görevi iptal eder.
        """
        logging.info(f"{self.name} görevi iptal ediliyor...")
        self.status["is_running"] = False
        self.status["error"] = "Görev iptal edildi."
        self.step_controller.abort_steps()

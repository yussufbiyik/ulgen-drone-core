import time
import logging
import asyncio

from controllers.drone_controller import DroneController
from controllers.step_controller import StepController

class Mission:
    """
    Temel görev sınıfı. Tüm görevler bu sınıftan türetilir.
    İçerisinde görevin adımlarını ve görev mantığını barındırır.
    Dron üzerinde olabildiğince az işlem yapmaya ve sadece görev ile ilgili işlemleri gerçekleştirmeye odaklanır.
    """
    def __init__(self, mission_name, drone_controller: DroneController, **kwargs):
        self.name = mission_name
        self.drone = drone_controller
        self.parameters = kwargs
        self.step_controller = StepController()
        self.status = {
            "is_running": False,
            "start_time": None,
            "end_time": None,
            "error": None
        }
    
    async def run(self):
        """
        Görevi başlatır. Adımları kodda eklendikleri sırayla çalıştırır.
        Görev bitince konsola verileri yazar.
        """
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
            isSuccess = self.status["error"] is None
            log_message = f"{self.name} görevi {'başarıyla' if isSuccess else 'hatalı olarak'} tamamlandı."
            if isSuccess:
                logging.info(log_message)
            else:
                logging.error(log_message)
            logging.info(f"Görevin tamamlanma süresi: {end_time - start_time:.2f} ms")

import logging
import asyncio
import time

from core.drone import Drone

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s')

class Step:
    def __init__(self, name, function, checkFunction, preCheckFunction=None, isRequired=False, timeout=None):
        """
        Adım sınıfı.

        Parameters:
        name (str): Adımın adı.
        function (Callable[..., Any]): Adımı gerçekleştiren fonksiyon.
        checkFunction (Callable[[], bool]): Adımın tamamlanıp tamamlanmadığını kontrol eden fonksiyon.
        preCheckFunction (Callable[[], bool], optional): Adımın çalıştırılmadan önce kontrol edilmesi gereken fonksiyon. Varsayılan None.
        isRequired (bool, optional): Adımın gerekli olup olmadığını belirler. Varsılan False.
        timeout (int, optional): Adımın zaman aşımı süresi (milisaniye cinsinden). Varsayılan None.
        """
        self.name = name
        self.function = function
        self.checkFunction = checkFunction
        self.preCheckFunction = preCheckFunction
        self.is_completed = False
        self.isRequired = isRequired
        self.timeout = timeout

class StepController:
    def __init__(self, drone: Drone):
        self.steps = []
        self.active_step = 0
        self.is_all_done = False
        self.drone = drone
        self.wait_for_neighbors= False

    def add_step(self, step: Step):
        """
        Adım ekler.

        Parameters:
        step (Step): Eklenecek adım nesnesi.
        """
        self.steps.append(step)
        logging.info(f"Adım eklendi: {step.name}")

    async def run_steps(self):
        """
        Adımları sırayla çalıştırır.
        """
        logging.info("Adımlar çalıştırılıyor...")
        for step in self.steps:
            step_index = self.steps.index(step) + 1
            self.active_step = step_index
            self.drone.mission_info["current_step"]["index"] = step_index
            self.drone.mission_info["current_step"]["status"] = 0
            start_time = time.time()*1000
            try:
                if step.preCheckFunction is not None:
                    while not await step.preCheckFunction():
                        logging.warning(f"Adım {step.name} ön kontrolü başarısız veya yok, atlanıyor.")
                        if step.timeout and (time.time()*1000 - start_time) > step.timeout:
                            logging.error(f"Adım {step.name} ön kontrolü zaman aşımına uğradı.")
                            break
                        await asyncio.sleep(0.1)
                    logging.info(f"Adım {step.name} ön kontrolü başarılı, çalıştırılıyor...")
                logging.info(f"Adım {step.name} çalıştırılıyor...")
                await step.function()
                logging.info(f"Adım {step.name} kontrol ediliyor...")
                while not await step.checkFunction():
                    logging.debug(f"Adım {step.name} henüz tamamlanmadı, tekrar kontrol ediliyor...")
                    if step.timeout and (time.time()*1000 - start_time) > step.timeout:
                        logging.error(f"Adım {step.name} kontrolü zaman aşımına uğradı.")
                        break
                    await asyncio.sleep(0.1)
                step.is_completed = True
                self.drone.mission_info["current_step"]["status"] = 1
                logging.info(f"Adım {step.name} başarıyla tamamlandı.")
                # Diğer dronları bekle
                if self.wait_for_neighbors:
                    logging.info("Diğer dronların adımı tamamlaması bekleniyor...")
                    while True:
                        drones_to_wait = [
                            neighbor
                            for neighbor in self.drone.neighbors
                            if (neighbor["data"]["mission"]["current_step"]["index"] == step_index
                            and neighbor["data"]["mission"]["current_step"]["status"] == 0)
                            or neighbor["data"]["mission"]["current_step"]["index"] < step_index
                        ]
                        if len(drones_to_wait) == 0:
                            break
                        logging.debug(f"Komşu drone {len(drones_to_wait)} tane drone henüz {step_index}. adımı tamamlamadı, bekleniyor...")
                        await asyncio.sleep(0.1)
                # await self.drone.mavsdk_controller.play_tune("success")
            except Exception as e:
                logging.exception(f"Adım {step.name} sırasında hata oluştu: {e}")
                logging.info("Acil iniş yapılıyor!")
                self.drone.mission_info["current_step"]["status"] = 2 # Görevi hatalı olarak işaretle
                step.is_completed = False
                # await self.drone.mavsdk_controller.play_tune("fail")
                await self.drone.mavsdk_controller.mavsdk.action.land()
                break
        logging.info(f"Görev bitti, {'başarıyla' if self.is_all_done else 'başarısızlıkla'} tamamlandı.")
        self.is_all_done = True
        return self.is_all_done


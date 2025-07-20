import logging
import asyncio

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
)

class Step:
    def __init__(self, name, function, checkFunction, preCheckFunction=None):
        """
        Adım sınıfı.

        Parameters:
        name (str): Adımın adı.
        function (Callable[..., Any]): Adımı gerçekleştiren fonksiyon.
        checkFunction (Callable[[], bool]): Adımın tamamlanıp tamamlanmadığını kontrol eden fonksiyon.
        """
        self.name = name
        self.function = function
        self.checkFunction = checkFunction
        self.preCheckFunction = preCheckFunction
        self.is_completed = False

class StepController:
    def __init__(self):
        self.steps = []
        self.is_all_done = False

    def add_step(self, step):
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
            try:
                if step.preCheckFunction is not None:
                    while not await step.preCheckFunction():
                        logging.warning(f"Adım {step.name} ön kontrolü başarısız veya yok, atlanıyor.")
                        await asyncio.sleep(0.1)
                    logging.info(f"Adım {step.name} ön kontrolü başarılı, çalıştırılıyor...")
                logging.info(f"Adım {step.name} çalıştırılıyor...")
                await step.function()
                logging.info(f"Adım {step.name} kontrol ediliyor...")
                while not await step.checkFunction():
                    logging.debug(f"Adım {step.name} henüz tamamlanmadı, tekrar kontrol ediliyor...")
                    await asyncio.sleep(0.1)
                step.is_completed = True
                logging.info(f"Adım {step.name} başarıyla tamamlandı.")
            except Exception as e:
                logging.error(f"Adım {step.name} sırasında hata oluştu: {e}")
                step.is_completed = False
        logging.info("Tüm adımlar başarıyla tamamlandı.")
        self.is_all_done = True


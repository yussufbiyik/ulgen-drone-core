import os
import time
import logging

import threading

os.makedirs("logs", exist_ok=True)

class Mission:
    def __init__(self, mission_name):
        logging.basicConfig(
                level=logging.INFO, 
                format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
                filename=f"../logs/MISSION_{mission_name}_{int(time.time()*1000)}.log",
            )

    def on_start(self):
        """
        Tek seferlik başlangıç eylemleri burada gerçekleşir.
        """
        pass

    def on_control(self):
        """
        Ana olay döngüsü.
        """
        pass

    def run(self):
        self.on_start()
        while True:
            self.on_control()
            time.sleep(0.1)
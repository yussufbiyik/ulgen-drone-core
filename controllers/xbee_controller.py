import os
import json
import time
import logging
import threading
import functools
from queue import Queue, Full

from digi.xbee.devices import XBeeDevice

os.makedirs("./logs", exist_ok=True)
logging.basicConfig(
        level=logging.INFO, 
        format='[%(asctime)s] - [%(levelname)s]\n\t⤷ %(message)s',
        filename=f"../logs/XBeeController.log",
    )

def check_connected(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.device.is_open():
            logging.error("XBee cihazı açık değil.")
            return None
        return func(self, *args, **kwargs)
    return wrapper

class XBeeController:
    def __init__(self, uuid, port, message_received_callback, baudrate=9600, max_queue_size=20):
        self.uuid = uuid
        self.port = port
        self.baudrate = baudrate
        self.device = XBeeDevice(port, baudrate)
        self.message_received_callback = message_received_callback
        self.recent_messages = Queue(maxsize=max_queue_size)
        self.queue_stop_event = threading.Event()
        
        if self.message_received_callback:
            threading.Thread(target=self.queue_processor, daemon=True).start()
            logging.warning("Mesaj kuyruğu işleme thread'i başlatıldı.")
        else:
            logging.warning("Mesaj alındığında çağrılacak callback fonksiyonu belirtilmemiş.")
    
    def queue_processor(self):
        """
        Mesaj kuyruğundan mesajları işleyen thread fonksiyonu.
        """
        while not self.queue_stop_event.is_set():
            message = self.recent_messages.get(timeout=0.5)
            message_data = message.data.decode('utf-8', errors='replace')
            logging.info(f"Mesaj işleniyor: {message_data}")
            self.message_received_callback(message)
            logging.info("Callback çağrıldı.")
            self.recent_messages.task_done()
    
    def default_message_received_callback(self, message):
        """
        Xbee'den gelen mesajları işleyen callback fonksiyonu.
        """
        try:
            message_data = message.data.decode('utf-8', errors='replace')
            logging.info(f"Mesaj alındı: {message_data}")
            try:
                self.recent_messages.put_nowait(message)
                logging.info("Mesaj kuyruğa eklendi")
            except Full:
                logging.error(f"Mesaj kuyruğa eklenemedi, kuyruk dolu.")
                # Kuyruk doluysa en eski mesajı sil ve yeni mesajı ekle
                logging.info("En eski mesaj siliniyor ve yeni mesaj ekleniyor.")
                self.recent_messages.get_nowait()
                self.recent_messages.put_nowait(message)
        except Exception as e:
            logging.error(f"Mesaj işlenirken hata oluştu: {e}")
    
    def listen(self):
        """
        Xbee mesajlarını dinler ve mesaj gelince callback fonksiyonunu çağırır.
        """
        try:
            if not self.device.is_open():
                self.device.open()
            self.device.add_data_received_callback(self.default_message_received_callback)
            logging.info("XBee dinleniyor...")
        except Exception as e:
            logging.error(f"XBee açılamadı: {e}")
            raise
    
    def construct_message(self, data):
        """
        Verilen mesajı JSON formatına çevirir.
        """
        message = {
            "uuid": self.uuid,
            "data": data,
            "timestamp": int(time.time()*1000)
        }
        logging.debug(f"Mesaj yapılandırıldı.")
        return json.dumps(message, ensure_ascii=False)
    
    @check_connected
    def send_broadcast_message(self, data):
        """
        Xbee üzerinden veri yayınlar (broadcast eder).
        """
        try:
            message = self.construct_message(data)
            self.device.send_data_broadcast(message)
            logging.info(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: Broadcast")
            return True
        except Exception as e:
            logging.error(f"Mesaj gönderilemedi: {e}")
            return False
    
    @check_connected
    def send_private_message(self, receiver, data):
        """
        Xbee üzerinden bir alıcıya veri gönderir.
        """
        message = self.construct_message(data)
        try:
            self.device.send_data(receiver, message)
            logging.info(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: {receiver}")
            return True
        except Exception as e:
            logging.error(f"Mesaj gönderilemedi: {e}")
            return False
    
    def close(self):
        """
        XBee cihazını kapatır ve mesaj kuyruğu işleme thread'ini durdurur.
        """
        if self.device.is_open():
            self.device.close()
            logging.info("XBee kapatıldı.")
            self.stop_event.set()
            logging.info("Mesaj kuyruğu işleme thread'i durduruldu.")
        else:
            logging.warning("XBee zaten kapalı.")
            

if __name__ == "__main__":
    # Örnek kullanım
    def message_received_callback(message):
        print(f"Mesaj alındı: {message.data.decode('utf-8', errors='replace')}")

    xbee = XBeeController(uuid="12345", port="/dev/ttyUSB0", message_received_callback=message_received_callback)
    xbee.listen()
    
    # Mesaj gönderme örneği
    xbee.send_broadcast_message({"test": "Hello, XBee!"})
    
    # Uygulama kapatılırken XBee cihazını kapat
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        xbee.close()
import os
import sys 
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import json
import threading
import time
import logging
import serial
from queue import Queue, Full
import functools
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import XBeeException, TransmitException, TimeoutException, InvalidOperatingModeException

# Logging configuration
log_name = "./logs/XBeeController.log"

# {
#   "id": 12345,
#   "st": {
#     "ab":1,
#     "a": 1,
#     "f_mode": 3,
#     "b": 50,
#     "gps": {
#       "la": 47.397971299999995,
#       "lo": 8.5461633,
#       "al": 5.200000286102295
#     }
#   }
# }

logger = logging.getLogger("XBeeController")
sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
fh = logging.FileHandler(log_name, mode='w')
fh.setLevel(logging.DEBUG)
logging.basicConfig(
        format='[%(asctime)s | %(levelname)s]\n\t⤷ %(message)s',
    level=logging.DEBUG,  # Genel log seviyesi DEBUG olarak ayarlandı
        handlers=[fh, sh]
    )

def check_connected(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.device.is_open():
            logger.error("XBee cihazı açık değil.")
            return None
        return func(self, *args, **kwargs)
    return wrapper

class XBeeController:
    def __init__(self, uuid, port, message_received_callback, baudrate=57600, max_queue_size=20):
        self.uuid = uuid
        self.port = port
        self.baudrate = baudrate
        self.device = XBeeDevice(port, baudrate)
        self.message_received_callback = message_received_callback
        self.recent_messages = Queue(maxsize=max_queue_size)
        self.queue_stop_event = threading.Event()
        # self.configure_xbee_api_mode()
        if self.message_received_callback:
            threading.Thread(target=self.queue_processor, daemon=True).start()
            logger.warning("Mesaj kuyruğu işleme thread'i başlatıldı.")
        else:
            logger.warning("Mesaj alındığında çağrılacak callback fonksiyonu belirtilmemiş.")
    
    def queue_processor(self):
        """
        Mesaj kuyruğundan mesajları işleyen thread fonksiyonu.
        """
        while not self.queue_stop_event.is_set():
            if self.recent_messages.empty():
                time.sleep(0.1)
                continue
            message = self.recent_messages.get(timeout=0.5)
            message_data = message.data.decode('utf-8', errors='replace')
            logger.info(f"Mesaj işleniyor: {message_data}")
            self.message_received_callback(message)
            logger.info("Callback çağrıldı.")
            self.recent_messages.task_done()
    
    def default_message_received_callback(self, message):
        """
        Xbee'den gelen mesajları işleyen callback fonksiyonu.
        """
        try:
            message_data = message.data.decode('utf-8', errors='replace')
            logger.info(f"Mesaj alındı: {message_data}")
            try:
                self.recent_messages.put_nowait(message)
                logger.info("Mesaj kuyruğa eklendi")
            except Full:
                logger.error(f"Mesaj kuyruğa eklenemedi, kuyruk dolu.")
                # Kuyruk doluysa en eski mesajı sil ve yeni mesajı ekle
                logger.info("En eski mesaj siliniyor ve yeni mesaj ekleniyor.")
                self.recent_messages.get_nowait()
                self.recent_messages.put_nowait(message)
        except Exception as e:
            logger.error(f"Mesaj işlenirken hata oluştu: {e}")
    
    def configure_xbee_api_mode(self):
        """
        XBee cihazını API moduna geçirir.
        """
        try:
            logger.info("XBee cihazı API moduna geçiriliyor...")
            # Serial bağlantı kur
            ser = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(1)  # Bağlantının stabilleşmesi için bekle
            # Command moduna geç
            ser.write(b'+++')
            time.sleep(2)
            response = ser.read(ser.in_waiting)
            logger.info(f"Command mode response: {response}")
            # API mode 1'e geç (AP=1)
            ser.write(b'ATAP1\r')
            time.sleep(0.5)
            response = ser.read(ser.in_waiting)
            logger.info(f"API mode response: {response}")
            # Ayarları kaydet
            ser.write(b'ATWR\r')
            time.sleep(0.5)
            response = ser.read(ser.in_waiting)
            logger.info(f"Write response: {response}")
            # Command modundan çık
            ser.write(b'ATCN\r')
            time.sleep(0.5)
            ser.close()
            logger.info("XBee başarıyla API moduna geçirildi.")
            return True
            
        except Exception as e:
            logger.error(f"XBee API moduna geçirilirken hata: {e}")
            return False

    def listen(self):
        """
        Xbee mesajlarını dinler ve mesaj gelince callback fonksiyonunu çağırır.
        """
        try:
            if not self.device.is_open():
                self.device.open()
            self.device.add_data_received_callback(self.default_message_received_callback)
            logger.info("XBee dinleniyor...")
        except Exception as e:
            logger.error(f"XBee açılamadı: {e}")
            raise
    
    def construct_message(self, data):
        """
        Verilen mesajı JSON formatına çevirir.
        """
        message = {
            "i": self.uuid,
            "d": data,
            "t": int(time.time()*1000)
        }
        logger.debug(f"Mesaj yapılandırıldı.")
        return json.dumps(message, ensure_ascii=False).replace("\n", "").replace(" ", "").encode('utf-8')
    
    @check_connected
    def send_broadcast_message(self, data):
        """
        Xbee üzerinden veri yayınlar (broadcast eder).
        """
        try:
            message = self.construct_message(data)
            logger.debug(f"Broadcast mesajı yapılandırıldı: {message}")
            self.device.send_data_broadcast(data)
            logger.info(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: Broadcast")
            return True
        except XBeeException as e:
            logger.error(f"XBee Hatası: {e}")
            return False
        except TimeoutException as e:
            logger.error(f"Zaman aşımı hatası: {e}")
            return False
        except TransmitException as e:
            logger.error(f"Transmit hatası: {e}")
            return False
        except InvalidOperatingModeException as e:
            logger.error(f"Geçersiz çalışma modu hatası: {e}")
            return False
    
    @check_connected
    def send_private_message(self, receiver, data):
        """
        Xbee üzerinden bir alıcıya veri gönderir.
        """
        message = self.construct_message(data)
        try:
            self.device.send_data(receiver, message)
            logger.info(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: {receiver}")
            return True
        except Exception as e:
            logger.error(f"Mesaj gönderilemedi: {e}")
            return False
    
    def close(self):
        """
        XBee cihazını kapatır ve mesaj kuyruğu işleme thread'ini durdurur.
        """
        if self.device.is_open():
            self.device.close()
            logger.info("XBee kapatıldı.")
            self.queue_stop_event.set()
            logger.info("Mesaj kuyruğu işleme thread'i durduruldu.")
        else:
            logger.warning("XBee zaten kapalı.")
            

if __name__ == "__main__":
    # Örnek kullanım
    def message_received_callback(message):
        logger.info(f"Mesaj alındı: {message.data.decode('utf-8', errors='replace')}")

    xbee = XBeeController(uuid="123", port="/dev/ttyUSB0", message_received_callback=message_received_callback)
    xbee.listen()
    
    # Uygulama kapatılırken XBee cihazını kapat
    try:
        while True:
            # Mesaj gönderme örneği
            xbee.send_broadcast_message("123,1,1,3,50,47.397971299999995,8.5461633,5.200000286102295")
            # xbee.send_broadcast_message({
            #     "d": {
            #         "ab":1,
            #         "a": 1,
            #         "fm": 3,
            #         "b": 50,
            #         "gps": 
            #             {
            #                 "la": 47.397971299999995,
            #                 "lo": 8.5461633,
            #                 "al": 5.200000286102295
            #             }
            #     }
            # })
            time.sleep(15)
    except KeyboardInterrupt:
        xbee.close()
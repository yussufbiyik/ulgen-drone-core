import os
import json
import time
import logging
import threading
import functools
import serial
from queue import Queue, Full, Empty

from digi.xbee.devices import XBeeDevice

log_name = "./logs/DroneController.log"
os.makedirs("./logs", exist_ok=True)
logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s | %(levelname)s]\n\t⤷ %(message)s',
        handlers=[
            logging.FileHandler(log_name, mode='w'),
            logging.StreamHandler()
        ]
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
            try:
                message = self.recent_messages.get(timeout=0.5)
                message_data = message.data.decode('utf-8', errors='replace')
                logging.info(f"Mesaj işleniyor: {message_data}")
                self.message_received_callback(message)
                logging.info("Callback çağrıldı.")
                self.recent_messages.task_done()
            except Empty:
                # Queue boşken timeout olursa devam et
                continue
            except Exception as e:
                logging.error(f"Mesaj işlenirken hata oluştu: {e}")
    
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
            # Önce cihazın mevcut olup olmadığını kontrol et
            if not self.check_device_availability():
                raise Exception(f"XBee cihazı {self.port} portunda bulunamadı.")
            
            # XBee'yi API moduna geçirmeyi dene
            if not self.configure_xbee_api_mode():
                logging.warning("XBee API moduna geçirilemedi, mevcut modda deneniyor...")
            
            # Eğer hala açılamıyorsa, doğru baud rate'i tespit etmeye çalış
            try:
                if not self.device.is_open():
                    logging.info("XBee cihazı açılıyor...")
                    self.device.open()
                    logging.info("XBee cihazı başarıyla açıldı.")
            except Exception as e:
                logging.warning(f"Varsayılan baud rate ({self.baudrate}) ile açılamadı: {e}")
                logging.info("Doğru baud rate tespit edilmeye çalışılıyor...")
                
                detected_baudrate = self.detect_baudrate()
                if detected_baudrate and detected_baudrate != self.baudrate:
                    logging.info(f"Baud rate {detected_baudrate} olarak güncelleniyor...")
                    self.baudrate = detected_baudrate
                    self.device = XBeeDevice(self.port, self.baudrate)
                    self.device.open()
                    logging.info("XBee cihazı doğru baud rate ile açıldı.")
                else:
                    raise e
                    
            self.device.add_data_received_callback(self.default_message_received_callback)
            logging.info("XBee dinleniyor...")
        except Exception as e:
            logging.error(f"XBee açılamadı: {e}")
            logging.info("XBee cihazının bağlı olduğundan ve doğru port belirtildiğinden emin olun.")
            logging.info("Ayrıca XBee cihazının API modunda olduğundan emin olun.")
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
            self.queue_stop_event.set()
            logging.info("Mesaj kuyruğu işleme thread'i durduruldu.")
        else:
            logging.warning("XBee zaten kapalı.")
            self.queue_stop_event.set()
    
    def check_device_availability(self):
        """
        XBee cihazının mevcut olup olmadığını kontrol eder.
        """
        try:
            # Portu açmayı dene
            test_serial = serial.Serial(self.port, self.baudrate, timeout=1)
            test_serial.close()
            logging.info(f"Port {self.port} mevcut ve erişilebilir.")
            return True
        except serial.SerialException as e:
            logging.error(f"Port {self.port} erişilemez: {e}")
            return False
        except Exception as e:
            logging.error(f"Port kontrolü sırasında beklenmeyen hata: {e}")
            return False
    
    def configure_xbee_api_mode(self):
        """
        XBee cihazını API moduna geçirir.
        """
        try:
            logging.info("XBee cihazı API moduna geçiriliyor...")
            
            # Serial bağlantı kur
            ser = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(1)  # Bağlantının stabilleşmesi için bekle
            
            # Command moduna geç
            ser.write(b'+++')
            time.sleep(1)
            response = ser.read(ser.in_waiting)
            logging.debug(f"Command mode response: {response}")
            
            # API mode 1'e geç (AP=1)
            ser.write(b'ATAP1\r')
            time.sleep(0.5)
            response = ser.read(ser.in_waiting)
            logging.debug(f"API mode response: {response}")
            
            # Ayarları kaydet
            ser.write(b'ATWR\r')
            time.sleep(0.5)
            response = ser.read(ser.in_waiting)
            logging.debug(f"Write response: {response}")
            
            # Command modundan çık
            ser.write(b'ATCN\r')
            time.sleep(0.5)
            
            ser.close()
            logging.info("XBee başarıyla API moduna geçirildi.")
            return True
            
        except Exception as e:
            logging.error(f"XBee API moduna geçirilirken hata: {e}")
            return False
    
    def detect_baudrate(self):
        """
        XBee cihazının baud rate'ini tespit etmeye çalışır.
        """
        common_baudrates = [9600, 115200, 57600, 38400, 19200, 4800, 2400]
        
        for baudrate in common_baudrates:
            try:
                logging.info(f"Baud rate {baudrate} deneniyor...")
                test_device = XBeeDevice(self.port, baudrate)
                test_device.open()
                
                # Eğer başarılı bir şekilde açıldıysa, bu doğru baud rate
                test_device.close()
                logging.info(f"Doğru baud rate bulundu: {baudrate}")
                return baudrate
                
            except Exception as e:
                logging.debug(f"Baud rate {baudrate} başarısız: {e}")
                continue
                
        logging.error("Hiçbir baud rate çalışmadı.")
        return None

if __name__ == "__main__":
    # Örnek kullanım
    def message_received_callback(message):
        print(f"Mesaj alındı: {message.data.decode('utf-8', errors='replace')}")

    xbee = XBeeController(uuid="12345", port="/dev/ttyUSB0", message_received_callback=message_received_callback)
    
    try:
        xbee.listen()
        
        # Mesaj gönderme örneği
        xbee.send_broadcast_message({"test": "Hello, XBee!"})
        
        # Uygulama kapatılırken XBee cihazını kapat
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Uygulama kapatılıyor...")
            xbee.close()
    except Exception as e:
        logging.error(f"Uygulama başlatılamadı: {e}")
        logging.info("Lütfen XBee cihazının bağlı olduğundan emin olun.")
        if hasattr(xbee, 'device') and xbee.device.is_open():
            xbee.close()
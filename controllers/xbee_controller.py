import threading
import time
import logging
from collections import deque
from digi.xbee.devices import XBeeDevice

# Logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s | %(levelname)s] %(message)s'
)

class XBeeController:
    def __init__(self, port="/dev/ttyUSB0", baud_rate=57600, send_interval=0.1, queue_retention=10, remote_node_id="REMOTE", drone_name="drone2"):
        """
        XBee Controller sınıfı
        
        Args:
            port: XBee cihazının bağlı olduğu port
            baud_rate: Baud rate
            send_interval: Mesaj gönderme aralığı (saniye)
            queue_retention: Kuyrukta mesaj tutma süresi (saniye)
            remote_node_id: Uzak cihaz ID'si (None ise broadcast)
            drone_name: Bu drone'un adı
        """
        self.port = port
        self.baud_rate = baud_rate
        self.send_interval = send_interval
        self.queue_retention = queue_retention
        self.remote_node_id = remote_node_id
        self.drone_name = drone_name
        
        self.device = XBeeDevice(port, baud_rate)
        self.remote_device_cache = None
        self.signal_queue = deque()
        self.queue_lock = threading.Lock()
        
        self.sender_thread = None
        self.cleaner_thread = None
        self.is_running = False
        
        logging.info(f"XBeeController oluşturuldu - Port: {port}, Baud: {baud_rate}")
    
    def get_message(self):
        """
        Gönderilecek mesajı oluşturur
        Format: name,velocity,position(x,y,z),orientation(x,y,z),timestamp
        """
        return f"{self.drone_name},0.5,0,0,0,0,0,0,{int(time.time())}"
    
    def data_receive_callback(self, xbee_message):
        """
        XBee'den gelen mesajları işleyen callback fonksiyonu
        """
        try:
            data = xbee_message.data.decode('utf-8')
            fields = data.split(',')
            logging.info(f"📩 Gelen mesaj: {fields}")
        except Exception as e:
            logging.warning(f"📩 Gelen mesaj (ham): {xbee_message.data} (Hata: {e})")
            fields = xbee_message.data
        
        with self.queue_lock:
            self.signal_queue.append((time.time(), 'IN', fields))
    
    def send_data_periodically(self):
        """
        Periyodik olarak veri gönderen thread fonksiyonu
        """
        if self.remote_node_id:
            try:
                self.remote_device_cache = self.device.get_network().discover_device(self.remote_node_id)
                if self.remote_device_cache:
                    logging.info(f"🎯 Uzak cihaz bulundu: {self.remote_device_cache.get_64bit_addr()}")
                else:
                    logging.warning("⚠️ Uzak cihaz bulunamadı, broadcast ile gönderilecek.")
            except Exception as e:
                logging.error(f"Cihaz bulma hatası: {e}")
        
        while self.is_running and self.device.is_open():
            try:
                msg = self.get_message()
                if self.remote_device_cache:
                    self.device.send_data(self.remote_device_cache, msg)
                    logging.debug("✅ Mesaj gönderildi.")
                else:
                    self.device.send_data_broadcast(msg)
                    logging.debug("📡 Broadcast ile mesaj gönderildi.")
                
                with self.queue_lock:
                    self.signal_queue.append((time.time(), 'OUT', msg))
            except Exception as e:
                logging.error(f"Gönderim hatası: {e}")
            
            time.sleep(self.send_interval)
    
    def queue_cleaner(self):
        """
        Eski mesajları kuyruktan temizleyen thread fonksiyonu
        """
        while self.is_running:
            now = time.time()
            with self.queue_lock:
                while self.signal_queue and now - self.signal_queue[0][0] > self.queue_retention:
                    self.signal_queue.popleft()
            time.sleep(1)
    
    def start(self):
        """
        XBee iletişimini başlatır
        """
        try:
            self.device.open()
            logging.info("📡 XBee cihazı açıldı, dinleniyor ve gönderiyor...")
            self.device.add_data_received_callback(self.data_receive_callback)
            
            self.is_running = True
            
            # Thread'leri başlat
            self.sender_thread = threading.Thread(target=self.send_data_periodically, daemon=True)
            self.sender_thread.start()
            
            self.cleaner_thread = threading.Thread(target=self.queue_cleaner, daemon=True)
            self.cleaner_thread.start()
            
            logging.info("XBee Controller başarıyla başlatıldı.")
            return True
            
        except Exception as e:
            logging.error(f"XBee Controller başlatılamadı: {e}")
            return False
    
    def stop(self):
        """
        XBee iletişimini durdurur
        """
        self.is_running = False
        
        if self.device.is_open():
            self.device.close()
            logging.info("🔌 XBee bağlantısı kapatıldı.")
        
        logging.info("XBee Controller durduruldu.")
    
    def get_signal_queue(self):
        """
        Sinyal kuyruğunun bir kopyasını döndürür
        """
        with self.queue_lock:
            return list(self.signal_queue)
    
    def is_connected(self):
        """
        XBee cihazının bağlı olup olmadığını kontrol eder
        """
        return self.device.is_open() if self.device else False
    
    def send_custom_message(self, message):
        """
        Özel bir mesaj gönderir
        """
        try:
            if not self.device.is_open():
                logging.error("XBee cihazı açık değil.")
                return False
            
            if self.remote_device_cache:
                self.device.send_data(self.remote_device_cache, message)
            else:
                self.device.send_data_broadcast(message)
            
            with self.queue_lock:
                self.signal_queue.append((time.time(), 'OUT', message))
            
            logging.info(f"Özel mesaj gönderildi: {message}")
            return True
            
        except Exception as e:
            logging.error(f"Özel mesaj gönderilemedi: {e}")
            return False


def main():
    """
    Örnek kullanım
    """
    # XBee Controller'ı oluştur
    xbee_controller = XBeeController(
        port="/dev/ttyUSB0",
        baud_rate=57600,
        send_interval=0.1,
        queue_retention=10,
        remote_node_id="REMOTE",  # None yaparsanız sadece broadcast yapar
        drone_name="drone2"
    )
    
    try:
        # Controller'ı başlat
        if xbee_controller.start():
            print("XBee Controller başlatıldı. Çıkmak için Enter'a basın...")
            input()
        else:
            print("XBee Controller başlatılamadı!")
    
    except KeyboardInterrupt:
        logging.info("Kullanıcı tarafından durduruldu.")
    
    finally:
        xbee_controller.stop()


if __name__ == "__main__":
    main()
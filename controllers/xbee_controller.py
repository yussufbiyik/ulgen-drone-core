import sys
import asyncio
import json
import threading
import time
import logging
import functools
import inspect
from queue import Queue, Full

from digi.xbee.devices import XBeeDevice, XBeeMessage, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import XBeeException, TransmitException, TimeoutException, InvalidOperatingModeException

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

def check_connected(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.device.is_open():
            logging.error("XBee cihazı açık değil.")
            return None
        return func(self, *args, **kwargs)
    return wrapper

class XBeeController:
    def __init__(self, message_received_callback, port="/dev/ttyUSB0", baudrate=57600, max_queue_size=100000):
        self.port = port
        self.baudrate = baudrate
        self.device = XBeeDevice(port, baudrate)
        self.device.open()
        self.address = int.from_bytes(self.device.get_64bit_addr().address, "big")
        self.network = self.device.get_network()
        # self.message_received_callback = message_received_callback
        self.subscribers = []
        if message_received_callback:
            self.subscribers.append(message_received_callback)
        self.recent_messages = Queue(maxsize=max_queue_size)
        self.queue_thread = asyncio.create_task(self.queue_processor())
        if self.subscribers:
            logging.warning("Mesaj kuyruğu işleme thread'i başlatıldı.")
        else:
            logging.warning("Mesaj alındığında çağrılacak callback fonksiyonu belirtilmemiş.")

    
    async def queue_processor(self):
        """
        Mesaj kuyruğundan mesajları işleyen thread fonksiyonu.
        """
        while not self.queue_thread.done():
            if self.recent_messages.empty():
                await asyncio.sleep(0.1)
                continue
            message = self.recent_messages.get(timeout=0.5)
            logging.debug(f"Mesaj işleniyor: {message}")
            for callback in list(self.subscribers):
                try:
                    is_async = inspect.iscoroutinefunction(callback)
                    if is_async:
                        await callback(message)
                    else:
                        callback(message)
                except Exception as e:
                    logging.exception(f"Bir abonenin callback fonksiyonu çalıştırılırken hata oluştu: {e}")
            logging.debug("Callback çağrıldı.")
            self.recent_messages.task_done()

    def subscribe(self, callback):
        """
        XBee'den gelen mesajlara bir callback fonksiyonunu abone eder.
        """
        self.subscribers.append(callback)
        if self.queue_thread.done():
            self.queue_thread = asyncio.create_task(self.queue_processor())
            logging.info("Mesaj kuyruğu işleme thread'i yeniden başlatıldı.")

    def unsubscribe(self, callback):
        """
        Verilen callback fonksiyonunu abonelikten çıkarır.
        """
        self.subscribers.remove(callback)

    def default_message_received_callback(self, message: XBeeMessage):
        """
        Xbee'den gelen mesajları işleyen callback fonksiyonu.
        """
        try:
            message_data = message.data.decode('utf-8')
            sender = int.from_bytes(message.remote_device.get_64bit_addr().address, "big")
            message_full = {
                "sender": sender,
                "isBroadcast": message.is_broadcast,
                "data": message_data,
                "timestamp": message.timestamp
            }
            logging.debug(f"Mesaj alındı: {message_full}")
            try:
                self.recent_messages.put_nowait(message_full)
                logging.debug("Mesaj kuyruğa eklendi")
            except Full:
                logging.error(f"Mesaj kuyruğa eklenemedi, kuyruk dolu.")
                # Kuyruk doluysa en eski mesajı sil ve yeni mesajı ekle
                logging.debug("En eski mesaj siliniyor ve yeni mesaj ekleniyor.")
                self.recent_messages.get_nowait()
                self.recent_messages.put_nowait(message_full)
        except Exception as e:
            logging.error(f"Mesaj işlenirken hata oluştu: {e}")

    def listen(self):
        """
        Xbee mesajlarını dinler ve mesaj gelince callback fonksiyonunu çağırır.
        """
        try:
            if not self.device.is_open():
                self.device.open()
                self.address = int.from_bytes(self.device.get_64bit_addr().address, "big")
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
            "i": self.address,
            "d": data,
            "t": int(time.time()*1000)
        }
        logging.debug(f"Mesaj yapılandırıldı.")
        return json.dumps(message, ensure_ascii=False).replace("\n", "").replace(" ", "").encode('utf-8')
    
    @check_connected
    def send_broadcast_message(self, data, construct_message=False):
        """
        Xbee üzerinden veri yayınlar (broadcast eder).
        """
        try:
            if construct_message:
                message = self.construct_message(data)
            else:
                message = data
            logging.debug(f"Broadcast mesajı yapılandırıldı: {message}")
            self.device.send_data_broadcast(message)
            logging.debug(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: Broadcast")
            return True
        except XBeeException as e:
            logging.error(f"XBee Hatası: {e}")
            return False
        except TimeoutException as e:
            logging.error(f"Zaman aşımı hatası: {e}")
            return False
        except TransmitException as e:
            logging.error(f"Transmit hatası: {e}")
            return False
        except InvalidOperatingModeException as e:
            logging.error(f"Geçersiz çalışma modu hatası: {e}")
            return False
    
    @check_connected
    def send_private_message(self, receiver: int, data):
        """
        Xbee üzerinden bir alıcıya veri gönderir.
        """
        # message = self.construct_message(data)
        try:
            receiver = receiver.to_bytes(8, "big")
            address = XBee64BitAddress(receiver)
            device = self.network.get_device_by_64(address)
            self.device.send_data(device, data)
            logging.debug(f"Mesaj gönderildi:\n Mesaj: {data}\nAlıcı: {receiver}")
            return True
        except Exception as e:
            logging.exception(f"Mesaj gönderilemedi: {e}")
            return False
    
    def close(self):
        """
        XBee cihazını kapatır ve mesaj kuyruğu işleme thread'ini durdurur.
        """
        if self.device.is_open():
            self.device.close()
            logging.info("XBee kapatıldı.")
            self.queue_thread.cancel()
            logging.info("Mesaj kuyruğu işleme thread'i durduruldu.")
        else:
            logging.warning("XBee zaten kapalı.")
            
def main():
    """
    XBeeController test fonksiyonu.
    Bu fonksiyon, XBee cihazını başlatır, mesaj gönderir ve dinler.
    """
    xbee = XBeeController(message_received_callback=XBeeController.default_message_received_callback)
    xbee.listen()
    
    # Test mesajı gönder
    try:
        while True:
            # xbee.send_broadcast_message("oto mesaj", construct_message=True)
            time.sleep(1)
    except KeyboardInterrupt:
        xbee.close()

if __name__ == "__main__":
    logging.info("XBeeController başlatılıyor...")
    main()
    logging.info("XBeeController testinin tüm adımları tamamlandı, çıkılıyor.")
    sys.exit(0)
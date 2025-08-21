import socket
import threading
import json
import time
import logging

# Logging yapılandırması
logging.basicConfig(
    format='%(asctime)s - %(levelname)s - %(message)s',
    level=logging.INFO,
    datefmt='%Y-%m-%d %H:%M:%S'
)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 5005

clients = {}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, SERVER_PORT))

logging.info(f"Sunucu {SERVER_IP}:{SERVER_PORT} üzerinde dinliyor")

def handle_message(data, addr):
    # Ham veriyi json'a dönüştür, decode hatalarını yok say
    raw_data_str = json.loads(data.decode(errors='ignore'))
    sender_id = raw_data_str.get("sender", None)
    
    # Yeni client ekle
    if addr not in clients:
        clients[addr] = sender_id
        logging.info(f"Yeni drone bağlandı {addr}, ID atandı: {sender_id}")

    broadcast_msg = {
        "sender": sender_id,
        "timestamp": time.time(),
        "data": raw_data_str.get("data", "")
    }

    broadcast_data = json.dumps(broadcast_msg).encode()

    target_id = raw_data_str.get("target", None)
    if target_id:
        # Belirtilen target ID'ye mesaj gönder
        for client_addr, client_id in clients.items():
            if client_id == target_id:
                sock.sendto(broadcast_data, client_addr)
                logging.info(f"Mesaj {sender_id} → {target_id} gönderildi")
                break
        else:
            logging.warning(f"Target ID {target_id} bulunamadı")
    else:
        # Gönderen hariç tüm drone'lara gönder
        for client_addr in clients:
            if client_addr != addr:
                sock.sendto(broadcast_data, client_addr)

while True:
    data, addr = sock.recvfrom(1024)
    threading.Thread(target=handle_message, args=(data, addr)).start()

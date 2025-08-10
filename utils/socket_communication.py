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
    id = raw_data_str.get("sender", None)
    if addr not in clients:
        clients[addr] = id
        logging.info(f"Yeni drone bağlandı {addr}, ID atandı: {id}")

    broadcast_msg = {
        "sender": id,
        "timestamp": time.time(),
        "data": raw_data_str.get("data", "")
    }

    broadcast_data = json.dumps(broadcast_msg).encode()

    # Gönderen hariç tüm drone'lara gönder
    for client_addr in clients:
        if client_addr != addr:
            sock.sendto(broadcast_data, client_addr)

while True:
    data, addr = sock.recvfrom(1024)
    threading.Thread(target=handle_message, args=(data, addr)).start()

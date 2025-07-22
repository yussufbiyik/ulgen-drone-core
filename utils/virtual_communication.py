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

clients = {}  # addr -> drone_id
next_id = 1

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, SERVER_PORT))

logging.info(f"Sunucu {SERVER_IP}:{SERVER_PORT} üzerinde dinliyor")

def handle_message(data, addr):
    global next_id
    if addr not in clients:
        clients[addr] = next_id
        logging.info(f"Yeni drone bağlandı {addr}, ID atandı: {next_id}")
        next_id += 1

    drone_id = clients[addr]

    # Ham veriyi stringe dönüştür, decode hatalarını yok say
    raw_data_str = data.decode(errors='ignore')

    broadcast_msg = {
        "sender": drone_id,
        "timestamp": time.time(),
        "data": raw_data_str
    }

    broadcast_data = json.dumps(broadcast_msg).encode()

    # Gönderen hariç tüm drone'lara gönder
    for client_addr in clients:
        if client_addr != addr:
            sock.sendto(broadcast_data, client_addr)

while True:
    data, addr = sock.recvfrom(1024)
    threading.Thread(target=handle_message, args=(data, addr)).start()

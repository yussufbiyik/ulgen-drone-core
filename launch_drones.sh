#!/bin/bash

# KULLANIM:
#   ./launch_drones.sh <drone_sayısı>
# ÖRNEK:
#   ./launch_drones.sh 3

DRONE_SAYISI=$1
if [ -z "$DRONE_SAYISI" ]; then
  echo "Kullanım: $0 <drone_sayısı>"
  exit 1
fi

# PX4 binary yolu
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [ ! -f "$PX4_BIN" ]; then
  echo "❌ Hata: PX4 çalıştırılabilir dosyası bulunamadı: $PX4_BIN"
  echo "Lütfen PX4'ü tekrar derleyin ve ardından bu scripti çalıştırın."
  exit 1
fi

# mavsdk_server binary yolu
MAVSDK_SERVER=$(python -c "import os, mavsdk; print(os.path.join(os.path.dirname(mavsdk.__file__), 'bin', 'mavsdk_server'))")
if [ ! -f "$MAVSDK_SERVER" ]; then
  echo "❌ Hata: mavsdk_server bulunamadı!"
  echo "pip install mavsdk komutu ile yüklemeyi deneyin."
  exit 1
fi

# Log klasörü
LOG_DIR="$HOME/drone_logs"
mkdir -p "$LOG_DIR"

# Port başlangıç değerleri
BASLANGIC_MAVLINK_PORT=14540       # PX4 MAVLink UDP portu (dinleme)
BASLANGIC_MAVSDK_REMOTE_PORT=14560 # mavsdk_server'ın dinleyeceği UDP portu
BASLANGIC_MAVSDK_TCP_PORT=50060    # mavsdk_server TCP portu (Python için)

# PID dizileri
PX4_PIDS=()
MAVSDK_PIDS=()

echo "🚁 $DRONE_SAYISI drone başlatılıyor..."

for (( i=0; i<$DRONE_SAYISI; i++ ))
do
  MAVLINK_PORT=$((BASLANGIC_MAVLINK_PORT + i))          # PX4 UDP listen port
  PX4_REMOTE_PORT=$((BASLANGIC_MAVSDK_REMOTE_PORT + i)) # PX4 MAVLink send port (=mavsdk_server UDP listen port)
  MAVSDK_TCP_PORT=$((BASLANGIC_MAVSDK_TCP_PORT + i))    # mavsdk_server TCP port

  Y_OFFSET=$((i * 2))

  echo ""
  echo "[$i] Drone Başlatılıyor..."
  echo "  PX4 UDP Dinleme Portu      : $MAVLINK_PORT"
  echo "  PX4 MAVLink Gönderme Portu : $PX4_REMOTE_PORT"
  echo "  MAVSDK Server TCP Portu    : $MAVSDK_TCP_PORT"
  echo "  MAVSDK Bağlantı Örnekleri:"

  echo "    Python (UDP ile):"
  echo "      drone = System(port=$MAVSDK_TCP_PORT)"
  echo "      await drone$i.connect(system_address=\"udp://0.0.0.0:$MAVLINK_PORT\")"

  # PX4 başlat
  PX4_SYS_AUTOSTART=4019 \
  PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_MODEL_POSE="0,$Y_OFFSET" \
  MAV_0_UDP_PRT=$MAVLINK_PORT \
  MAV_0_REMOTE_PRT=$PX4_REMOTE_PORT \
  $PX4_BIN -i $i > "$LOG_DIR/px4_sitl_$i.log" 2>&1 &

  PX4_PID=$!
  PX4_PIDS+=($PX4_PID)

  # mavsdk_server başlat
  $MAVSDK_SERVER udp://:$PX4_REMOTE_PORT -p $MAVSDK_TCP_PORT > "$LOG_DIR/mavsdk_server_$i.log" 2>&1 &

  MAVSDK_PID=$!
  MAVSDK_PIDS+=($MAVSDK_PID)

  echo "  → PX4 PID: $PX4_PID | MAVSDK PID: $MAVSDK_PID"

  sleep 3
done

echo ""
echo "✅ Tüm dronelar başarıyla başlatıldı. Kapatmak için Ctrl+C'ye basın."

# Temizlik fonksiyonu: Ctrl+C ile kapatınca tüm süreçleri öldürür
temizlik() {
  echo ""
  echo "🧹 Tüm PX4 ve mavsdk_server süreçleri sonlandırılıyor..."

  for pid in "${PX4_PIDS[@]}"; do
    echo "PX4 PID $pid sonlandırılıyor..."
    kill -9 $pid 2>/dev/null
  done

  for pid in "${MAVSDK_PIDS[@]}"; do
    echo "MAVSDK PID $pid sonlandırılıyor..."
    kill -9 $pid 2>/dev/null
  done

  echo "✅ Tüm süreçler başarıyla kapatıldı. Görüşmek üzere!"
  exit 0
}

trap temizlik SIGINT

# Script sonsuza kadar açık kalsın
while true; do sleep 1; done

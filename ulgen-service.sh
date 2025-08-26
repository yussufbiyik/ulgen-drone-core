#!/bin/bash

# 30 saniye için wifi bağlantısını bekle
timeout=30
interval=1
elapsed=0
until ping -c1 8.8.8.8 &>/dev/null; do
    echo "Wifi bekleniyor..."
    sleep $interval
    elapsed=$((elapsed + interval))
    if [ $elapsed -ge $timeout ]; then
        echo "30 saniye geçti, wifi olmadan devam ediliyor..."
        break
    fi
done

cd "$HOME/drone-core" || exit 1
 
# Güncellemeleri al
git reset --hard
git pull

# Servis scriptini çalıştır
python "$HOME/drone-core/service.py"

# sudo mkdir -p ~/.ssh && sudo chmod 700 ~/.ssh && sudo touch ~/.ssh/known_hosts && sudo chmod 600 ~/.ssh/known_hosts && sudo chown pi:pi ~/.ssh ~/.ssh/known_hosts && sudo mv Desktop/id_ed25519 ~/.ssh && sudo chmod 600 ~/.ssh/id_ed25519 && git clone git@github.com:ULGEN-Suru-IHA-Takimi/drone-core.git && cd drone-core && git checkout sartname_gorevleri 
# cd drone-core && conda activate mavsdk && python service.py
# @reboot ~/ulgen-service.sh >> ~/log.txt 2>&1
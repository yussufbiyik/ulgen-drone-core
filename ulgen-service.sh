#!/bin/bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate mavsdk
cd "$HOME/drone-core" || exit 1

# Güncellemeleri al
git reset --hard
git pull

# Servis scriptini çalıştır
exec python "$HOME/drone-core/service.py"

# sudo mkdir -p ~/.ssh && sudo mv Desktop/id_ed25519 ~/.ssh && chmod 600 ~/.ssh/id_ed25519 && cd drone-core && git checkout sartname_gorevleri && 
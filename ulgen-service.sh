#!/bin/bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate mavsdk
cd "$HOME/ulgen/drone-core" || exit 1

# Güncellemeleri al
git pull

# Servis scriptini çalıştır
exec python "$HOME/ulgen/drone-core/service.py"
import math

def distance_meters(lat1, lon1, lat2, lon2):
    """
    Haversine formülü ile mesafe hesaplama
    """
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
            math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def latlon_to_ned(target_lat, target_lon, current_lat, current_lon):
    """
    GPS koordinatlarını NED düzlemine çevir
    """
    d_north = distance_meters(current_lat, current_lon, target_lat, current_lon)
    d_east = distance_meters(current_lat, current_lon, current_lat, target_lon)
    if target_lat < current_lat:
        d_north *= -1
    if target_lon < current_lon:
        d_east *= -1
    return d_north, d_east

def ned_to_latlon(d_north, d_east, current_lat, current_lon):
    """
    NED düzleminden GPS koordinatlarına dönüşüm
    """
    R = 6371000  # Dünya'nın yarıçapı
    new_lat = current_lat + (d_north / R) * (180 / math.pi)
    new_lon = current_lon + (d_east / (R * math.cos(math.radians(current_lat)))) * (180 / math.pi)
    return new_lat, new_lon

def detect_pose(self_pose, neighbor_poses):
    """
    Dronlar arası açıları ve mesafeleri hesaplar.
    """
    for neighbor in neighbor_poses:
        # Dronlar arası mesafeyi hesapla
        distance = distance_meters(self_pose["latitude"], self_pose["longitude"],
                                   neighbor["latitude"], neighbor["longitude"])
        # Dronlar arası açıyı hesapla
        angle = math.atan2(neighbor["longitude"] - self_pose["longitude"],
                           neighbor["latitude"] - self_pose["latitude"])
import copy
import math
import logging

def calculate_formation_positions(formation_type, d):
    if formation_type == "v":
        return [(0, d), (-d, 0), (d, 0)]
    elif formation_type == "cizgi":
        return [(-d, 0), (0, 0), (d, 0)]
    elif formation_type == "ok":
        return [(0, -d), (d, 0), (-d, 0)]

def distance_meters(pos1, pos2):
    """
    Haversine formülü ile mesafe hesaplama
    """
    R = 6371000
    lat1, lon1 = pos1['latitude'], pos1['longitude']
    lat2, lon2 = pos2['latitude'], pos2['longitude']
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
            math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def latlon_to_ned(target_pos, current_pos):
    """
    GPS koordinatlarını NED düzlemine çevir
    """
    d_north = distance_meters(current_pos, {
        'latitude': target_pos['latitude'],
        'longitude': current_pos['longitude']
    })
    d_east = distance_meters(current_pos, {
        'latitude': current_pos['latitude'],
        'longitude': target_pos['longitude']
    })
    if target_pos['latitude'] < current_pos['latitude']:
        d_north *= -1
    if target_pos['longitude'] < current_pos['longitude']:
        d_east *= -1
    return d_north, d_east

def get_distances_and_angles(current_pos, target_pos):
    """
    Mevcut ve hedef konumlar arasındaki mesafe ve açıları hesaplar.
    """
    d_north, d_east = latlon_to_ned(target_pos, current_pos)
    distance = math.sqrt(d_north**2 + d_east**2)
    angle = math.atan2(d_east, d_north)
    return d_north, d_east, distance, angle

def ned_to_latlon(d_north, d_east, current_lat, current_lon):
    """
    NED düzleminden GPS koordinatlarına dönüşüm
    """
    R = 6371000  # Dünya'nın yarıçapı
    new_lat = current_lat + (d_north / R) * (180 / math.pi)
    new_lon = current_lon + (d_east / (R * math.cos(math.radians(current_lat)))) * (180 / math.pi)
    return new_lat, new_lon

def calculate_formation_weight_center(self_position, neighbors):
    """
    Formasyon merkezini hesaplar.
    """
    if not neighbors:
        return self_position
    total_lat = self_position["latitude"]
    total_lon = self_position["longitude"]
    count = 1 

    for neighbor in neighbors:
        total_lat += neighbor["data"]["gps_position"]["latitude"]
        total_lon += neighbor["data"]["gps_position"]["longitude"]
        count += 1

    center_lat = total_lat / count
    center_lon = total_lon / count

    return {'latitude': center_lat, 'longitude': center_lon}

def calculate_ideal_formation_positions(formation_type, center_position, d):
    """
    Belirtilen formasyon tipine göre ideal konumları hesaplar.
    """
    if formation_type not in ["v", "cizgi", "ok"]:
        raise ValueError("Geçersiz formasyon tipi desteklenmiyor.")
    positions = calculate_formation_positions(formation_type, d)
    ideal_positions = []
    
    for pos in positions:
        lat_offset, lon_offset = pos
        new_lat, new_lon = ned_to_latlon(lat_offset, lon_offset, center_position['latitude'], center_position['longitude'])
        ideal_positions.append({'latitude': new_lat, 'longitude': new_lon})
    
    return ideal_positions

def assign_position(formation_positions, current_position, drone_id, neighbors=[]):
    """
    Verilen formasyon pozisyonlarından boş olup en kısa mesafede olanı döndürür.
    """
    available_positions = copy.deepcopy(formation_positions)
    assignments = {}
    all_drones = sorted([
        {"sender": drone_id, "data": {"gps_position": current_position}},
        *neighbors
    ], key=lambda drone: int(drone["sender"]))
    
    for drone in all_drones:
        closest_position_of_drone = min(available_positions, key=lambda pos: distance_meters(pos, drone["data"]["gps_position"]))
        assignments[drone["sender"]] = closest_position_of_drone
        available_positions.remove(closest_position_of_drone)
    
    closest_position = assignments[drone_id]
    return closest_position, assignments
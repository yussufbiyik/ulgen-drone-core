import copy
import math
import logging

def calculate_formation_positions(formation_type, d):
    h = d * math.sin(math.radians(60))
    if formation_type == "v":
        return [
            (0, 0),           # merkez (uç)
            (-d/2, -h),       # sol kanat
            (d/2, -h)         # sağ kanat
        ]
    elif formation_type == "cizgi":
        return [(0, -d), (0, 0), (0, d)]
    elif formation_type == "ok":
        return [
            (0, 0),           # merkez (uç)
            (-d/2, h),       # sol kanat
            (d/2, h)         # sağ kanat
        ]

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
        raise ValueError("Geçersiz formasyon tipi, desteklenmiyor.")
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
    available_positions = sorted(
        formation_positions,
        key=lambda pos: pos['latitude'] + pos['longitude']
    )
    original_drones = sorted(
        [{"sender": drone_id, "data": {"gps_position": current_position}}, *neighbors],
        key=lambda drone: int(drone["sender"])
    )
    all_drones = original_drones.copy()
    
    assignments = {}
    for position in available_positions:
        closest_drone = min(
            all_drones,
            key=lambda drone: (
                distance_meters(drone["data"]["gps_position"], position),
                original_drones.index(drone)  # fixed tie-break
            )
        )
        assignments[closest_drone["sender"]] = position
        all_drones.remove(closest_drone)

    return assignments[drone_id], assignments
# def assign_position(formation_positions, current_position, drone_id, neighbors=[]):
#     """
#     Verilen formasyon pozisyonlarından boş olup en kısa mesafede olanı döndürür.
#     """
#     available_positions = copy.deepcopy(formation_positions)
#     all_drones = sorted([
#         {"sender": drone_id, "data": {"gps_position": current_position}},
#         *neighbors
#     ], key=lambda drone: int(drone["sender"]))
#     assignments = {}
#     sorted_positions = sorted([
#         {"latitude": pos['latitude'], "longitude": pos['longitude']}
#         for pos in available_positions
#     ], key=lambda pos: pos['latitude'] + pos['longitude'])
#     for position in sorted_positions:
#         closest_drone = min(
#             all_drones,
#             key=lambda drone: (
#                 distance_meters(drone["data"]["gps_position"], position),
#                 all_drones.index(drone)
#             )
#         )
#         logging.debug(f"Dron {closest_drone['sender']} için en yakın pozisyon: {position}, mesafe: {distance_meters(closest_drone['data']['gps_position'], position)}")
#         assignments[closest_drone["sender"]] = position
#         all_drones.remove(closest_drone)
#     closest_position = assignments[drone_id]
#     logging.info(f"Dron {drone_id} için en yakın pozisyon: {closest_position}")
#     return closest_position, assignments
    
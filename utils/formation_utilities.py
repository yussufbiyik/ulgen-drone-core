import copy
import math
import logging

def calculate_formation_positions(formation_type, d):
    h = d * math.sin(math.radians(60))
    if formation_type == "ok":
        return [
            (0, 0),           # merkez (uç)
            (-d/2, -h),       # sol kanat
            (d/2, -h)         # sağ kanat
        ]
    elif formation_type == "cizgi":
        return [(0, -d), (0, 0), (0, d)]
    elif formation_type == "v":
        return [
            (0, 0),           # merkez (uç)
            (-d/2, h),        # sol kanat
            (d/2, h)          # sağ kanat
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
    if not neighbors:
        logging.debug("Aktif komşu drone bulunamadı.")
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
        rot_n, rot_e = rotate_position(lat_offset, lon_offset, -18)
        new_lat, new_lon = ned_to_latlon(rot_n, rot_e, center_position['latitude'], center_position['longitude'])
        ideal_positions.append({'latitude': new_lat, 'longitude': new_lon})
    
    return ideal_positions

def assign_position(formation_positions, current_position, drone_id, neighbors=[]):
    """
    Her adımda tüm dronlar ve pozisyonlar arasındaki en kısa mesafeyi bulur, 
    o drona o pozisyonu atar, sonra tekrar eder.
    """
    # Tüm drone bilgilerini topla
    drones = sorted(
        [{"sender": drone_id, "data": {"gps_position": current_position}}, *neighbors],
        key=lambda drone: int(drone["sender"])
    )
    available_positions = sorted(
        formation_positions,
        key=lambda pos: pos['latitude'] + pos['longitude']
    )
    available_drones = drones.copy()
    assignments = {}

    while available_positions and available_drones:
        # Tüm (drone, position) çiftleri için mesafeyi hesapla
        all_pairs = [
            (drone, pos, distance_meters(drone["data"]["gps_position"], pos))
            for drone in available_drones
            for pos in available_positions
        ]
        # Mesafeye göre en küçük çifti seç (eşitlikte drone ID küçük olana ver)
        drone, pos, dist = min(
            all_pairs,
            key=lambda x: (x[2], int(x[0]["sender"]))
        )

        assignments[drone["sender"]] = pos
        logging.debug(
            f"Drone {drone['sender']} için seçilen pozisyon: {pos}, mesafe: {dist}"
        )

        # Kullanılan dronu ve pozisyonu listeden çıkar
        available_drones.remove(drone)
        available_positions.remove(pos)

    my_position = assignments[drone_id]
    logging.info(f"Drone {drone_id} için atanan pozisyon: {my_position}")
    return my_position, assignments

def rotate_position(d_n, d_e, angle):
    """
    Verilen NED cinsi konumu belirtilen açı kadar döndürür.
    """
    theta = math.radians(angle)
    xr = d_n * math.cos(theta) - d_e * math.sin(theta)
    yr = d_n * math.sin(theta) + d_e * math.cos(theta)
    return xr, yr

def wrap_number_in_range(number, range):
    """
    Verilen sayıyı belirtilen aralığa sınırlar.
    """
    while number > 180:
        number -= 360
    while number < -180:
        number += 360
    return number

def angle_between(position1, position2):
    """
    İki konum arasındaki açıyı hesaplar.
    """
    d_north, d_east, _, _ = get_distances_and_angles(position1, position2)
    return math.degrees(math.atan2(d_east, d_north))
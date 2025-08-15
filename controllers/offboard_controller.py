import math
import logging
import asyncio

from mavsdk.offboard import OffboardError, VelocityNedYaw, VelocityBodyYawspeed

from utils.formation_utilities import latlon_to_ned, get_distances_and_angles, wrap_number_in_range

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

class OffboardController:
    def __init__(self, drone):
        self.drone = drone
        self.prev_position = None
        self.prev_ratio = 0.0
        self.time_elapsed_since_last_target = 0.0
        self.initial_navigation_duration = None
        self.initial_distance_to_target = None

    def clamp_velocity(self, v, limit=1.0):
        """
        Hızı sınırlar.
        :param v: Hız değeri
        :param limit: Sınır değeri
        :return: Sınırlanmış hız
        """
        return max(-limit, min(limit, v))

    async def apf_controller(self):
        """
        APF: Komşu dronelardan kaçınmak için hız vektörü üretir.
        """
        current_data = await self.drone.mavsdk_controller.get_general_info()
        current_position = current_data["gps_position"]

        vx, vy = self.drone.apf.compute_apf(current_position, self.drone.neighbors)
        return vx, vy

    # PID ile Navigasyon
    async def pid_controller(self, d_north, d_east, down_distance, distance, angle):
        """
        PID kontrolü: hedef pozisyona yönelmek için hız vektörü üretir.
        """
        dt = 0.1  # Sabit güncelleme süresi
        speed = self.drone.pid_ne.compute(distance, dt)
        down_speed = self.drone.pid_d.compute(down_distance, dt)
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)
        return vx, vy, down_speed
    # Standart Mod ile Navigasyon
    async def smooth_navigate(self, d_north, d_east, distance, angle, max_speed, navigation_duration = None):
        """
        Hedef konuma yumuşak bir gaz-fren profili ile ilerler
        Navigasyon süresi varsa max_speed değişkeni aşılmayacak şekilde hızını ayarlar,
        aksi halde sabit hızda ilerler.

        :param current_location: Mevcut konum
        :param target_location: Hedef konum
        :param max_speed: Maksimum hız
        :param navigation_duration: Navigasyon süresi
        """
        # Optimal duruş mesafesi, maksimum hızda 1 saniyede katedilen mesafe olarak belirlenir.
        # Bu, aracın yavaşlamaya başlayacağı "frenleme bölgesini" tanımlar.
        optimal_stop_distance = max_speed * 1.0

        if distance > optimal_stop_distance:
            # Eğer hedefe olan mesafe frenleme bölgesinden büyükse, maksimum hızla ilerle.
            # Hız vektörü, hedef yönündeki birim vektörün max_speed ile çarpılmasıyla bulunur.
            # (d_north / distance) ve (d_east / distance) ifadeleri yön vektörünü normalize eder.
            vx = (d_north / distance) * max_speed
            vy = (d_east / distance) * max_speed
        else:
            # Frenleme bölgesine girildiğinde, hız hedefe olan mesafe ile orantılı olarak azaltılır.
            # Bu, hedefe yaklaştıkça aracın yavaşlamasını sağlayan basit bir P-kontrolördür (P=0.1).
            # Bu sayede hedefe yumuşak bir şekilde varılır ve hedefi geçme riski azalır.
            vx = d_north * 0.5
            vy = d_east * 0.5
        
        return vx, vy

    async def background_offboard_controller(self):
        while True:
            if not self.drone.offboard_status["is_active"]:
                logging.debug("OffboardController kapalı, kontrol döngüsü atlanıyor.")
                await asyncio.sleep(0.1)
                continue

            if not await self.drone.mavsdk_controller.mavsdk.offboard.is_active():
                try:
                    yaw = await self.drone.mavsdk_controller.get_yaw()
                    await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, yaw)
                    )
                    await self.drone.mavsdk_controller.mavsdk.offboard.start()
                    logging.debug("Offboard modu başlatıldı.")
                except OffboardError as e:
                    logging.error(f"Offboard moduna geçiş başarısız: {e}")
                    await asyncio.sleep(0.1)
                    continue

            target_pos = self.drone.offboard_status.get("target_position")

            if target_pos is None:
                logging.debug("Hedef konum ayarlanmamış. Hover modu aktif.")
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, await self.drone.mavsdk_controller.get_yaw())
                )
                await asyncio.sleep(0.1)
                continue
            current_data = await self.drone.mavsdk_controller.get_general_info()
            current_position = current_data["gps_position"]
            down_distance = current_position["altitude"] - self.drone.offboard_status["altitude_to_keep"]
            d_north, d_east, distance, angle = get_distances_and_angles(current_position, target_pos)

            # Hedef konum değişmişse
            if self.prev_position != target_pos:
                self.time_elapsed_since_last_target = 0.0
                self.prev_position = target_pos
                self.prev_ratio = 0.0
                self.drone.pid_ne.reset()
                self.drone.pid_d.reset()
                # Hedefe doğru bir yayı izler gibi yumuşak dönüş yap
                while True:
                    current_data = await self.drone.mavsdk_controller.get_general_info()
                    current_position = current_data["gps_position"]
                    d_north, d_east, distance, angle = get_distances_and_angles(current_position, self.drone.formation["weight_center"])
                    current_yaw_deg = current_data["attitude"]["yaw"]
                    target_yaw_deg = math.degrees(angle)
                    distance = math.degrees(distance)
                    yaw_error = wrap_number_in_range((target_yaw_deg - current_yaw_deg), [-180, 180])
                    logging.info(f"Hedefe doğru yönelme: Hedef Yaw: {target_yaw_deg}, Mevcut Yaw: {current_yaw_deg}, Hata: {yaw_error}")
                    yaw_rate = yaw_error * 0.5
                    if(abs(yaw_error) < 3):
                        logging.info("Hedefe doğru yönelme tamamlandı.")
                        break
                    await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_body(
                        VelocityBodyYawspeed(1.0, 0.0, 0.0, yaw_rate)
                    )
                    await asyncio.sleep(0.1)
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw)
                )

            if self.drone.offboard_status["navigation_method"] == "standard":
                # Standart navigasyon
                vx, vy = await self.smooth_navigate(
                    d_north, d_east, distance, angle,
                    self.drone.speed_limit
                )
            else:
                # PID ve APF ile hızları hesapla
                vx, vy, down_speed = await self.pid_controller(
                    d_north, d_east, down_distance, distance, angle
                )
                # logging.info(f"PID Hız: vx={vx}, vy={vy}")
            apf_vx, apf_vy = await self.apf_controller()

            # Hızları birleştir ve sınırla
            vx = self.clamp_velocity(vx, self.drone.speed_limit) - apf_vx
            vy = self.clamp_velocity(vy, self.drone.speed_limit) - apf_vy
            # logging.info(f"Son Hız: vx={vx}, vy={vy}, APF: vx={apf_vx}, vy={apf_vy}")
            # Dronun gittiği yöne doğru önünü dönmesi için
            if distance > self.drone.waypoint_threshold:
                yaw = math.degrees(math.atan2(vy, vx)) if vx != 0 or vy != 0 else 0.0
            else:
                yaw = await self.drone.mavsdk_controller.get_yaw()

            try:
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=min(down_speed, 0.5), yaw_deg=yaw)
                )
            except OffboardError as e:
                logging.error(f"Hız vektörü ayarlanamadı: {e}")
            
            self.time_elapsed_since_last_target += 0.1
            await asyncio.sleep(0.1)
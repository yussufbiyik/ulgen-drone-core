import math
import logging
import asyncio

from mavsdk.offboard import OffboardError, VelocityNedYaw

from utils.formation_utilities import latlon_to_ned

logging.basicConfig(level=logging.INFO, format='[%(asctime)s - %(levelname)s]:\n\t%(message)s')

class OffboardController:
    def __init__(self, drone):
        self.drone = drone

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

    async def pid_controller(self, target_position):
        """
        PID kontrolü: hedef pozisyona yönelmek için hız vektörü üretir.
        """
        current_data = await self.drone.mavsdk_controller.get_general_info()
        current_position = current_data["gps_position"]

        d_north, d_east = latlon_to_ned(
            target_position["latitude"], target_position["longitude"],
            current_position["latitude"], current_position["longitude"]
        )
        distance = math.sqrt(d_north**2 + d_east**2)
        angle = math.atan2(d_east, d_north)

        dt = 0.05  # Sabit güncelleme süresi
        speed = self.drone.pid_ne.compute(distance, dt)
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)
        return vx, vy, d_north, d_east, distance

    async def background_offboard_controller(self):
        while True:
            if not self.drone.offboard_status["is_active"]:
                logging.debug("OffboardController kapalı, kontrol döngüsü atlanıyor.")
                await asyncio.sleep(0.5)
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
                    await asyncio.sleep(0.5)
                    continue

            target_pos = self.drone.offboard_status.get("target_position")

            if target_pos is None:
                logging.warning("Hedef konum ayarlanmamış. Hover moduna geçiliyor.")
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, await self.drone.mavsdk_controller.get_yaw())
                )
                await asyncio.sleep(0.05)
                continue

            # PID ve APF ile hızları hesapla
            pid_vx, pid_vy, d_north, d_east, distance = await self.pid_controller(target_pos)
            apf_vx, apf_vy = await self.apf_controller()

            # Hızları birleştir ve sınırla
            vx = self.clamp_velocity(pid_vx) + apf_vx
            vy = self.clamp_velocity(pid_vy) + apf_vy

            # Dronun gittiği yöne doğru önünü dönmesi için
            if distance > self.drone.waypoint_threshold:
                yaw = math.degrees(math.atan2(vy, vx)) if vx != 0 or vy != 0 else 0.0
            else:
                yaw = await self.drone.mavsdk_controller.get_yaw()

            try:
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=0.0, yaw_deg=yaw)
                )
            except OffboardError as e:
                logging.error(f"Hız vektörü ayarlanamadı: {e}")

            await asyncio.sleep(0.05)
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

        dt = 0.1  # Sabit güncelleme süresi
        vx = self.drone.pid_n.compute(d_north, dt)
        vy = self.drone.pid_e.compute(d_east, dt)
        return vx, vy

    async def background_offboard_controller(self):
        while True:
            if not self.drone.offboard_status["is_active"]:
                logging.debug("OffboardController kapalı, kontrol döngüsü atlanıyor.")
                await asyncio.sleep(0.1)
                continue

            if not await self.drone.mavsdk_controller.mavsdk.offboard.is_active():
                try:
                    await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                    await self.drone.mavsdk_controller.mavsdk.offboard.start()
                    logging.debug("Offboard modu başlatıldı.")
                except OffboardError as e:
                    logging.error(f"Offboard moduna geçiş başarısız: {e}")
                    await asyncio.sleep(0.5)
                    continue

            target_pos = self.drone.offboard_status.get("target_position")
            alt_to_keep = self.drone.offboard_status.get("altitude_to_keep")

            if target_pos is None:
                logging.warning("Hedef konum ayarlanmamış. Hover moduna geçiliyor.")
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.1)
                continue

            # Güncel konum bilgisi
            current_data = await self.drone.mavsdk_controller.get_general_info()
            current_position = current_data["gps_position"]

            # PID ve APF ile hızları hesapla
            pid_vx, pid_vy = await self.pid_controller(target_pos)
            apf_vx, apf_vy = await self.apf_controller()

            # İrtifa kontrolü
            error_z = alt_to_keep - current_position["altitude"]
            vz = self.drone.pid_z.compute(error_z, 0.1)

            # Hızları birleştir ve sınırla
            vx = self.clamp_velocity(pid_vx) + apf_vx
            vy = self.clamp_velocity(pid_vy) + apf_vy
            vz = self.clamp_velocity(vz)

            try:
                await self.drone.mavsdk_controller.mavsdk.offboard.set_velocity_ned(
                    VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=-vz, yaw_deg=0.0)
                )
            except OffboardError as e:
                logging.error(f"Hız vektörü ayarlanamadı: {e}")

            await asyncio.sleep(0.1)
import logging
import asyncio

from core.mission import Mission

from controllers.step_controller import Step
from controllers.drone_controller import DroneController



class FormationMission(Mission):
    def __init__(self, drone: DroneController, **kwargs):
        super().__init__("3B Formasyon Görevi", drone, **kwargs)
        self.active_formation = self.parameters.get("active_formation", "3B")
        self.formation_wait_time = self.parameters.get("formation_wait_time", 10.0)
    
    async def run(self):
        await super().run()
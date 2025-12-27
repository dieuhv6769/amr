from PyQt5.QtCore import QRectF

import c_amr.constants as const

class ObstacleModel:
    def __init__(self, rect: QRectF):
        self.full_rect = rect
        self.walkable_margin = const.WALKABLE_MARGIN_PIXELS
        self.stations = []
        self.solid_rect = self.calculate_solid_rect()

    def calculate_solid_rect(self):
        return self.full_rect.adjusted(
            self.walkable_margin, self.walkable_margin,
            -self.walkable_margin, -self.walkable_margin
        )
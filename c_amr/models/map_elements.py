from PyQt5.QtCore import QPointF

class NodeModel:
    def __init__(self, node_id: int, pos: QPointF):
        self.node_id = node_id
        self.pos = pos

class StationModel:
    def __init__(self, station_id: int, pos: QPointF, parent_obstacle=None):
        self.station_id = station_id
        self.pos = pos
        self.parent_obstacle = parent_obstacle
import random
import math
from PyQt5.QtCore import QPointF, QRectF

import c_amr.constants as const

class RobotModel:
    def __init__(self, robot_id: int, pos: QPointF):
        self.robot_id = robot_id
        self.pos = pos
        self.dx = 0.0
        self.dy = 0.0
        self.heading = random.uniform(0, 360)
        self.tv = 0.0
        self.rv = 0.0
        self.soc = 100.0
        self.np = 99.9
        self.cp = 0.1
        self.fd = 0.0
        self.is_c_amr = False
        self.robot_size = const.DEFAULT_ROBOT_SIZE_M * const.PIXELS_PER_METER
        self.markov_radius = 3.0
        
        self.state = "IDLE"
        self.path = []
        self.path_index = 0
        self.target_pos = None
        self.current_node_id = None
        self.path_changed = False
        self.ignore_obstacle_collisions = True

    def update_movement(self, map_rect: QRectF, obstacles: list, model):
        self._update_ai_state_and_movement(model)
        
        previous_pos = self.pos
        
        if self.state == "MOVING":
            self._move_along_path()
        else:
            self.dx = 0.0
            self.dy = 0.0
            self.tv = 0.0
            self.rv = 0.0
            
        proposed_pos = QPointF(previous_pos.x() + self.dx, previous_pos.y() + self.dy)
        
        self.pos = self._handle_collisions(proposed_pos, map_rect, obstacles)
        
        if self.tv > 0.01:
             self.heading = math.degrees(math.atan2(self.dy, self.dx))

    def _update_ai_state_and_movement(self, model):
        algo = model.algorithm_handler
        if not algo:
            return

        if self.state == "IDLE":
            self.state = "PLANNING"
            
            if model.stations:
                random_station = random.choice(model.stations)
                
                approach_pos = self._calculate_approach_point(random_station)
                destination_pos = approach_pos
                
                path = algo.plan_path_graph_astar(self.pos, destination_pos)
                
                if path:
                    path.append(random_station.pos)
                    self.set_new_path(path)
                else:
                    self.state = "IDLE" 
            else:
                self.state = "IDLE"
        
    def _move_along_path(self):
        if not self.path or self.path_index >= len(self.path):
            self.state = "IDLE"
            self.dx = 0.0
            self.dy = 0.0
            self.tv = 0.0
            return

        self.target_pos = self.path[self.path_index]
        
        dx_vec = self.target_pos.x() - self.pos.x()
        dy_vec = self.target_pos.y() - self.pos.y()
        dist_to_target = math.hypot(dx_vec, dy_vec)

        speed_px_per_tick = const.RANDOM_TV_MAX_MPS * const.PIXELS_PER_METER * (const.UPDATE_INTERVAL_MS / 1000.0)

        if dist_to_target < const.WAYPOINT_ARRIVAL_TOLERANCE_PX or dist_to_target < speed_px_per_tick:
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.state = "IDLE"
                self.dx, self.dy, self.tv = 0, 0, 0
        else:
            angle_rad = math.atan2(dy_vec, dx_vec)
            self.dx = speed_px_per_tick * math.cos(angle_rad)
            self.dy = speed_px_per_tick * math.sin(angle_rad)
            self.tv = const.RANDOM_TV_MAX_MPS
            
    def _handle_collisions(self, proposed_pos: QPointF, map_rect: QRectF, obstacles: list) -> QPointF:
        final_pos = proposed_pos
        
        half_size = self.robot_size / 2
        final_pos.setX(max(map_rect.left() + half_size, min(map_rect.right() - half_size, final_pos.x())))
        final_pos.setY(max(map_rect.top() + half_size, min(map_rect.bottom() - half_size, final_pos.y())))

        if self.ignore_obstacle_collisions:
            return final_pos

        for obs in obstacles:
            if obs.solid_rect.intersects(QRectF(final_pos, final_pos).adjusted(-half_size, -half_size, half_size, half_size)):
                return self.pos 

        return final_pos

    def set_new_path(self, path: list):
        if path and len(path) > 1:
            self.path = path
            self.path_index = 1
            self.target_pos = self.path[self.path_index]
            self.state = "MOVING"
            self.path_changed = True
        else:
            self.path = []
            self.path_index = 0
            self.target_pos = None
            self.state = "IDLE"

    def _calculate_approach_point(self, station) -> QPointF:
        if not station.parent_obstacle:
            return station.pos

        parent_obs_rect = station.parent_obstacle.full_rect
        station_pos = station.pos
        approach_dist = const.APPROACH_DISTANCE_PX 

        if abs(station_pos.y() - parent_obs_rect.top()) < 1: 
            return QPointF(station_pos.x(), station_pos.y() - approach_dist)
        elif abs(station_pos.y() - parent_obs_rect.bottom()) < 1: 
            return QPointF(station_pos.x(), station_pos.y() + approach_dist)
        elif abs(station_pos.x() - parent_obs_rect.left()) < 1: 
            return QPointF(station_pos.x() - approach_dist, station_pos.y())
        elif abs(station_pos.x() - parent_obs_rect.right()) < 1: 
            return QPointF(station_pos.x() + approach_dist, station_pos.y())
        
        return station.pos

class CentralAMRAgentModel(RobotModel):
    def __init__(self, robot_id: int, pos: QPointF):
        super().__init__(robot_id, pos)
        self.is_c_amr = True
        self.robot_size = const.DEFAULT_ROBOT_SIZE_M * const.PIXELS_PER_METER * 1.2
        self.target_station_id = -1
        
        self.battery_capacity = const.DEFAULT_BATTERY_CAPACITY
        self.battery = self.battery_capacity

    def set_new_path(self, path: list, target_station_id: int = -1):
        self.target_station_id = target_station_id
        self.current_node_id = None
        super().set_new_path(path)

    def update_movement(self, map_rect: QRectF, obstacles: list, model):
        if not self.path:
            self.state = "IDLE"
            self.dx, self.dy, self.tv = 0, 0, 0
            return

        previous_pos = self.pos
        if self.state == "MOVING":
            self._move_along_path()
        else:
            self.dx, self.dy, self.tv = 0, 0, 0
            
        proposed_pos = QPointF(previous_pos.x() + self.dx, previous_pos.y() + self.dy)
        
        self.pos = self._handle_collisions(proposed_pos, map_rect, obstacles)
        
        if self.tv > 0.01:
             self.heading = math.degrees(math.atan2(self.dy, self.dx))


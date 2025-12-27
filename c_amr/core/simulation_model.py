# c_amr/core/simulation_model.py
import random
import math
from PyQt5.QtCore import QObject, pyqtSignal, QRectF, QPointF

import c_amr.constants as const
from c_amr.models.obstacle_model import ObstacleModel
from c_amr.models.robot_model import RobotModel, CentralAMRAgentModel
from c_amr.models.map_elements import NodeModel, StationModel
from c_amr.algorithms.amr_algorithms import AMR_Algorithms
from c_amr.algorithms.interaction_logic import InteractionLogic
from c_amr.utils.map_generator import generate_detail_traffic_map_layout

class SimulationModel(QObject):
    model_updated = pyqtSignal()
    map_layout_changed = pyqtSignal()
    status_message_changed = pyqtSignal(str, int)
    path_found = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.robots = {}
        self.c_amr = None
        self.obstacles = []
        self.nodes = []
        self.stations = []
        self.map_rect = QRectF()
        self.algorithm_handler = None
        self.interaction_handler = InteractionLogic()
        self.node_id_counter = 1
        self.station_id_counter = 1
        self.path_is_active = False
        self.horizontal_paths = {}
        self.vertical_corridors = {}
        self.intersections = []
        self.lane_transitions = []
        self.traffic_config_params = {}

    def load_new_generated_layout(self, layout_data: dict):
        self._clear_simulation()

        for obs_data in layout_data.get("obstacles", []):
            rect = QRectF(obs_data["x"], obs_data["y"], obs_data["w"], obs_data["h"])
            self.obstacles.append(ObstacleModel(rect))
        
        self.stations = []
        for station_data in layout_data.get("stations", []):
            pos = QPointF(station_data["x"], station_data["y"])
            
            parent_model = None
            parent_index = station_data.get('parent_obstacle_index', -1) 
            
            if 0 <= parent_index < len(self.obstacles):
                parent_model = self.obstacles[parent_index]
            
            station = StationModel(station_data["id"], pos, parent_model)
            
            if parent_model:
                parent_model.stations.append(station)
                
            self.stations.append(station)
            if station.station_id >= self.station_id_counter:
                self.station_id_counter = station.station_id + 1
        
        traffic_data = layout_data.get("traffic_network", {})
        self.horizontal_paths = traffic_data.get("horizontal_paths", {})
        self.vertical_corridors = traffic_data.get("vertical_corridors", {})
        self.intersections = traffic_data.get("intersections", [])
        self.lane_transitions = traffic_data.get("lane_transitions", [])
        self.traffic_config_params = traffic_data.get("config_params", {})

        if self.obstacles:
            all_rects = [obs.full_rect for obs in self.obstacles]
            bounding_rect = all_rects[0]
            for r in all_rects[1:]:
                bounding_rect = bounding_rect.united(r)
            self.map_rect = bounding_rect.adjusted(-50, -50, 50, 50)

        self._generate_robots()

        self.update_algorithm_handler()
        
        self.map_layout_changed.emit()
        self.model_updated.emit()

    def update_simulation_tick(self):
        for robot in self.robots.values():
            robot.update_movement(self.map_rect, self.obstacles, self)

        if self.c_amr and self.path_is_active and not self.c_amr.path:
            self.path_found.emit([])
            self.path_is_active = False
        
        self.model_updated.emit()
    
    def restart(self, config: dict):
        map_config = {
            "map_width_m": config.get("map_width_m", const.DEFAULT_MAP_WIDTH_M),
            "map_height_m": config.get("map_height_m", const.DEFAULT_MAP_HEIGHT_M),
            "markov_radius_m": config.get("markov_radius_m", 3.0),
        }
        
        layout_data = generate_detail_traffic_map_layout(map_config)

        if layout_data:
            self.load_new_generated_layout(layout_data)
            if self.c_amr:
                radius_m = config.get("markov_radius_m", 3.0)
                self.set_camr_markov_radius(radius_m)
        else:
            print("ERROR: Could not create default layout on startup.")

    def _clear_simulation(self):
        self._clear_simulation_data()
        self.robots.clear()
        self.c_amr = None

    def _clear_simulation_data(self):
        self.obstacles.clear()
        self.stations.clear()
        self.nodes.clear()
        self.horizontal_paths.clear()
        self.vertical_corridors.clear()
        self.intersections.clear()
        self.lane_transitions.clear()
        self.traffic_config_params.clear()
        self.station_id_counter = 1
        self.node_id_counter = 1
        self.algorithm_handler = None

    def update_algorithm_handler(self):
        self.algorithm_handler = AMR_Algorithms(
            c_amr_agent=self.c_amr, 
            all_obstacles=self.obstacles, 
            map_rect=self.map_rect, 
            horizontal_paths=self.horizontal_paths, 
            vertical_corridors=self.vertical_corridors, 
            intersections=self.intersections
        )

    def _generate_robots(self):
        robot_size = const.DEFAULT_ROBOT_SIZE_M * const.PIXELS_PER_METER
        for i in range(1, const.TOTAL_ROBOT_COUNT + 1):
            
            is_valid_pos = False
            while not is_valid_pos:
                pos = QPointF(random.uniform(0, self.map_rect.width()),
                              random.uniform(0, self.map_rect.height()))
                
                robot_rect = QRectF(pos.x(), pos.y(), robot_size, robot_size)
                
                is_valid_pos = True
                
                if self.obstacles:
                    for obs in self.obstacles:
                        if robot_rect.intersects(obs.solid_rect):
                            is_valid_pos = False
                            break
            
            if i == const.C_AMR_ID:
                robot = CentralAMRAgentModel(i, pos)
                self.c_amr = robot
            else:
                robot = RobotModel(i, pos)
            
            self.robots[i] = robot

    def import_obstacles_from_dict(self, data: dict):
        self._clear_simulation_data()
        for obs_data in data.get("obstacles", []):
            rect = QRectF(obs_data["x"], obs_data["y"], obs_data["w"], obs_data["h"])
            self.obstacles.append(ObstacleModel(rect))
        self.model_updated.emit()
        self.map_layout_changed.emit()

    def import_network_from_dict(self, data: dict):
        self.nodes.clear()
        self.stations.clear()
        self.node_id_counter = 1
        self.station_id_counter = 1
        
        for node_data in data.get("nodes", []):
            pos = QPointF(node_data["x"], node_data["y"])
            node = NodeModel(node_data["id"], pos)
            self.nodes.append(node)
            if node.node_id >= self.node_id_counter:
                self.node_id_counter = node.node_id + 1
        
        for station_data in data.get("stations", []):
            pos = QPointF(station_data["x"], station_data["y"])
            
            parent_model = None
            parent_index = station_data.get('parent_obstacle_index', -1)
            
            if 0 <= parent_index < len(self.obstacles):
                parent_model = self.obstacles[parent_index]
            
            station = StationModel(station_data["id"], pos, parent_model)
            
            if parent_model:
                parent_model.stations.append(station)
                
            self.stations.append(station)
            if station.station_id >= self.station_id_counter:
                self.station_id_counter = station.station_id + 1
                
        self.model_updated.emit()
        self.map_layout_changed.emit()

    def set_camr_target_station(self, station_id: int, algorithm_a_star: bool = True):
        if not self.c_amr or not self.algorithm_handler:
            if self.c_amr:
                self.c_amr.set_new_path([], -1)
            self.path_found.emit([])
            self.path_is_active = False
            return

        if station_id == -1:
            self.c_amr.set_new_path([], -1)
            self.path_found.emit([])
            self.path_is_active = False
            self.status_message_changed.emit("Task cancelled.", 3000)
            return

        target_station = next((s for s in self.stations if s.station_id == station_id), None)
        if not target_station:
            self.status_message_changed.emit(f"Error: Station ID {station_id} not found.", 3000)
            return
        
        approach_pos = self._calculate_approach_point(target_station)
        start_pos = self.c_amr.pos
        
        path = None
        if algorithm_a_star:
            path = self.algorithm_handler.plan_path_graph_astar(start_pos, approach_pos)
            if path:
                path.append(target_station.pos)
        else:
            path = self.algorithm_handler.plan_path_rrt(start_pos, target_station.pos)

        if path:
            self.c_amr.set_new_path(path, station_id)
            self.path_found.emit(path)
            self.path_is_active = True
            self.status_message_changed.emit(f"C-AMR moving to Station {station_id}...", 3000)
        else:
            self.c_amr.set_new_path([], -1)
            self.path_found.emit([])
            self.path_is_active = False
            self.status_message_changed.emit(f"Could not find a lane path to Station {station_id}.", 4000)

    def _calculate_approach_point(self, station: StationModel) -> QPointF:
        if not station.parent_obstacle:
            if self.obstacles:
                parent_obs = min(self.obstacles, key=lambda obs: obs.full_rect.center().manhattanLength() - station.pos.manhattanLength())
                station.parent_obstacle = parent_obs
            else:
                return station.pos

        parent_obs_rect = station.parent_obstacle.full_rect
        station_pos = station.pos
        approach_dist = const.APPROACH_DISTANCE_PX

        tolerance = 5.0 
        if abs(station_pos.y() - parent_obs_rect.top()) < tolerance: 
            return QPointF(station_pos.x(), station_pos.y() - approach_dist)
        elif abs(station_pos.y() - parent_obs_rect.bottom()) < tolerance: 
            return QPointF(station_pos.x(), station_pos.y() + approach_dist)
        elif abs(station_pos.x() - parent_obs_rect.left()) < tolerance: 
            return QPointF(station_pos.x() - approach_dist, station_pos.y())
        elif abs(station_pos.x() - parent_obs_rect.right()) < tolerance: 
            return QPointF(station_pos.x() + approach_dist, station_pos.y())
        
        return station.pos

    def add_station(self, pos: QPointF, parent_obstacle: ObstacleModel):
        station = StationModel(self.station_id_counter, pos, parent_obstacle)
        self.stations.append(station)
        parent_obstacle.stations.append(station)
        self.station_id_counter += 1
        self.model_updated.emit()

    def load_default_map(self):
        config = {
            "map_width_m": const.DEFAULT_MAP_WIDTH_M,
            "map_height_m": const.DEFAULT_MAP_HEIGHT_M,
            "markov_radius_m": 3.0,
        }
        self.restart(config)

    def add_obstacle(self, rect: QRectF):
        self.obstacles.append(ObstacleModel(rect))
        self.model_updated.emit()
        self.map_layout_changed.emit()

    def add_node(self, pos: QPointF):
        self.nodes.append(NodeModel(self.node_id_counter, pos))
        self.node_id_counter += 1
        self.model_updated.emit()
        self.map_layout_changed.emit()

    def export_obstacles_to_dict(self) -> dict:
        return {
            "obstacles": [{"x": o.full_rect.x(), "y": o.full_rect.y(), "w": o.full_rect.width(), "h": o.full_rect.height()} for o in self.obstacles]
        }

    def export_network_to_dict(self) -> dict:
        stations_data = []
        for s in self.stations:
            parent_index = -1
            if s.parent_obstacle and s.parent_obstacle in self.obstacles:
                try:
                    parent_index = self.obstacles.index(s.parent_obstacle)
                except ValueError:
                    parent_index = -1
            stations_data.append({
                "id": s.station_id, 
                "x": s.pos.x(), 
                "y": s.pos.y(),
                "parent_obstacle_index": parent_index
            })

        return {
            "nodes": [{"id": n.node_id, "x": n.pos.x(), "y": n.pos.y()} for n in self.nodes],
            "stations": stations_data
        }
    
    def update_fleet_robot_state(self, robot_id: int, state_data: dict):
        robot = self.robots.get(robot_id)
        if not robot:
            robot = RobotModel(robot_id, state_data["pos"])
            self.robots[robot_id] = robot
        robot.pos = state_data["pos"]
        robot.heading = state_data["heading"]
        robot.tv = state_data["tv"]
        robot.rv = state_data["rv"]
        robot.soc = state_data["soc"]
        robot.np = state_data["np"]
        robot.cp = state_data["cp"]
        robot.fd = state_data["fd"]
        angle_rad = math.radians(robot.heading)
        speed_px = robot.tv * const.PIXELS_PER_METER
        robot.dx = speed_px * math.cos(angle_rad)
        robot.dy = speed_px * math.sin(angle_rad)

    def get_interaction_data(self):
        if not self.c_amr:
            return [], []
        return self.interaction_handler.calculate_markov_interactions(self.c_amr, self.robots)

    def set_camr_markov_radius(self, radius_m: float):
        if self.c_amr:
            self.c_amr.markov_radius = radius_m * const.PIXELS_PER_METER
            self.model_updated.emit()

    def plan_path_for_camr(self, goal_pos: QPointF, use_road_network: bool):
        if not self.c_amr or not self.algorithm_handler:
            self.status_message_changed.emit("C-AMR not available.", 3000)
            return
        start_pos = self.c_amr.pos
        path = None
        if use_road_network:
            path = self.algorithm_handler.plan_path_graph_astar(start_pos, goal_pos)
        else:
            path = self.algorithm_handler.plan_path_rrt(start_pos, goal_pos)
        if path:
            self.c_amr.set_new_path(path)
            self.path_found.emit(path)
            self.path_is_active = True
            self.status_message_changed.emit("Path found!", 3000)
        else:
            self.c_amr.set_new_path([])
            self.path_found.emit([])
            self.path_is_active = False
            self.status_message_changed.emit("Could not find a path.", 4000)


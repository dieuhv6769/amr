# c_amr/utils/map_generator.py

import random
from typing import List, Dict, Tuple, Any
from c_amr.utils.traffic_calculator import TrafficCalculator 
from c_amr.utils.config import Config

Obstacle = Dict[str, float]
Station = Dict[str, any]
PathData = Dict[str, Any]
Intersection = Tuple[float, float]
VerticalCorridors = Dict[float, float]

class MapLayout:
    def __init__(self, config: Config):
        self.config = config
        self.obstacles: List[Obstacle] = []
        self.stations: List[Station] = []

    def create_basic_map_layout(self):
        current_y = self.config.MARGIN_PX
        station_id_counter = 1
        
        while current_y < self.config.MAP_HEIGHT_PX - self.config.MARGIN_PX:
            row_plan: List[Dict[str, float]] = []
            current_x = self.config.MARGIN_PX

            while current_x < self.config.MAP_WIDTH_PX - self.config.MARGIN_PX:
                obs_width = random.choice(self.config.POSSIBLE_WIDTHS_PX)
                if current_x + obs_width > self.config.MAP_WIDTH_PX - self.config.MARGIN_PX: 
                    break
                row_plan.append({'width': obs_width})
                current_x += obs_width + random.choice(self.config.INTRA_ROW_SPACINGS_PX)

            if not row_plan: 
                break
            
            tall_obstacle_indices = [random.randint(0, len(row_plan) - 1)] if row_plan else []
            current_x = self.config.MARGIN_PX
            
            for i, plan in enumerate(row_plan):
                obs_width = plan['width']
                obs_height = self.config.POSSIBLE_OBSTACLE_HEIGHTS_PX[1] if i in tall_obstacle_indices else self.config.POSSIBLE_OBSTACLE_HEIGHTS_PX[0]
                obstacle = {'x': current_x, 'y': current_y, 'w': obs_width, 'h': obs_height}
                self.obstacles.append(obstacle)
                
                obstacle_index = len(self.obstacles) - 1

                s_size = self.config.STATION_SIZE_PX
                s_inset = self.config.STATION_INSET_PX
                x, y, w, h = obstacle['x'], obstacle['y'], obstacle['w'], obstacle['h']
                possible_positions = [
                    (x + w / 2 - s_size / 2, y + s_inset),
                    (x + w / 2 - s_size / 2, y + h - s_inset - s_size),
                    (x + s_inset, y + h / 2 - s_size / 2),
                    (x + w - s_inset - s_size, y + h / 2 - s_size / 2)
                ]
                for pos in random.sample(possible_positions, random.choice([1, 2, 3, 4])):
                    self.stations.append({'id': station_id_counter, 'x': pos[0], 'y': pos[1], 'parent_obstacle_index': obstacle_index})
                    station_id_counter += 1

                current_x += obs_width + random.choice(self.config.INTRA_ROW_SPACINGS_PX)
                
            current_y += self.config.FIXED_ROW_HEIGHT_PX + random.choice(self.config.ROW_SPACINGS_PX)
        
        return self.obstacles, self.stations

def generate_detail_traffic_map_layout(map_config: dict) -> dict:
    map_width_m = map_config.get("map_width_m", 50.0)
    map_height_m = map_config.get("map_height_m", 50.0)
    config = Config(map_width_m, map_height_m)
    
    layout = MapLayout(config)
    obstacles, stations = layout.create_basic_map_layout()

    calculator = TrafficCalculator(obstacles, config)
    calculator.calculate_all()

    traffic_network_data = {
            "horizontal_paths": calculator.horizontal_paths,
            "vertical_corridors": calculator.vertical_corridors,
            "intersections": calculator.intersections,
            "config_params": {
                "NARROW_LANE_THRESHOLD_PX": config.NARROW_LANE_THRESHOLD_PX,
                "STROKE_WIDTH_TRAFFIC": config.STROKE_WIDTH_TRAFFIC,
                "MAP_HEIGHT_PX": config.MAP_HEIGHT_PX
            }
    }

    layout_data = {
        "obstacles": [{"x": o['x'], "y": o['y'], "w": o['w'], "h": o['h']} for o in obstacles],
        "stations": [{"id": s['id'], "x": s['x'], "y": s['y'], 'parent_obstacle_index': s['parent_obstacle_index']} for s in stations],
        "traffic_network": traffic_network_data
    }
    return layout_data

def export_layout_to_svg(output_path: str, layout_data: dict, map_config: dict):
    pass

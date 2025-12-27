
from typing import List, Dict, Tuple, Any, Set

import numpy as np
from c_amr.utils.config import Config

Obstacle = Dict[str, float]
Intersection = Tuple[float, float]
VerticalCorridors = Dict[float, float]
PathData = Dict[str, Any]


class BsplineUtils:
    @staticmethod
    def find_knot_vector(n: int, p: int) -> np.ndarray:
        knot_vector = np.zeros(n + p + 2)
        for i in range(p + 1):
            knot_vector[i] = 0.0
        for i in range(p + 1, n + 1):
            knot_vector[i] = (i - p) / (n - p + 1)
        for i in range(n + 1, n + p + 2):
            knot_vector[i] = 1.0
        return knot_vector

    @staticmethod
    def basis_function(i: int, p: int, u: float, knot_vector: np.ndarray) -> float:
        if p == 0:
            if knot_vector[i] <= u < knot_vector[i + 1]:
                return 1.0
            elif u == 1.0 and knot_vector[i + 1] == 1.0:
                return 1.0
            else:
                return 0.0
        
        denominator1 = knot_vector[i + p] - knot_vector[i]
        term1 = 0
        if denominator1 != 0:
            term1 = ((u - knot_vector[i]) / denominator1) * BsplineUtils.basis_function(i, p - 1, u, knot_vector)

        denominator2 = knot_vector[i + p + 1] - knot_vector[i + 1]
        term2 = 0
        if denominator2 != 0:
            term2 = ((knot_vector[i + p + 1] - u) / denominator2) * BsplineUtils.basis_function(i + 1, p - 1, u, knot_vector)
        return term1 + term2

    @staticmethod
    def bspline_curve(control_points: List[Tuple[float, float]], p: int, num_points: int) -> np.ndarray:
        control_points_np = np.array(control_points)
        n = len(control_points_np) - 1
        knot_vector = BsplineUtils.find_knot_vector(n, p)
        curve_points = []
        domain = np.linspace(knot_vector[p], knot_vector[n + 1], num_points)
        for u in domain:
            curve_point = np.zeros(2)
            for i in range(n + 1):
                basis_val = BsplineUtils.basis_function(i, p, u, knot_vector)
                curve_point += basis_val * control_points_np[i]
            curve_points.append(curve_point)
        return np.array(curve_points)


class TrafficCalculator:
    def __init__(self, obstacles: List[Obstacle], config: Config):
        self.obstacles = obstacles
        self.config = config
        self.horizontal_paths: List[Dict] = []
        self.vertical_corridors: List[Dict] = []
        self.intersections: List[Dict] = []

    def calculate_all(self):
        if not self.obstacles: 
            print("No obstacles provided for traffic calculation.")
            return

        rows = {obs['y']: [] for obs in self.obstacles}
        for obs in self.obstacles: 
            rows[obs['y']].append(obs)

        for y_key in rows: 
            rows[y_key].sort(key=lambda o: o['x'])

        sorted_rows_y = sorted(rows.keys())

        self._calculate_horizontal_paths(rows, sorted_rows_y)
        self._calculate_vertical_corridors()
        self._find_intersections()

    def _calculate_horizontal_paths(self, rows: Dict, sorted_rows_y: List):
        h_id_counter = 0

        if sorted_rows_y:
            first_row_y = sorted_rows_y[0]
            gap_top = first_row_y
            if gap_top >= self.config.MIN_PATH_SPACE_PX:
                path_y = gap_top / 2
                is_wide = gap_top >= self.config.NARROW_LANE_THRESHOLD_PX
                segments = [{'x1': 0, 'x2': self.config.MAP_WIDTH_PX, 'y': path_y, 'is_wide': is_wide, 'gap': gap_top}]
                self.horizontal_paths.append({'id': f'H{h_id_counter}', 'y': path_y, 'segments': segments})
                h_id_counter += 1

        for i in range(len(sorted_rows_y) - 1):
            row1_y, row2_y = sorted_rows_y[i], sorted_rows_y[i+1]
            obstacles_in_row1 = rows[row1_y]
            path_segments = []
            
            for obs in obstacles_in_row1:
                ceiling_y = obs['y'] + obs['h']
                floor_y = row2_y
                gap = floor_y - ceiling_y
                if gap >= self.config.MIN_PATH_SPACE_PX:
                    path_y = ceiling_y + gap / 2
                    is_wide = gap >= self.config.NARROW_LANE_THRESHOLD_PX
                    path_segments.append({'x1': obs['x'], 'x2': obs['x'] + obs['w'], 'y': path_y, 'is_wide': is_wide, 'gap': gap})
            
            if not obstacles_in_row1: continue
            
            first_obs = obstacles_in_row1[0]
            if first_obs['x'] > 0:
                ceiling_y = row1_y + first_obs['h']
                floor_y = row2_y
                gap = floor_y - ceiling_y
                if gap >= self.config.MIN_PATH_SPACE_PX:
                    path_y = ceiling_y + gap / 2
                    is_wide = gap >= self.config.NARROW_LANE_THRESHOLD_PX
                    if is_wide:
                        path_segments.append({'x1': 0, 'x2': first_obs['x'], 'y': path_y, 'is_wide': is_wide, 'gap': gap})
            
            for k in range(len(obstacles_in_row1) - 1):
                obs_left = obstacles_in_row1[k]
                obs_right = obstacles_in_row1[k+1]
                x1, x2 = obs_left['x'] + obs_left['w'], obs_right['x']
                if x2 > x1:
                    max_h = max(obs_left['h'], obs_right['h'])
                    ceiling_y = row1_y + max_h
                    floor_y = row2_y
                    gap = floor_y - ceiling_y
                    if gap >= self.config.MIN_PATH_SPACE_PX:
                        path_y = ceiling_y + gap / 2
                        is_wide = gap >= self.config.NARROW_LANE_THRESHOLD_PX
                        if is_wide:
                            path_segments.append({'x1': x1, 'x2': x2, 'y': path_y, 'is_wide': is_wide, 'gap': gap})
            
            last_obs = obstacles_in_row1[-1]
            if last_obs['x'] + last_obs['w'] < self.config.MAP_WIDTH_PX:
                ceiling_y = row1_y + last_obs['h']
                floor_y = row2_y
                gap = floor_y - ceiling_y
                if gap >= self.config.MIN_PATH_SPACE_PX:
                    path_y = ceiling_y + gap / 2
                    is_wide = gap >= self.config.NARROW_LANE_THRESHOLD_PX
                    if is_wide:
                        path_segments.append({'x1': last_obs['x'] + last_obs['w'], 'x2': self.config.MAP_WIDTH_PX, 'y': path_y, 'is_wide': is_wide, 'gap': gap})

            if path_segments:
                path_segments.sort(key=lambda s: s['x1'])
                path_y_level = path_segments[0]['y']
                self.horizontal_paths.append({'id': f'H{h_id_counter}', 'y': path_y_level, 'segments': path_segments})
                h_id_counter += 1

        if sorted_rows_y:
            last_row_y = sorted_rows_y[-1]
            obstacles_in_last_row = rows[last_row_y]
            max_h = max(obs['h'] for obs in obstacles_in_last_row) if obstacles_in_last_row else 0
            ceiling_bottom = last_row_y + max_h
            gap_bottom = self.config.MAP_HEIGHT_PX - ceiling_bottom
            if gap_bottom >= self.config.MIN_PATH_SPACE_PX:
                path_y = ceiling_bottom + gap_bottom / 2
                is_wide = gap_bottom >= self.config.NARROW_LANE_THRESHOLD_PX
                segments = [{'x1': 0, 'x2': self.config.MAP_WIDTH_PX, 'y': path_y, 'is_wide': is_wide, 'gap': gap_bottom}]
                self.horizontal_paths.append({'id': f'H{h_id_counter}', 'y': path_y, 'segments': segments})
                h_id_counter += 1

    def _calculate_vertical_corridors(self):
        all_x_events: Set[float] = {0, self.config.MAP_WIDTH_PX}
        for obs in self.obstacles: 
            all_x_events.add(obs['x'])
            all_x_events.add(obs['x'] + obs['w'])

        sorted_all_x = sorted(list(all_x_events))

        temp_corridors = {}
        for i in range(len(sorted_all_x) - 1):
            x1, x2 = sorted_all_x[i], sorted_all_x[i+1]
            gap = x2 - x1
            if gap < self.config.MIN_PATH_SPACE_PX:
                continue
            mid_x = x1 + gap / 2
            if all(mid_x < obs['x'] or mid_x > obs['x'] + obs['w'] for obs in self.obstacles):
                temp_corridors[mid_x] = gap
        
        sorted_corridor_x = sorted(temp_corridors.keys())
        for i, x_coord in enumerate(sorted_corridor_x):
            self.vertical_corridors.append({
                'id': f'V{i}',
                'x': x_coord,
                'width': temp_corridors[x_coord]
            })

    def _find_intersections(self):
        processed_intersections = set()
        extension = self.config.NARROW_LANE_THRESHOLD_PX 

        for h_path in self.horizontal_paths:
            for segment in h_path['segments']:
                for v_corridor in self.vertical_corridors:
                    x_corridor, width_v = v_corridor['x'], v_corridor['width']
                    if segment['x1'] <= x_corridor <= segment['x2']:
                        intersection_coord = (round(x_corridor), round(segment['y']))
                        if intersection_coord in processed_intersections:
                            continue
                        self._create_intersection_node(x_corridor, segment, width_v, 'real', None)
                        processed_intersections.add(intersection_coord)
        
        for h_path in self.horizontal_paths:
            for segment in h_path['segments']:
                for v_corridor in self.vertical_corridors:
                    x_corridor, width_v = v_corridor['x'], v_corridor['width']
                    intersection_coord = (round(x_corridor), round(segment['y']))
                    if intersection_coord in processed_intersections:
                        continue 

                    if x_corridor > segment['x2'] and abs(segment['x2'] - x_corridor) < extension:
                        self._create_intersection_node(x_corridor, segment, width_v, 'virtual', 'left')
                        processed_intersections.add(intersection_coord)
                    
                    elif x_corridor < segment['x1'] and abs(segment['x1'] - x_corridor) < extension:
                        self._create_intersection_node(x_corridor, segment, width_v, 'virtual', 'right')
                        processed_intersections.add(intersection_coord)
    
    def _create_intersection_node(self, x_corridor, segment, width_v, intersection_type, virtual_side):
        gap_h = segment['gap']
        y = segment['y']
        is_wide_h = segment['is_wide']
        is_wide_v = width_v >= self.config.NARROW_LANE_THRESHOLD_PX
        
        L_h = gap_h / 4.0 if is_wide_h else 0
        L_v = width_v / 4.0 if is_wide_v else 0
        offset = 20

        points = {
            'center':   {'x': x_corridor, 'y': y},
            'GU':       {'x': x_corridor + L_v, 'y': y - L_h, 'is_wide': is_wide_v, 'type': 'green'},
            'GD':       {'x': x_corridor + L_v, 'y': y + L_h, 'is_wide': is_wide_v, 'type': 'green'},
            'GR':       {'x': x_corridor + L_v, 'y': y - L_h, 'is_wide': is_wide_h, 'type': 'green'},
            'GL':       {'x': x_corridor - L_v, 'y': y - L_h, 'is_wide': is_wide_h, 'type': 'green'},
            'RU':       {'x': x_corridor - L_v, 'y': y - L_h, 'is_wide': is_wide_v, 'type': 'red'},
            'RD':       {'x': x_corridor - L_v, 'y': y + L_h, 'is_wide': is_wide_v, 'type': 'red'},
            'RR':       {'x': x_corridor + L_v, 'y': y + L_h, 'is_wide': is_wide_h, 'type': 'red'},
            'RL':       {'x': x_corridor - L_v, 'y': y + L_h, 'is_wide': is_wide_h, 'type': 'red'},
        }
        points['GU_adj'] = {'x': points['GU']['x'],              'y': points['GU']['y'] - offset,    'type': 'adjacent'}
        points['GD_adj'] = {'x': points['GD']['x'],              'y': points['GD']['y'] + offset,    'type': 'adjacent'}
        points['GR_adj'] = {'x': points['GR']['x'] + offset,     'y': points['GR']['y'],              'type': 'adjacent'}
        points['GL_adj'] = {'x': points['GL']['x'] - offset,     'y': points['GL']['y'],              'type': 'adjacent'}
        points['RU_adj'] = {'x': points['RU']['x'],              'y': points['RU']['y'] - offset,    'type': 'adjacent'}
        points['RD_adj'] = {'x': points['RD']['x'],              'y': points['RD']['y'] + offset,    'type': 'adjacent'}
        points['RR_adj'] = {'x': points['RR']['x'] + offset,     'y': points['RR']['y'],              'type': 'adjacent'}
        points['RL_adj'] = {'x': points['RL']['x'] - offset,     'y': points['RL']['y'],              'type': 'adjacent'}

        intersection_node = {
            'x': x_corridor,
            'y': y,
            'points': points,
            'is_wide_h': is_wide_h,
            'is_wide_v': is_wide_v,
            'type': intersection_type,
            'virtual_side': virtual_side
        }
        self.intersections.append(intersection_node)


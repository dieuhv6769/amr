# c_amr/algorithms/amr_algorithms.py
import math
from PyQt5.QtCore import QPointF, QLineF

import c_amr.constants as const
from c_amr.algorithms.rrt import RRT
from c_amr.algorithms.graph_search import AStarGraphSearch
from c_amr.utils.traffic_calculator import BsplineUtils


class AMR_Algorithms:
    def __init__(self, c_amr_agent, all_obstacles, map_rect,
                 horizontal_paths=None, vertical_corridors=None, intersections=None):
        self.agent = c_amr_agent
        self.all_obstacles = all_obstacles
        self.map_rect = map_rect
        self.horizontal_paths = horizontal_paths if isinstance(horizontal_paths, dict) else {}
        self.vertical_corridors = vertical_corridors if isinstance(vertical_corridors, dict) else {}
        self.intersections = intersections if isinstance(intersections, list) else []

        self.node_positions = {}
        self.road_network_graph = self._build_lane_based_road_network()
        
        if self.road_network_graph:
            self.astar_search = AStarGraphSearch(self.road_network_graph, self.node_positions)
        else:
            self.astar_search = None
            
        self.rrt_search = None 
        self.bspline = BsplineUtils()

    def update_obstacles(self, all_obstacles):
        self.all_obstacles = all_obstacles
        if self.rrt_search:
            self.rrt_search.obstacles = all_obstacles
            
    def _generate_turn_path(self, start_node_id, end_node_id):
        try:
            start_inter_index = int(start_node_id.split('_')[0][1:])
            start_point_key = start_node_id.split('_')[1]
            end_inter_index = int(end_node_id.split('_')[0][1:])
            end_point_key = end_node_id.split('_')[1]
        except (IndexError, ValueError): return None

        if start_inter_index != end_inter_index: return None
        if start_inter_index >= len(self.intersections): return None
            
        intersection_data = self.intersections[start_inter_index]
        p = intersection_data.get('points', {})

        num_points = 20

        turn_type = (start_point_key, end_point_key)
        
        control_points = []
        
        if turn_type in [('RL','GU'), ('GD','GL'), ('GR', 'RD'), ('RU', 'RR')]:
            keys = [f"{start_point_key}_adj", start_point_key, 'center', end_point_key, f"{end_point_key}_adj"]
            
            if not all(key in p for key in keys): 
                return None 
                
            control_points = [
                (p[keys[0]]['x'], p[keys[0]]['y']),
                (p[keys[1]]['x'], p[keys[1]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y']),
                (p[keys[3]]['x'], p[keys[3]]['y']),
                (p[keys[4]]['x'], p[keys[4]]['y']),
            ]
        elif turn_type == ('RL','RD'):
            keys = [f"{start_point_key}_adj", start_point_key, f"{end_point_key}_adj"]
            if not all(key in p for key in keys): return None
            
            control_points = [
                (p[keys[0]]['x'] - 20, p[keys[0]]['y']),
                (p[keys[0]]['x'], p[keys[0]]['y']),
                (p[keys[1]]['x'], p[keys[1]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y'] + 20)
            ]
        elif turn_type == ('GD','RR'):
            keys = [f"{start_point_key}_adj", end_point_key, f"{end_point_key}_adj"]
            if not all(key in p for key in keys): return None
            
            control_points = [
                (p[keys[0]]['x'], p[keys[0]]['y'] + 20),
                (p[keys[0]]['x'], p[keys[0]]['y']),
                (p[keys[1]]['x'], p[keys[1]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y']),
                (p[keys[2]]['x'] + 20, p[keys[2]]['y'])
            ]
        elif turn_type == ('GR','GU'):
            keys = [f"{start_point_key}_adj", start_point_key, f"{end_point_key}_adj"]
            if not all(key in p for key in keys): return None
            
            control_points = [
                (p[keys[0]]['x'] + 20, p[keys[0]]['y']),
                (p[keys[0]]['x'], p[keys[0]]['y']),
                (p[keys[1]]['x'], p[keys[1]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y'] - 20)
            ]
        elif turn_type == ('RU','GL'):
            keys = [f"{start_point_key}_adj", end_point_key, f"{end_point_key}_adj"]
            if not all(key in p for key in keys): return None
            
            control_points = [
                (p[keys[0]]['x'], p[keys[0]]['y'] - 20),
                (p[keys[0]]['x'], p[keys[0]]['y']),
                (p[keys[1]]['x'], p[keys[1]]['y']),
                (p[keys[2]]['x'], p[keys[2]]['y']),
                (p[keys[2]]['x'] - 20, p[keys[2]]['y'])
            ]
        
        if not control_points:
            return None
        
        curve = BsplineUtils.bspline_curve(control_points, p=3, num_points=num_points)
        return [QPointF(p[0], p[1]) for p in curve]

    def _project_point_on_segment(self, p: QPointF, v: QPointF, w: QPointF) -> QPointF:
        l2 = (v.x() - w.x())**2 + (v.y() - w.y())**2
        if l2 == 0.0:
            return v
        t = max(0, min(1, QPointF.dotProduct(p - v, w - v) / l2))
        projection = v + t * (w - v)
        return projection

    def plan_path_rrt(self, start_pos, end_pos):
        self.rrt_search = RRT(start_pos, end_pos, self.all_obstacles, self.map_rect,
                              step_size=const.RRT_STEP_SIZE,
                              max_iterations=const.RRT_MAX_ITERATIONS)
        path = self.rrt_search.find_path()
        return path

    def _remove_loops_from_node_path(self, node_id_path: list) -> list:
        if not node_id_path:
            return []
            
        new_path = []
        node_indices = {} 
        
        for node_id in node_id_path:
            if node_id in node_indices:
                first_index = node_indices[node_id]
                
                nodes_in_loop = new_path[first_index + 1:]
                for looped_node_id in nodes_in_loop:
                    if looped_node_id in node_indices:
                        del node_indices[looped_node_id]
                        
                new_path = new_path[:first_index + 1]
            else:
                new_path.append(node_id)
                node_indices[node_id] = len(new_path) - 1
                
        return new_path

    def plan_path_graph_astar(self, start_pos: QPointF, end_pos: QPointF):
        if not self.road_network_graph or not self.astar_search: 
            print("Warning: A* search or road network not initialized.")
            return None

        start_node_id = self._find_nearest_node_id(start_pos, find_source_node=True)
        end_node_id = self._find_nearest_node_id(end_pos, find_source_node=False) 

        if start_node_id is None or end_node_id is None: 
            print(f"Error: Could not find nearest nodes. Start: {start_node_id}, End: {end_node_id}")
            return None
        if start_node_id == end_node_id: return [start_pos, end_pos]
        
        
        node_id_path = self.astar_search.find_path(start_node_id, end_node_id)

        if not node_id_path: 
            print(f"A* Error: No path found from {start_node_id} to {end_node_id}")
            return None
        
        final_path_points = []
        TOLERANCE = 1e-6 
        SPIKE_TOLERANCE = 1.0 

        if len(node_id_path) >= 2:
            if node_id_path[0] not in self.node_positions or node_id_path[1] not in self.node_positions:
                print(f"Error: Path node IDs {node_id_path[0]} or {node_id_path[1]} not in node_positions.")
                return None
                    
            first_node_pos = self.node_positions[node_id_path[0]]
            second_node_pos = self.node_positions[node_id_path[1]]
            merge_point = self._project_point_on_segment(start_pos, first_node_pos, second_node_pos)
            
            self._append_point_safely(final_path_points, start_pos, TOLERANCE, SPIKE_TOLERANCE)
            
            if math.hypot(start_pos.x() - merge_point.x(), start_pos.y() - merge_point.y()) > 1.0:
                self._append_point_safely(final_path_points, merge_point, TOLERANCE, SPIKE_TOLERANCE)
            
            self._append_point_safely(final_path_points, first_node_pos, TOLERANCE, SPIKE_TOLERANCE)

        else: 
            self._append_point_safely(final_path_points, start_pos, TOLERANCE, SPIKE_TOLERANCE)
            if len(node_id_path) == 1:
                if node_id_path[0] not in self.node_positions:
                    print(f"Error: Path node ID {node_id_path[0]} not in node_positions.")
                    return None
                first_node_pos = self.node_positions[node_id_path[0]]
                self._append_point_safely(final_path_points, first_node_pos, TOLERANCE, SPIKE_TOLERANCE)

        for i in range(len(node_id_path) - 1):
            current_node_id = node_id_path[i]
            next_node_id = node_id_path[i+1]
            
            if current_node_id not in self.node_positions or next_node_id not in self.node_positions:
                print(f"Error: Path node IDs {current_node_id} or {next_node_id} not in node_positions.")
                continue 

            current_node_pos = self.node_positions[current_node_id] 
            
            turn_path = self._generate_turn_path(current_node_id, next_node_id)

            if turn_path:
                self._append_point_safely(final_path_points, current_node_pos, TOLERANCE, SPIKE_TOLERANCE)
                
                while turn_path and math.hypot(turn_path[0].x() - current_node_pos.x(), turn_path[0].y() - current_node_pos.y()) < 2.0 * TOLERANCE:
                    turn_path.pop(0)

                self._extend_path_safely(final_path_points, turn_path, TOLERANCE, SPIKE_TOLERANCE)

            else:
                next_pos = self.node_positions[next_node_id]
                self._append_point_safely(final_path_points, next_pos, TOLERANCE, SPIKE_TOLERANCE)
        
        self._append_point_safely(final_path_points, end_pos, TOLERANCE, SPIKE_TOLERANCE)

        return final_path_points
    
    def _append_point_safely(self, path_list: list, new_point: QPointF, tol: float, spike_tol: float):
        if not path_list:
            path_list.append(new_point)
            return

        last_point = path_list[-1]
        dist_to_last = math.hypot(new_point.x() - last_point.x(), new_point.y() - last_point.y())
        
        if dist_to_last < tol:
            return 

        if len(path_list) < 2:
            path_list.append(new_point) 
            return
            
        prev_point = path_list[-2]

        dx1 = last_point.x() - prev_point.x()
        dy1 = last_point.y() - prev_point.y()
        dx2 = new_point.x() - last_point.x()
        dy2 = new_point.y() - last_point.y()

        is_vertical_spike = (abs(dx1) < spike_tol and 
                             abs(dx2) < spike_tol and 
                             (dy1 * dy2 < 0)) 

        is_horizontal_spike = (abs(dy1) < spike_tol and 
                               abs(dy2) < spike_tol and 
                               (dx1 * dx2 < 0)) 

        if is_vertical_spike or is_horizontal_spike:
            path_list[-1] = new_point
        else:
            path_list.append(new_point)

    def _extend_path_safely(self, path_list: list, new_points: list, tol: float, spike_tol: float):
        for point in new_points:
            self._append_point_safely(path_list, point, tol, spike_tol)

    def _find_nearest_node_id(self, pos: QPointF, find_source_node: bool = False):
        if not self.node_positions: return None
        min_dist_sq = float('inf')
        nearest_id = None
        for node_id, node_pos in self.node_positions.items():
            
            if find_source_node:
                if not self.road_network_graph.get(node_id): 
                    continue 
                    
            dist_sq = (node_pos.x() - pos.x())**2 + (node_pos.y() - pos.y())**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_id = node_id
        return nearest_id

    def _build_lane_based_road_network(self):
        if not self.intersections:
            return {}

        graph = {}
        
        for i, inter in enumerate(self.intersections):
            is_wide_h = inter.get('is_wide_h', False)
            is_wide_v = inter.get('is_wide_v', False)
            
            if is_wide_h and is_wide_v:
                for p_key, p_val in inter['points'].items():
                    if p_key != 'center' and 'adj' not in p_key:
                        node_id = f"i{i}_{p_key}"
                        self.node_positions[node_id] = QPointF(p_val['x'], p_val['y'])
                        graph[node_id] = []

        for i, inter in enumerate(self.intersections):
            is_wide_h = inter.get('is_wide_h', False)
            is_wide_v = inter.get('is_wide_v', False)
            
            connections = []
            
            if is_wide_h and is_wide_v:
                connections = [
                    ('RL', 'RD'), ('GD', 'RR'), ('GR', 'GU'), ('RU', 'GL'), 
                    ('RL', 'GU'), ('GD', 'GL'), ('GR', 'RD'), ('RU', 'RR') 
                ]
            
            for start_key, end_key in connections:
                id1 = f"i{i}_{start_key}"
                id2 = f"i{i}_{end_key}"
                if id1 in graph and id2 in graph:
                    pos1 = self.node_positions[id1]
                    pos2 = self.node_positions[id2]
                    dist = math.hypot(pos1.x() - pos2.x(), pos1.y() - pos2.y())
                    graph[id1].append((id2, dist))

        for i in range(len(self.intersections)):
            for j in range(i + 1, len(self.intersections)):
                inter1, inter2 = self.intersections[i], self.intersections[j]
                pos1, pos2 = QPointF(inter1['x'], inter1['y']), QPointF(inter2['x'], inter2['y']) 

                if abs(pos1.y() - pos2.y()) < 1:
                    left_inter_idx, right_inter_idx = (i, j) if pos1.x() < pos2.x() else (j, i)
                    
                    id_out = f"i{right_inter_idx}_GL" 
                    id_in  = f"i{left_inter_idx}_GR" 
                    if id_out in graph and id_in in graph:
                        dist = abs(self.node_positions[id_out].x() - self.node_positions[id_in].x())
                        graph[id_out].append((id_in, dist))

                    id_out = f"i{left_inter_idx}_RR" 
                    id_in  = f"i{right_inter_idx}_RL" 
                    if id_out in graph and id_in in graph:
                        dist = abs(self.node_positions[id_out].x() - self.node_positions[id_in].x())
                        graph[id_out].append((id_in, dist))
                
                if abs(pos1.x() - pos2.x()) < 1:
                    top_inter_idx, bottom_inter_idx = (i, j) if pos1.y() < pos2.y() else (j, i)

                    id_out = f"i{bottom_inter_idx}_GU" 
                    id_in  = f"i{top_inter_idx}_GD"   
                    if id_out in graph and id_in in graph:
                        dist = abs(self.node_positions[id_out].y() - self.node_positions[id_in].y())
                        graph[id_out].append((id_in, dist))

                    id_out = f"i{top_inter_idx}_RD"    
                    id_in  = f"i{bottom_inter_idx}_RU" 
                    if id_out in graph and id_in in graph:
                        dist = abs(self.node_positions[id_out].y() - self.node_positions[id_in].y())
                        graph[id_out].append((id_in, dist))

        return graph

    def _ensure_node_exists(self, graph, inter_index, point_key):
        node_id = f"i{inter_index}_{point_key}"
        if node_id not in graph:
            try:
                if inter_index < len(self.intersections) and point_key in self.intersections[inter_index]['points']:
                    p_val = self.intersections[inter_index]['points'][point_key]
                    self.node_positions[node_id] = QPointF(p_val['x'], p_val['y'])
                    graph[node_id] = []
                else:
                    print(f"Warning: Could not find point data for node ID: {node_id}")
                    return None
                    
            except (KeyError, IndexError, ValueError, TypeError) as e:
                print(f"Error processing node ID: {node_id}. Error: {e}")
                return None 
                
        return node_id
    
    def _build_lane_based_road_network1(self):
        if not self.intersections:
            return {}

        graph = {}

        for i, inter in enumerate(self.intersections):
            is_wide_h = inter.get('is_wide_h', False)
            is_wide_v = inter.get('is_wide_v', False)
            is_real = inter.get('type') == 'real'
            
            turn_keys = [] 
            
            if is_wide_h and is_wide_v:
                if is_real:
                    turn_keys = [('RL', 'RD'), ('GD', 'RR'), ('GR', 'GU'), ('RU', 'GL'), ('RL', 'GU'), ('GD', 'GL'), ('GR', 'RD'), ('RU', 'RR')]
                else:
                    virtual_side = inter.get('virtual_side')
                    if virtual_side == 'left':
                        turn_keys = [('RU', 'GL'), ('RL', 'RD'), ('RL', 'GU'), ('GD', 'GL')]
                    elif virtual_side == 'right':
                        turn_keys = [('GR', 'GU'), ('GD', 'RR'), ('RU', 'RR'), ('GR', 'RD')]
            elif not is_wide_h and is_wide_v: 
                if is_real:
                    turn_keys = [('RL', 'RD'), ('RU', 'RL'), ('RR', 'GU'), ('GD', 'RR'), ('RL', 'GU'), ('RL', 'GD'), ('RR', 'RD'), ('RU', 'RR')]
                else:
                    virtual_side = inter.get('virtual_side')
                    if virtual_side == 'left':
                        turn_keys = [('RU', 'RL'), ('RL', 'RD'), ('RL', 'GU'), ('RL', 'GD')]
                    elif virtual_side == 'right':
                        turn_keys = [('RR', 'GU'), ('GD', 'RR'), ('RR', 'RD'), ('RU', 'RR')]
            elif is_wide_h and not is_wide_v: 
                if is_real:
                    turn_keys = [('GR', 'GU'), ('GD', 'RR'), ('GU', 'GL'), ('RL', 'GD'), ('GD', 'GL'), ('GU', 'RR'), ('GD', 'GR'), ('GU', 'RL')]
                else:
                    virtual_side = inter.get('virtual_side')
                    if virtual_side == 'left':
                         turn_keys = [('GU', 'GL'), ('RL', 'GD'), ('GD', 'GL'), ('GU', 'RL')]
                    elif virtual_side == 'right':
                        turn_keys = [('GR', 'GU'), ('GD', 'RR'), ('GU', 'RR'), ('GD', 'GR')]
            elif not is_wide_h and not is_wide_v: 
                if is_real:
                    turn_keys = [
                        ('GU', 'RL'), ('GD', 'RL'), ('GD', 'RR'), ('RR', 'GU'), 
                        ('RL', 'GU'), ('RL', 'GD'), ('RR', 'GD'), ('GU', 'RR') 
                    ]
                else:
                    virtual_side = inter.get('virtual_side')
                    if virtual_side == 'left':
                        turn_keys = [('GU', 'RL'), ('GD', 'RL')]
                    elif virtual_side == 'right':
                         turn_keys = [('GD', 'RR'), ('RR', 'GU')]

            for start_key, end_key in turn_keys:
                id1 = self._ensure_node_exists(graph, i, start_key)
                id2 = self._ensure_node_exists(graph, i, end_key)
                
                if id1 and id2:
                    pos1 = self.node_positions[id1]
                    pos2 = self.node_positions[id2]
                    dist = math.hypot(pos1.x() - pos2.x(), pos1.y() - pos2.y())
                    graph[id1].append((id2, dist))

        for path_y, path_data in self.horizontal_paths.items():
            is_wide_h = path_data.get('is_wide', False)
            inter_indices = sorted(path_data.get('intersections', []), 
                                   key=lambda idx: self.intersections[idx]['x'])
            
            for k in range(len(inter_indices) - 1):
                left_inter_idx = inter_indices[k]
                right_inter_idx = inter_indices[k+1]

                if is_wide_h:
                    id_out_g = self._ensure_node_exists(graph, right_inter_idx, 'GL')
                    id_in_g  = self._ensure_node_exists(graph, left_inter_idx, 'GR')
                    if id_out_g and id_in_g:
                        dist = abs(self.node_positions[id_out_g].x() - self.node_positions[id_in_g].x())
                        graph[id_out_g].append((id_in_g, dist))
                    
                    id_out_r = self._ensure_node_exists(graph, left_inter_idx, 'RR')
                    id_in_r  = self._ensure_node_exists(graph, right_inter_idx, 'RL')
                    if id_out_r and id_in_r:
                        dist = abs(self.node_positions[id_out_r].x() - self.node_positions[id_in_r].x())
                        graph[id_out_r].append((id_in_r, dist))
                else:
                    id_l = self._ensure_node_exists(graph, left_inter_idx, 'RR') 
                    id_r = self._ensure_node_exists(graph, right_inter_idx, 'RL') 
                    if id_l and id_r:
                        dist = abs
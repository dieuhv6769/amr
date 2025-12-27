import heapq
import math

class AStarGraphSearch:
    def __init__(self, graph: dict, node_positions: dict):
        self.graph = graph
        self.node_positions = node_positions

    def find_path(self, start_id, end_id):
        if start_id not in self.graph or end_id not in self.graph:
            return None

        open_set = [(0.0, start_id)]
        open_set_hash = {start_id}
        
        came_from = {}
        
        g_score = {node_id: float('inf') for node_id in self.graph}
        g_score[start_id] = 0
        
        f_score = {node_id: float('inf') for node_id in self.graph}
        f_score[start_id] = self._heuristic(start_id, end_id)

        while open_set:
            _, current_id = heapq.heappop(open_set)
            
            if current_id in open_set_hash:
                open_set_hash.remove(current_id)
            else:
                continue

            if current_id == end_id:
                return self._reconstruct_path(came_from, current_id)

            for neighbor_id, cost in self.graph.get(current_id, []):
                tentative_g_score = g_score[current_id] + cost
                if tentative_g_score < g_score[neighbor_id]:
                    came_from[neighbor_id] = current_id
                    g_score[neighbor_id] = tentative_g_score
                    f_score[neighbor_id] = tentative_g_score + self._heuristic(neighbor_id, end_id)
                    
                    if neighbor_id not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
                        open_set_hash.add(neighbor_id)
        
        return None 

    def _heuristic(self, node_id1, node_id2):
        pos1 = self.node_positions[node_id1]
        pos2 = self.node_positions[node_id2]
        return math.hypot(pos1.x() - pos2.x(), pos1.y() - pos2.y())

    def _reconstruct_path(self, came_from, current_id):
        total_path = [current_id]
        while current_id in came_from:
            current_id = came_from[current_id]
            total_path.insert(0, current_id)
        return total_path

import math
import random
from PyQt5.QtCore import QPointF, QLineF
from PyQt5.QtGui import QPainterPath

class RRTNode:
    def __init__(self, pos: QPointF):
        self.pos = pos
        self.parent = None

class RRT:
    def __init__(self, start_pos: QPointF, end_pos: QPointF,
                 obstacles: list, map_rect, step_size=20.0, max_iterations=2000):
        self.start_node = RRTNode(start_pos)
        self.end_node = RRTNode(end_pos)
        self.obstacles = obstacles
        self.map_rect = map_rect
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.node_list = [self.start_node]

    def find_path(self):
        final_goal_pos = self.end_node.pos
        approachable_goal_node = self._get_approachable_goal()
        
        original_end_node = self.end_node
        self.end_node = approachable_goal_node
        
        path_to_approach = None
        for i in range(self.max_iterations):
            rnd_node = self._get_random_node()
            nearest_node = self._get_nearest_node(self.node_list, rnd_node)
            new_node = self._steer(nearest_node, rnd_node, self.step_size)

            if self._is_collision_free(from_node=nearest_node, to_node=new_node):
                new_node.parent = nearest_node
                self.node_list.append(new_node)

                if self._is_goal_reached(new_node, self.end_node):
                    if self._is_collision_free(from_node=new_node, to_node=self.end_node):
                        self.end_node.parent = new_node
                        path_to_approach = self._reconstruct_path(self.end_node)
                        break 

        self.end_node = original_end_node

        if path_to_approach:
            smoothed = self.smooth_path(path_to_approach)
            smoothed.append(final_goal_pos)
            return smoothed
            
        return None 

    def _get_approachable_goal(self):
        goal_pos = self.end_node.pos
        for obs in self.obstacles:
            if obs.full_rect.contains(goal_pos):
                parent_obs_rect = obs.full_rect
                approach_dist = 5.0 

                dist_top = abs(goal_pos.y() - parent_obs_rect.top())
                dist_bottom = abs(goal_pos.y() - parent_obs_rect.bottom())
                dist_left = abs(goal_pos.x() - parent_obs_rect.left())
                dist_right = abs(goal_pos.x() - parent_obs_rect.right())
                
                min_dist = min(dist_top, dist_bottom, dist_left, dist_right)

                if min_dist == dist_top:
                    return RRTNode(QPointF(goal_pos.x(), parent_obs_rect.top() - approach_dist))
                elif min_dist == dist_bottom:
                    return RRTNode(QPointF(goal_pos.x(), parent_obs_rect.bottom() + approach_dist))
                elif min_dist == dist_left:
                    return RRTNode(QPointF(parent_obs_rect.left() - approach_dist, goal_pos.y()))
                else: 
                    return RRTNode(QPointF(parent_obs_rect.right() + approach_dist, goal_pos.y()))
        
        return self.end_node

    def smooth_path(self, path, iterations=50):
        if not path or len(path) < 3:
            return path
        
        smoothed_path = list(path)
        for _ in range(iterations):
            if len(smoothed_path) < 3: break
            
            i = random.randint(0, len(smoothed_path) - 1)
            j = random.randint(0, len(smoothed_path) - 1)
            if abs(i - j) <= 1: continue
            if i > j: i, j = j, i
            
            node1 = RRTNode(smoothed_path[i])
            node2 = RRTNode(smoothed_path[j])
            
            if self._is_collision_free(from_node=node1, to_node=node2):
                del smoothed_path[i+1:j]
        return smoothed_path

    def _is_collision_free(self, from_node, to_node=None):
        if to_node is None:
            return not any(obs.full_rect.contains(from_node.pos) for obs in self.obstacles)

        line_path = QPainterPath()
        line_path.moveTo(from_node.pos)
        line_path.lineTo(to_node.pos)

        for obs in self.obstacles:
            if line_path.intersects(obs.full_rect):
                return False
            
        return True

    def _get_random_node(self):
        if random.randint(0, 100) > 90:
            return RRTNode(self.end_node.pos)
        rnd_x = random.uniform(self.map_rect.left(), self.map_rect.right())
        rnd_y = random.uniform(self.map_rect.top(), self.map_rect.bottom())
        return RRTNode(QPointF(rnd_x, rnd_y))
        
    def _get_nearest_node(self, node_list, rnd_node):
        return min(node_list, key=lambda node: math.hypot(node.pos.x() - rnd_node.pos.x(), node.pos.y() - rnd_node.pos.y()))
        
    def _steer(self, from_node, to_node, extend_length=float("inf")):
        d, theta = self._calculate_distance_and_angle(from_node, to_node)
        new_pos = QPointF(from_node.pos.x() + min(extend_length, d) * math.cos(theta),
                          from_node.pos.y() + min(extend_length, d) * math.sin(theta))
        return RRTNode(new_pos)
        
    def _is_goal_reached(self, from_node, to_node, tolerance=None):
        if tolerance is None: tolerance = self.step_size
        return math.hypot(from_node.pos.x() - to_node.pos.x(), from_node.pos.y() - to_node.pos.y()) <= tolerance
        
    def _reconstruct_path(self, final_node):
        path = [final_node.pos]
        node = final_node
        while node.parent:
            path.append(node.parent.pos)
            node = node.parent
        return path[::-1]
        
    def _calculate_distance_and_angle(self, from_node, to_node):
        dx = to_node.pos.x() - from_node.pos.x()
        dy = to_node.pos.y() - from_node.pos.y()
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
# c_amr/views/simulation_scene.py
import math
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsPathItem, QGraphicsRectItem
from PyQt5.QtGui import QBrush, QPen, QPainter, QPainterPath, QColor
from PyQt5.QtCore import pyqtSignal, QPointF, Qt, QRectF

import c_amr.constants as const
from c_amr.core.simulation_model import SimulationModel
from c_amr.views.item_views import (ObstacleItemView, RobotItemView, CentralAMRItemView, LaneNodeItem, CentralAMRAgentModel, CentralAMRAgentItemView,
                               NodeItemView, StationItemView, TrafficNetworkItem, DebugIntersectionItem)

class SimulationScene(QGraphicsScene):
    map_clicked_signal = pyqtSignal(QPointF, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundBrush(QBrush(const.SCENE_BG_COLOR))
        self.path_item = None
        self.robot_views = {}
        self.model = None
        self.traffic_network_item = None

        self.camr_mission_path_item = None 
        self.camr_destination_highlight = None
        self.robot_path_items = {} 

    def update_static_layout(self, model: SimulationModel):
        self.model = model
        
        if not self.model:
            return

        if self.sceneRect() != self.model.map_rect:
            self.setSceneRect(self.model.map_rect)
        
        self.clear_static_items()
        
        for obs_model in model.obstacles:
            self.addItem(ObstacleItemView(obs_model))
        for station_model in model.stations:
            self.addItem(StationItemView(station_model))

        if model.horizontal_paths or model.vertical_corridors:
            self.traffic_network_item = TrafficNetworkItem(model)
            self.addItem(self.traffic_network_item)

    def update_dynamic_items(self, model: SimulationModel):
        self.model = model
        if not self.model: return
            
        current_robot_ids = set(model.robots.keys())
        
        for robot_id, robot_model in model.robots.items():
            if robot_id in self.robot_views:
                view = self.robot_views[robot_id]
                view.setPos(robot_model.pos)
                view.update()
            else:
                robot_size_px = robot_model.robot_size
                if robot_model.is_c_amr:
                    view = CentralAMRAgentItemView(robot_model, robot_size_px)
                else:
                    view = RobotItemView(robot_model, robot_size_px)
                self.addItem(view)
                self.robot_views[robot_id] = view
        
        ids_to_remove = set(self.robot_views.keys()) - current_robot_ids
        for robot_id in ids_to_remove:
            if robot_id in self.robot_views:
                self.removeItem(self.robot_views[robot_id])
                del self.robot_views[robot_id]
        
        self.update_all_robot_paths(model)

    def update_all_robot_paths(self, model: SimulationModel):
        drawn_robot_ids = set()

        for robot_id, robot_model in model.robots.items():
            drawn_robot_ids.add(robot_id)
            path = robot_model.path

            if path and len(path) > 1:
                if robot_id not in self.robot_path_items:
                    path_item = QGraphicsPathItem()
                    if robot_model.is_c_amr:
                        path_item.setPen(QPen(const.C_AMR_PATH_COLOR, 2, Qt.PenStyle.SolidLine))
                    else:
                        path_item.setPen(QPen(const.RRT_PATH_COLOR, 1, Qt.PenStyle.DashLine))
                    path_item.setZValue(0.6)
                    self.addItem(path_item)
                    self.robot_path_items[robot_id] = path_item
                
                painter_path = QPainterPath()
                painter_path.moveTo(robot_model.pos)
                for point in path[robot_model.path_index:]:
                    painter_path.lineTo(point)
                self.robot_path_items[robot_id].setPath(painter_path)

            elif robot_id in self.robot_path_items:
                self.removeItem(self.robot_path_items[robot_id])
                del self.robot_path_items[robot_id]
        
        ids_to_remove = set(self.robot_path_items.keys()) - drawn_robot_ids
        for robot_id in ids_to_remove:
            self.removeItem(self.robot_path_items[robot_id])
            del self.robot_path_items[robot_id]
    
    def draw_camr_mission_path_destination_point(self, path: list):
        if self.camr_mission_path_item:
            self.removeItem(self.camr_mission_path_item)
            self.camr_mission_path_item = None
        if self.camr_destination_highlight:
            self.removeItem(self.camr_destination_highlight)
            self.camr_destination_highlight = None
        if path and len(path) > 1:
            destination_pos = path[-1]
            highlight_size = 10
            self.camr_destination_highlight = QGraphicsEllipseItem(
                destination_pos.x() - highlight_size / 2,
                destination_pos.y() - highlight_size / 2,
                highlight_size,
                highlight_size
            )
            highlight_pen = QPen(const.C_AMR_PATH_COLOR, 2)
            highlight_brush = QBrush(const.C_AMR_PATH_COLOR.lighter(150))
            self.camr_destination_highlight.setPen(highlight_pen)
            self.camr_destination_highlight.setBrush(highlight_brush)
            self.camr_destination_highlight.setZValue(0.8)
            self.addItem(self.camr_destination_highlight)

    def drawForeground(self, painter, rect):
        super().drawForeground(painter, rect)
        
        if not self.model or not self.model.c_amr or self.model.c_amr.markov_radius <= 0:
            return
            
        c_amr = self.model.c_amr
        camr_center = c_amr.pos

        pen = QPen(const.MARKOV_LINE_COLOR, 1, Qt.PenStyle.DashLine)
        painter.setPen(pen)
        
        for robot in self.model.robots.values():
            if robot is c_amr: 
                continue
            
            other_center = robot.pos
            distance = math.hypot(camr_center.x() - other_center.x(), camr_center.y() - other_center.y())
            
            if distance < c_amr.markov_radius:
                painter.drawLine(camr_center, other_center)

    def clear_all_items(self):
        self.clear()
        self.robot_views.clear()
        self.robot_path_items.clear()
        self.camr_mission_path_item = None
        self.camr_destination_highlight = None
        self.traffic_network_item = None

    def clear_static_items(self):
        items_to_remove = [item for item in self.items() if not isinstance(item, (RobotItemView, CentralAMRAgentItemView))]
        for item in items_to_remove:
            self.removeItem(item)
        
        for item in self.robot_path_items.values():
            self.removeItem(item)
        self.robot_path_items.clear()
        if self.camr_mission_path_item:
            self.removeItem(self.camr_mission_path_item)
            self.camr_mission_path_item = None

        if self.camr_destination_highlight:
            self.removeItem(self.camr_destination_highlight)
            self.camr_destination_highlight = None

        self.traffic_network_item = None

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        grid_pen = QPen(const.GRID_PEN_COLOR, const.GRID_PEN_WIDTH, Qt.PenStyle.SolidLine)
        painter.setPen(grid_pen)
        left, right = int(rect.left()), int(rect.right())
        top, bottom = int(rect.top()), int(rect.bottom())
        first_left = left - (left % const.GRID_CELL_SIZE)
        for x in range(first_left, right, const.GRID_CELL_SIZE):
            painter.drawLine(x, top, x, bottom)
        first_top = top - (top % const.GRID_CELL_SIZE)
        for y in range(first_top, bottom, const.GRID_CELL_SIZE):
            painter.drawLine(left, y, right, y)

        border_pen = QPen(const.BORDER_PEN_COLOR, 3, Qt.PenStyle.SolidLine)
        painter.setPen(border_pen)
        painter.drawRect(self.sceneRect())

    def mousePressEvent(self, event):
        scene_pos = event.scenePos()
        item_at_click = self.itemAt(scene_pos, self.views()[0].transform())
        self.map_clicked_signal.emit(scene_pos, item_at_click)
        super().mousePressEvent(event)

class SimulationView(QGraphicsView):
    path_request_signal = pyqtSignal(QPointF)
    mouse_moved_signal = pyqtSignal(str)

    def __init__(self, scene: SimulationScene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setMouseTracking(True)
        self.current_edit_mode = "none"
        self.drawing_item = None
        self.start_pos = None
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

    def wheelEvent(self, event):
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        zoom_factor = 1.25 if event.angleDelta().y() > 0 else 1 / 1.25
        self.scale(zoom_factor, zoom_factor)
        self.setTransformationAnchor(QGraphicsView.AnchorViewCenter)

    def zoom_to_fit(self):
        if self.scene():
            self.fitInView(self.scene().sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def set_edit_mode(self, mode: str):
        self.current_edit_mode = mode
        if mode != "none":
            self.setDragMode(QGraphicsView.NoDrag)
            if mode == "obstacle":
                self.setCursor(Qt.CursorShape.CrossCursor)
            else:
                self.setCursor(Qt.CursorShape.PointingHandCursor)
        else:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.setCursor(Qt.CursorShape.ArrowCursor)

    def mousePressEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        if event.button() == Qt.MouseButton.RightButton and self.current_edit_mode == "none":
            self.path_request_signal.emit(scene_pos)
            return

        if event.button() == Qt.MouseButton.LeftButton and self.current_edit_mode == "obstacle":
            self.start_pos = scene_pos
            self.drawing_item = QGraphicsRectItem(QRectF(self.start_pos, self.start_pos))
            self.drawing_item.setPen(QPen(const.DRAWING_RECT_COLOR, 2, Qt.PenStyle.DashLine))
            self.scene().addItem(self.drawing_item)
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        coords_text = f"Coordinates: ({scene_pos.x():.1f}, {scene_pos.y():.1f})"
        self.mouse_moved_signal.emit(coords_text)

        if self.current_edit_mode == "obstacle" and self.drawing_item:
            rect = QRectF(self.start_pos, scene_pos).normalized()
            self.drawing_item.setRect(rect)

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.current_edit_mode == "obstacle" and self.drawing_item:
            final_rect = self.drawing_item.rect()
            self.scene().removeItem(self.drawing_item)
            self.drawing_item = None

        super().mouseReleaseEvent(event)

    def leaveEvent(self, event):
        self.mouse_moved_signal.emit("Coordinates: N/A")
        super().leaveEvent(event)
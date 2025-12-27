# c_amr/views/layer_dialogs.py
import math
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QGraphicsView, QGraphicsScene, 
                             QGraphicsItem, QStyle, QScrollArea, QWidget, QLabel,
                             QGroupBox, QFormLayout)
from PyQt5.QtGui import QBrush, QPen, QFont, QPainter # THÊM: Import QPainter
from PyQt5.QtCore import Qt, QTimer

import c_amr.constants as const
from c_amr.core.simulation_model import SimulationModel
from c_amr.views.item_views import ObstacleItemView, RobotItemView, CentralAMRItemView, NodeItemView, StationItemView
class LayerScene(QGraphicsScene):
    def drawBackground(self, painter: QPainter, rect):
        super().drawBackground(painter, rect)
        painter.fillRect(rect, QBrush(const.SCENE_BG_COLOR))
        
        grid_pen = QPen(const.GRID_PEN_COLOR, 1, Qt.PenStyle.SolidLine)
        painter.setPen(grid_pen)
        
        left, right = int(rect.left()), int(rect.right())
        top, bottom = int(rect.top()), int(rect.bottom())
        
        first_left = left - (left % const.GRID_CELL_SIZE)
        for x in range(first_left, right, const.GRID_CELL_SIZE):
            painter.drawLine(x, top, x, bottom)
            
        first_top = top - (top % const.GRID_CELL_SIZE)
        for y in range(first_top, bottom, const.GRID_CELL_SIZE):
            painter.drawLine(left, y, right, y)

class LiveLayerViewDialog(QDialog):
    def __init__(self, model: SimulationModel, parent=None):
        super().__init__(parent)
        self.model = model
        
        self.setWindowTitle("Layer 4 View (Real-time Layer)")
        self.setGeometry(200, 200, 800, 700)
        self.setMinimumSize(600, 500)

        layout = QVBoxLayout(self)
        
        self.scene = LayerScene()
        self.view = QGraphicsView(self.scene)
        self.view.setDragMode(QGraphicsView.RubberBandDrag)
        
        map_view_fixed_height = 450
        self.view.setFixedHeight(map_view_fixed_height)
        
        layout.addWidget(self.view)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.info_container = QWidget()
        self.info_layout = QVBoxLayout(self.info_container)
        self.info_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll_area.setWidget(self.info_container)
        layout.addWidget(self.scroll_area)

        self.scene.setSceneRect(self.model.map_rect)
        self.robot_view_map = {}
        self._populate_scene()

        self.scene.selectionChanged.connect(self._on_selection_changed)
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_robot_views)
        self.update_timer.start(const.UPDATE_INTERVAL_MS * 2)

        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        self._on_selection_changed()

    def _populate_scene(self):
        robot_size_px = const.DEFAULT_ROBOT_SIZE_M * const.PIXELS_PER_METER
        for robot_model in self.model.robots.values():
            if robot_model.is_c_amr:
                view = CentralAMRItemView(robot_model, robot_size_px)
            else:
                view = RobotItemView(robot_model, robot_size_px)

            view.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
            self.scene.addItem(view)
            self.robot_view_map[robot_model.robot_id] = view

    def _update_robot_views(self):
        for robot_id, view_item in self.robot_view_map.items():
            if robot_id in self.model.robots:
                model = self.model.robots[robot_id]
                view_item.setPos(model.pos)
                view_item.update()
        
        if self.scene.selectedItems():
            self._on_selection_changed()

    def _on_selection_changed(self):
        while self.info_layout.count():
            child = self.info_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        selected_items = self.scene.selectedItems()
        if not selected_items:
            label = QLabel("Select one or more robots on the map to view details.")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.info_layout.addWidget(label)
        else:
            sorted_items = sorted(selected_items, key=lambda item : item.model.robot_id) # type: ignore
            for item in sorted_items:
                if isinstance(item, RobotItemView):
                    model = item.model
                    robot_group_box = QGroupBox(f"Robot ID: {model.robot_id}")
                    robot_layout = QFormLayout(robot_group_box)
                    robot_layout.setContentsMargins(5, 10, 5, 5)
                    robot_layout.setSpacing(4)

                    pos_label = QLabel(f"({model.pos.x():.1f}, {model.pos.y():.1f})")
                    heading_label = QLabel(f"{model.heading:.2f}°")
                    velo_label = QLabel(f"({model.tv:.2f} m/s, {model.rv:.2f} rad/s)")
                    soc_label = QLabel(f"{model.soc:.1f}%")
                    np_label = QLabel(f"{model.np:.1f}%")
                    cp_label = QLabel(f"{model.cp:.1f}%")
                    fd_label = QLabel(f"{model.fd:.1f}")

                    robot_layout.addRow("Position:", pos_label)
                    robot_layout.addRow("Heading:", heading_label)
                    robot_layout.addRow("Velocity:", velo_label)
                    robot_layout.addRow("SoC:", soc_label)
                    robot_layout.addRow("Nav. Precision:", np_label)
                    robot_layout.addRow("Coll. Prob.:", cp_label)
                    robot_layout.addRow("Fault Diagnosis:", fd_label)
                    
                    self.info_layout.addWidget(robot_group_box)
        
        self.info_layout.addStretch()

    def closeEvent(self, event):
        self.update_timer.stop()
        super().closeEvent(event)

class LayerViewDialog(QDialog):
    def __init__(self, model: SimulationModel, layer_number: int, parent=None):
        super().__init__(parent)
        self.model = model
        self.layer_number = layer_number
        
        self.setWindowTitle(f"Layer {self.layer_number} View")
        self.setGeometry(200, 200, 800, 600)

        layout = QVBoxLayout(self)
        self.scene = LayerScene()
        self.view = QGraphicsView(self.scene)
        layout.addWidget(self.view)

        self.scene.setSceneRect(self.model.map_rect)
        self._populate_scene()
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _populate_scene(self):
        if self.layer_number == 1:
            for obs_model in self.model.obstacles:
                self.scene.addItem(ObstacleItemView(obs_model))
        
        elif self.layer_number == 2:
            for node_model in self.model.nodes:
                self.scene.addItem(NodeItemView(node_model))
            for station_model in self.model.stations:
                self.scene.addItem(StationItemView(station_model))
                
            if self.model.horizontal_paths and self.model.vertical_corridors and self.model.intersections:
                traffic_item = TrafficNetworkItem(self.model)
                self.scene.addItem(traffic_item)


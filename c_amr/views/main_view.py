# c_amr/views/main_view.py
from PyQt5.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                             QFormLayout, QGroupBox, QLabel, QSpinBox, 
                             QDoubleSpinBox, QPushButton, QCheckBox, QComboBox)
from PyQt5.QtCore import pyqtSignal, QPointF, QTimer

import c_amr.constants as const
from datetime import datetime
from c_amr.views.simulation_scene import SimulationScene, SimulationView


class MainView(QMainWindow):
    restart_button_clicked = pyqtSignal(dict)
    path_requested = pyqtSignal(QPointF)
    markov_radius_changed = pyqtSignal(float)
    show_layer_requested = pyqtSignal(int)
    edit_mode_toggled = pyqtSignal(str, bool)
    
    import_obstacles_requested = pyqtSignal()
    import_network_requested = pyqtSignal()
    export_obstacles_requested = pyqtSignal()
    export_network_requested = pyqtSignal()
    
    build_map_requested = pyqtSignal()
    export_svg_requested = pyqtSignal()
    
    go_to_station_requested = pyqtSignal(int)
    zoom_to_fit_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(const.WINDOW_TITLE)
        self.setGeometry(100, 100, 1280, 800)

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self.update_time)
        self.clock_timer.start(1000)

        self.displayed_station_ids = []
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        self.config_panel = self._create_config_panel()
        self.info_panel = self._create_info_panel()
        self.scene = SimulationScene()
        self.view = SimulationView(self.scene)
        
        main_layout.addWidget(self.config_panel)
        main_layout.addWidget(self.view, 1)
        main_layout.addWidget(self.info_panel)

        self.status_bar = self.statusBar()
        self.mouse_coords_label = QLabel("Coordinates: N/A")
        self.status_bar.addPermanentWidget(self.mouse_coords_label)

        self.view.path_request_signal.connect(self.path_requested)
        self.markov_radius_spinbox.valueChanged.connect(self.markov_radius_changed)
        self.view.mouse_moved_signal.connect(self.update_mouse_coords)
        self.go_to_station_button.clicked.connect(self._on_go_to_station_clicked)

    def _create_config_panel(self) -> QWidget:
        panel = QWidget()
        layout = QFormLayout(panel)
        panel.setFixedWidth(const.CONFIG_PANEL_WIDTH)
        
        map_info_group = QGroupBox("Map Info")
        map_info_layout = QFormLayout(map_info_group)
        self.map_width_spinbox = QDoubleSpinBox()
        self.map_width_spinbox.setRange(25, 250.0)
        self.map_width_spinbox.setSingleStep(5)
        self.map_width_spinbox.setValue(const.DEFAULT_MAP_WIDTH_M)
        self.map_height_spinbox = QDoubleSpinBox()
        self.map_height_spinbox.setRange(25, 250)
        self.map_height_spinbox.setSingleStep(5)
        self.map_height_spinbox.setValue(const.DEFAULT_MAP_HEIGHT_M)
        map_info_layout.addRow("Map Width (m):", self.map_width_spinbox)
        map_info_layout.addRow("Map Height (m):", self.map_height_spinbox)
        layout.addRow(map_info_group)

        generation_group = QGroupBox("Map Generation")
        generation_layout = QVBoxLayout(generation_group)
        self.build_map_button = QPushButton("Build New Map")
        self.export_svg_button = QPushButton("Export to SVG")
        generation_layout.addWidget(self.build_map_button)
        generation_layout.addWidget(self.export_svg_button)
        layout.addRow(generation_group)

        editing_group = QGroupBox("Map Editing")
        editing_layout = QVBoxLayout(editing_group)
        self.add_obstacle_button = QPushButton("Add Obstacle")
        self.add_node_button = QPushButton("Add Node")
        self.add_station_button = QPushButton("Add Station")
        self.add_obstacle_button.setCheckable(True)
        self.add_node_button.setCheckable(True)
        self.add_station_button.setCheckable(True)
        editing_layout.addWidget(self.add_obstacle_button)
        editing_layout.addWidget(self.add_node_button)
        editing_layout.addWidget(self.add_station_button)
        layout.addRow(editing_group)
        
        data_group = QGroupBox("Data Management")
        data_layout = QVBoxLayout(data_group)
        self.import_obstacles_button = QPushButton("Import Obstacles")
        self.export_obstacles_button = QPushButton("Export Obstacles")
        self.import_network_button = QPushButton("Import Nodes & Stations")
        self.export_network_button = QPushButton("Export Nodes & Stations")
        data_layout.addWidget(self.import_obstacles_button)
        data_layout.addWidget(self.export_obstacles_button)
        data_layout.addWidget(self.import_network_button)
        data_layout.addWidget(self.export_network_button)
        layout.addRow(data_group)
        
        zoom_fit_button = QPushButton("Zoom to Fit")
        layout.addRow(zoom_fit_button)

        restart_button = QPushButton("Apply & Restart")
        layout.addRow(restart_button)
        
        restart_button.clicked.connect(lambda: self.restart_button_clicked.emit(self.get_config()))
        zoom_fit_button.clicked.connect(self.zoom_to_fit_requested)
        self.import_obstacles_button.clicked.connect(self.import_obstacles_requested)
        self.export_obstacles_button.clicked.connect(self.export_obstacles_requested)
        self.import_network_button.clicked.connect(self.import_network_requested)
        self.export_network_button.clicked.connect(self.export_network_requested)
        self.build_map_button.clicked.connect(self.build_map_requested)
        self.export_svg_button.clicked.connect(self.export_svg_requested)
        self.add_obstacle_button.toggled.connect(lambda checked: self._on_edit_mode_toggled("obstacle", checked))
        self.add_node_button.toggled.connect(lambda checked: self._on_edit_mode_toggled("node", checked))
        self.add_station_button.toggled.connect(lambda checked: self._on_edit_mode_toggled("station", checked))

        return panel

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, lambda: self.zoom_to_fit_requested.emit())

    def _create_info_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        panel.setFixedWidth(const.INFO_PANEL_WIDTH)
        
        camr_info_group = QGroupBox(f"AMR Info (ID: {const.C_AMR_ID})")
        camr_info_layout = QFormLayout(camr_info_group)
        self.camr_pos_label = QLabel("N/A")
        self.camr_heading_label = QLabel("N/A")
        self.camr_velo_label = QLabel("N/A")
        self.camr_soc_label = QLabel("N/A")
        self.camr_np_label = QLabel("N/A")
        self.camr_cp_label = QLabel("N/A")
        self.camr_fd_label = QLabel("N/A")
        camr_info_layout.addRow("<b>Position (px):</b>", self.camr_pos_label)
        camr_info_layout.addRow("<b>Heading (°):</b>", self.camr_heading_label)
        camr_info_layout.addRow("<b>Velocity (tv, rv):</b>", self.camr_velo_label)
        camr_info_layout.addRow("<b>SoC (%):</b>", self.camr_soc_label)
        camr_info_layout.addRow("<b>Nav. Precision (%):</b>", self.camr_np_label)
        camr_info_layout.addRow("<b>Coll. Prob. (%):</b>", self.camr_cp_label)
        camr_info_layout.addRow("<b>Fault Diagnosis:</b>", self.camr_fd_label)
        layout.addWidget(camr_info_group)
        
        camr_state_group = QGroupBox("AMR State Control")
        camr_state_layout = QFormLayout(camr_state_group)
        self.current_time = QLabel(f"{datetime.now().strftime('%Y:%m:%d %H:%M:%S')}")
        camr_state_layout.addRow("<b>Who:</b>", QLabel(f"{const.C_AMR_ID}"))
        camr_state_layout.addRow("<b>When:</b>", self.current_time)
        camr_state_layout.addRow("<b>Where:</b>", QLabel("A1"))
        camr_state_layout.addRow("<b>What:</b>", QLabel("None"))
        camr_state_layout.addRow("<b>How:</b>", QLabel("None"))
        camr_state_layout.addRow("<b>Why:</b>", QLabel("None"))
        layout.addWidget(camr_state_group)

        task_group = QGroupBox("Task Control")
        task_layout = QFormLayout(task_group)
        self.station_selector_combo = QComboBox()
        self.station_selector_combo.setMaxVisibleItems(10)
        self.station_selector_combo.setStyleSheet("""
            QComboBox QAbstractItemView {
                border: 1px solid darkgray;
                selection-background-color: lightgray;
            }
        """)
        self.go_to_station_button = QPushButton("Go to Station")
        self.planning_mode_checkbox = QCheckBox("Use Road Network (A*)")
        task_layout.addRow(self.planning_mode_checkbox)
        task_layout.addRow("Target Station:", self.station_selector_combo)
        task_layout.addWidget(self.go_to_station_button)
        layout.addWidget(task_group)
        
        markov_group = QGroupBox("Markov Blanket")
        markov_layout = QFormLayout(markov_group)
        self.markov_radius_spinbox = QDoubleSpinBox()
        self.markov_radius_spinbox.setRange(0.0, 50.0)
        self.markov_radius_spinbox.setSingleStep(0.5)
        self.markov_radius_spinbox.setValue(3.0)
        self.approaching_camr_label = QLabel("None")
        self.camr_approaching_label = QLabel("None")
        self.approaching_camr_label.setWordWrap(True)
        self.camr_approaching_label.setWordWrap(True)
        markov_layout.addRow("Radius (m):", self.markov_radius_spinbox)
        markov_layout.addRow("Approaching C-AMR:", self.approaching_camr_label)
        markov_layout.addRow("C-AMR Approaching:", self.camr_approaching_label)
        layout.addWidget(markov_group)
        
        layer_group = QGroupBox("High Definition - Local Dynamic Map")
        layer_layout = QFormLayout(layer_group)
        layer_names = ["SLAM-mapping Layer", "Configurable Layer", "Transient Layer", "Real-time Layer"]
        for i, name in enumerate(layer_names, 1):
            btn = QPushButton(f"Show Layer")
            btn.clicked.connect(lambda _, num=i: self.show_layer_requested.emit(num))
            layer_layout.addRow(name + ":", btn)
        layout.addWidget(layer_group)
        
        layout.addStretch()
        return panel

    def update_time(self):
        self.current_time.setText(datetime.now().strftime("%Y:%m:%d %H:%M:%S"))
        
    def update_info_panel(self, c_amr_model, interaction_data, stations):
        if not c_amr_model:
            self.camr_pos_label.setText("N/A")
            self.camr_heading_label.setText("N/A")
            self.camr_velo_label.setText("N/A")
            self.camr_soc_label.setText("N/A")
            self.camr_np_label.setText("N/A")
            self.camr_cp_label.setText("N/A")
            self.camr_fd_label.setText("N/A")
        else:
            self.camr_pos_label.setText(f"({c_amr_model.pos.x():.1f}, {c_amr_model.pos.y():.1f})")
            self.camr_heading_label.setText(f"{c_amr_model.heading:.2f}")
            self.camr_velo_label.setText(f"({c_amr_model.tv:.2f} m/s, {c_amr_model.rv:.2f} rad/s)")
            self.camr_soc_label.setText(f"{c_amr_model.soc:.1f}")
            self.camr_np_label.setText(f"{c_amr_model.np:.1f}")
            self.camr_cp_label.setText(f"{c_amr_model.cp:.1f}")
            self.camr_fd_label.setText(f"{c_amr_model.fd:.1f}")
            
        robots_approaching, camr_approaching = interaction_data
        self.approaching_camr_label.setText(", ".join(robots_approaching) if robots_approaching else "None")
        self.camr_approaching_label.setText(", ".join(camr_approaching) if camr_approaching else "None")
        
        new_station_ids = sorted([s.station_id for s in stations])
        if new_station_ids != self.displayed_station_ids:
            self.displayed_station_ids = new_station_ids
            current_selection = self.station_selector_combo.currentData()
            self.station_selector_combo.blockSignals(True)
            self.station_selector_combo.clear()
            self.station_selector_combo.addItem("None", -1)
            for station_id in self.displayed_station_ids:
                self.station_selector_combo.addItem(f"Station {station_id}", station_id)
            index = self.station_selector_combo.findData(current_selection)
            if index != -1:
                self.station_selector_combo.setCurrentIndex(index)
            self.station_selector_combo.blockSignals(False)

    def _on_go_to_station_clicked(self):
        station_id = self.station_selector_combo.currentData()
        if station_id is not None and station_id != -1:
            self.go_to_station_requested.emit(station_id)

    def update_mouse_coords(self, coords_text: str):
        self.mouse_coords_label.setText(coords_text)
        
    def get_config(self) -> dict:
        return {
            "map_width_m": self.map_width_spinbox.value(),
            "map_height_m": self.map_height_spinbox.value(),
            "markov_radius_m": self.markov_radius_spinbox.value(),
        }
        
    def _on_edit_mode_toggled(self, mode: str, checked: bool):
        if checked:
            if mode == "obstacle":
                self.add_node_button.setChecked(False)
                self.add_station_button.setChecked(False)
            elif mode == "node":
                self.add_obstacle_button.setChecked(False)
                self.add_station_button.setChecked(False)
            elif mode == "station":
                self.add_obstacle_button.setChecked(False)
                self.add_node_button.setChecked(False)
        self.edit_mode_toggled.emit(mode, checked)
        
    def show_status_message(self, message: str, timeout: int):
        self.statusBar().showMessage(message, timeout)


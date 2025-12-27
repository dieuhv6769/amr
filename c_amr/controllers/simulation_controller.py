# c_amr/controllers/simulation_controller.py
import json
import os
from datetime import datetime
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from PyQt5.QtCore import QTimer

from c_amr.core.agent_core import HeadlessAMRAgent
from c_amr.views.main_view import MainView
from c_amr.views.layer_dialogs import LayerViewDialog, LiveLayerViewDialog
from c_amr.views.item_views import ObstacleItemView
from c_amr.utils.map_generator import generate_detail_traffic_map_layout, export_layout_to_svg

class SimulationController:
    def __init__(self, agent: HeadlessAMRAgent, view: MainView):
        self.agent = agent
        self.view = view
        self.current_edit_mode = "none"
        self._connect_signals()
        
        self.agent.set_markov_radius(self.view.markov_radius_spinbox.value())

    def _connect_signals(self):
        self.view.restart_button_clicked.connect(self.agent.restart_simulation)
        self.view.path_requested.connect(self._handle_path_request)
        self.view.markov_radius_changed.connect(self.agent.set_markov_radius)
        self.view.edit_mode_toggled.connect(self._handle_edit_mode_toggle)
        self.agent.model.path_found.connect(self.view.scene.draw_camr_mission_path_destination_point)
        self.view.show_layer_requested.connect(
            lambda num: QTimer.singleShot(0, lambda: self._handle_show_layer_request(num))
        )
        self.view.import_obstacles_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_import_obstacles)
        )
        self.view.import_network_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_import_network)
        )
        self.view.export_obstacles_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_export_obstacles)
        )
        self.view.export_network_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_export_network)
        )
        self.view.build_map_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_build_new_map)
        )
        self.view.export_svg_requested.connect(
            lambda: QTimer.singleShot(0, self._handle_export_svg)
        )
        
        self.view.scene.map_clicked_signal.connect(self._handle_map_click)
        self.view.go_to_station_requested.connect(
            lambda station_id: self.agent.model.set_camr_target_station(station_id, self.view.planning_mode_checkbox.isChecked())
        )
        self.view.zoom_to_fit_requested.connect(self.view.view.zoom_to_fit)

        self.agent.model.map_layout_changed.connect(self._on_map_layout_changed)
        self.agent.model.model_updated.connect(self._on_model_update)

        self.agent.model.status_message_changed.connect(self.view.show_status_message)

    def _handle_build_new_map(self):
        try:
            map_config = self.view.get_config()
            layout_data = generate_detail_traffic_map_layout(map_config)
            
            if layout_data:
                self.agent.model.load_new_generated_layout(layout_data)
                self.view.show_status_message("Successfully built and loaded new map.", 4000)
            else:
                raise ValueError("Map generation returned no data.")
        except Exception as e:
            QMessageBox.critical(self.view, "Error", f"Could not build map:\n{e}")

    def _handle_export_svg(self):
        if not self.agent.model.obstacles:
            QMessageBox.information(self.view, "Info", "There is no map to export.")
            return

        try:
            data_dir = os.path.join(os.getcwd(), "data")
            os.makedirs(data_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"map_{timestamp}.svg"
            output_path = os.path.join(data_dir, filename)

            obstacles_data = self.agent.model.export_obstacles_to_dict()
            network_data = self.agent.model.export_network_to_dict()
            
            layout_to_export = {
                "obstacles": obstacles_data.get("obstacles", []),
                "stations": network_data.get("stations", [])
            }

            map_config = self.view.get_config()
            
            export_layout_to_svg(output_path, layout_to_export, map_config)
            
            self.view.show_status_message(f"Map exported to: {output_path}", 6000)

        except Exception as e:
            QMessageBox.critical(self.view, "Error", f"Could not export SVG:\n{e}")

    def _handle_import_obstacles(self):
        path, _ = QFileDialog.getOpenFileName(self.view, "Import Obstacles JSON", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.agent.model.import_obstacles_from_dict(data)
                self.view.show_status_message(f"Imported obstacles from {path}", 3000)
            except Exception as e:
                QMessageBox.critical(self.view, "Error", f"Could not import file:\n{e}")

    def _handle_import_network(self):
        path, _ = QFileDialog.getOpenFileName(self.view, "Import Network JSON", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.agent.model.import_network_from_dict(data)
                self.view.show_status_message(f"Imported network from {path}", 3000)
            except Exception as e:
                QMessageBox.critical(self.view, "Error", f"Could not import file:\n{e}")

    def _handle_export_obstacles(self):
        path, _ = QFileDialog.getSaveFileName(self.view, "Export Obstacles JSON", "obstacles.json", "JSON Files (*.json)")
        if path:
            if not path.lower().endswith('.json'): path += '.json'
            try:
                data = self.agent.model.export_obstacles_to_dict()
                with open(path, 'w') as f: json.dump(data, f, indent=4)
                self.view.show_status_message(f"Exported obstacles to {path}", 3000)
            except Exception as e:
                QMessageBox.critical(self.view, "Error", f"Could not export file:\n{e}")

    def _handle_export_network(self):
        path, _ = QFileDialog.getSaveFileName(self.view, "Export Network JSON", "nodes_stations.json", "JSON Files (*.json)")
        if path:
            if not path.lower().endswith('.json'): path += '.json'
            try:
                data = self.agent.model.export_network_to_dict()
                with open(path, 'w') as f: json.dump(data, f, indent=4)
                self.view.show_status_message(f"Exported network to {path}", 3000)
            except Exception as e:
                QMessageBox.critical(self.view, "Error", f"Could not export file:\n{e}")

    def _handle_path_request(self, goal_pos):
        use_road_network = self.view.planning_mode_checkbox.isChecked()
        self.agent.plan_path(goal_pos, use_road_network)

    def _handle_show_layer_request(self, layer_number: int):
        if layer_number == 4:
            dialog = LiveLayerViewDialog(self.agent.model, self.view)
        else:
            dialog = LayerViewDialog(self.agent.model, layer_number, self.view)
        dialog.exec_()

    def _handle_edit_mode_toggle(self, mode: str, checked: bool):
        if checked:
            self.current_edit_mode = mode
            self.agent.stop()
        else:
            if not (self.view.add_obstacle_button.isChecked() or 
                    self.view.add_node_button.isChecked() or 
                    self.view.add_station_button.isChecked()):
                self.current_edit_mode = "none"
                self.agent.start()
        self.view.view.set_edit_mode(self.current_edit_mode)

    def _handle_map_click(self, pos, item):
        if self.current_edit_mode == "node":
            if not isinstance(item, ObstacleItemView):
                self.agent.model.add_node(pos)
        elif self.current_edit_mode == "station":
            if isinstance(item, ObstacleItemView):
                self.agent.model.add_station(pos, item.model)
        elif self.current_edit_mode == "obstacle":
            self.agent.model.add_obstacle(item)

    def _on_map_layout_changed(self):
        self.view.scene.update_static_layout(self.agent.model)

    def _on_model_update(self):
        self.view.scene.update_dynamic_items(self.agent.model)

        interaction_data = self.agent.model.get_interaction_data()
        stations = self.agent.model.stations
        self.view.update_info_panel(self.agent.model.c_amr, interaction_data, stations)

# c_amr/core/agent_core.py
import time
from PyQt5.QtCore import QObject, QTimer, QCoreApplication

from c_amr.core.simulation_model import SimulationModel
from c_amr.ros.ros2_communicator import ROS2Communicator
import c_amr.constants as const

class HeadlessAMRAgent(QObject):
    def __init__(self, use_gui=False):
        super().__init__()
        self.use_gui = use_gui
        
        if QCoreApplication.instance() is None:
            self._app = QCoreApplication([])

        self.model = SimulationModel()
        self.ros_communicator = ROS2Communicator(self.model)
        # self.model.camr_state_publish_request.connect(self.ros_communicator.publish_camr_state)

        if self.use_gui:
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._tick)
        else:
            self.timer = None

    def _tick(self):
        self.model.update_simulation_tick()

    def run(self):
        self.model.load_default_map()
        
        if self.use_gui:
            self.start()
            print("Agent Core running with QTimer for GUI mode.")
        else:
            print("Agent Core running in headless loop. Press Ctrl+C to exit.")
            try:
                while True:
                    self._tick()
                    time.sleep(const.UPDATE_INTERVAL_MS / 1000.0)
            except KeyboardInterrupt:
                print("\nShutting down headless agent.")

    def start(self):
        if self.timer and not self.timer.isActive():
            self.timer.start(const.UPDATE_INTERVAL_MS)

    def stop(self):
        if self.timer:
            self.timer.stop()

    def shutdown(self):
        print("Shutting down Agent Core...")
        self.stop()

    def restart_simulation(self, config):
        self.model.restart(config)

    def plan_path(self, goal_pos, use_road_network):
        self.model.plan_path_for_camr(goal_pos, use_road_network)
    
    def set_markov_radius(self, radius_m):
        self.model.set_camr_markov_radius(radius_m)
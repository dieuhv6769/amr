# c_amr/gui_main.py
import sys
import rclpy
from PyQt5.QtWidgets import QApplication

from c_amr.core.agent_core import HeadlessAMRAgent
from c_amr.controllers.simulation_controller import SimulationController
from c_amr.views.main_view import MainView

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    agent_core = HeadlessAMRAgent(use_gui=True)
    view = MainView()
    controller = SimulationController(agent_core, view)
    agent_core.run()
    view.showMaximized()
    exit_code = app.exec_()
    agent_core.shutdown()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()

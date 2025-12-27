# c_amr/ros/ros2_communicator.py
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QPointF

from c_amr import constants as const
from c_amr.core.simulation_model import SimulationModel

try:
    from amr_interfaces.msg import AmrState
except ImportError:
    print("Cảnh báo: Không thể import message tùy chỉnh 'AmrState'.")
    print("Vui lòng tạo và build package 'amr_interfaces' với 'AmrState.msg'.")
    AmrState = None


class ROS2Communicator(Node):
    """
    Quản lý các publisher và subscriber ROS2 cho C-AMR.
    """
    def __init__(self, model: SimulationModel):
        super().__init__(f'amr_{const.C_AMR_ID:03d}_ui_node')
        self.model = model
        self.is_initialized = False

        if AmrState is None:
            self.get_logger().error("Không thể khởi tạo ROS2 communicator nếu không có message AmrState.")
            return

        publisher_topic = f'/amr_{const.C_AMR_ID:03d}'
        self.publisher_ = self.create_publisher(AmrState, publisher_topic, 10)
        self.get_logger().info(f"Publisher đã bắt đầu cho topic '{publisher_topic}'")

        self.subscribers = []
        for i in range(1, const.TOTAL_ROBOT_COUNT + 1):
            if i == const.C_AMR_ID:
                continue

            # THAY ĐỔI: Cập nhật định dạng topic
            topic_name = f'/amr_{i:03d}'
            sub = self.create_subscription(
                AmrState,
                topic_name,
                # Tạo callback với lambda để truyền ID của robot
                lambda msg, robot_id=i: self.amr_state_callback(msg, robot_id),
                10)
            self.subscribers.append(sub)
            self.get_logger().info(f"Đã subscribe vào topic '{topic_name}'")
        
        self.is_initialized = True

    def amr_state_callback(self, msg: AmrState, robot_id: int):
        """
        Hàm callback được gọi khi nhận được message từ một robot khác.
        """
        # THAY ĐỔI: Chuyển toàn bộ dữ liệu từ message vào một dictionary
        state_data = {
            "pos": QPointF(msg.x, msg.y),
            "heading": msg.heading,
            "tv": msg.tv,
            "rv": msg.rv,
            "soc": msg.soc,
            "np": msg.np,
            "cp": msg.cp,
            "fd": msg.fd
        }
        self.model.update_fleet_robot_state(robot_id, state_data)

    def publish_camr_state(self):
        """
        Publish trạng thái hiện tại của C-AMR.
        """
        if not self.is_initialized or not self.model.c_amr:
            return
            
        camr = self.model.c_amr
        msg = AmrState()
        msg.x = camr.pos.x()
        msg.y = camr.pos.y()
        msg.heading = camr.heading
        msg.tv = camr.tv
        msg.rv = camr.rv
        msg.soc = camr.soc
        msg.np = camr.np
        msg.cp = camr.cp
        msg.fd = camr.fd
        
        self.publisher_.publish(msg)

    def spin(self):
        """
        Xử lý một loạt các sự kiện ROS2.
        """
        rclpy.spin_once(self, timeout_sec=0)

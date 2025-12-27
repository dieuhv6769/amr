import math
from PyQt5.QtCore import QPointF

class InteractionLogic:
    @staticmethod
    def calculate_markov_interactions(c_amr_model, all_robots: dict):
        robots_approaching_camr = []
        camr_approaching_robots = []

        if not c_amr_model or c_amr_model.markov_radius <= 0:
            return robots_approaching_camr, camr_approaching_robots

        p_camr = c_amr_model.pos
        v_camr = QPointF(c_amr_model.dx, c_amr_model.dy)

        for robot_id, other_robot in all_robots.items():
            if other_robot is c_amr_model:
                continue

            p_other = other_robot.pos
            v_other = QPointF(other_robot.dx, other_robot.dy)

            d_vec = p_other - p_camr
            distance = math.hypot(d_vec.x(), d_vec.y())

            if distance < c_amr_model.markov_radius:
                dot_camr_to_other = QPointF.dotProduct(v_camr, d_vec)
                if dot_camr_to_other > 0:
                    camr_approaching_robots.append(str(robot_id))

                dot_other_to_camr = QPointF.dotProduct(v_other, -d_vec)
                if dot_other_to_camr > 0:
                    robots_approaching_camr.append(str(robot_id))

        return robots_approaching_camr, camr_approaching_robots

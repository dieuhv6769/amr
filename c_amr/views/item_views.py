# c_amr/views/item_views.py
# Chứa tất cả các lớp QGraphicsItem để hiển thị các đối tượng từ model.
# Các lớp này chỉ chịu trách nhiệm vẽ, không chứa logic nghiệp vụ.

import math
import bisect
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsEllipseItem, QGraphicsItem
from PyQt5.QtGui import QBrush, QPen, QPolygonF, QPainterPath, QColor, QFont
from PyQt5.QtCore import QRectF, QPointF, Qt

import c_amr.constants as const
from c_amr.models.robot_model import RobotModel, CentralAMRAgentModel
from c_amr.models.obstacle_model import ObstacleModel
from c_amr.models.map_elements import NodeModel, StationModel
from c_amr.utils.traffic_calculator import BsplineUtils

class ObstacleItemView(QGraphicsRectItem):
    def __init__(self, model: ObstacleModel):
        super().__init__(model.full_rect)
        self.model = model
        self.setPen(QPen(Qt.PenStyle.NoPen))
        self.setCacheMode(QGraphicsItem.CacheMode.DeviceCoordinateCache)

    def paint(self, painter, option, widget):
        walkable_color = const.OBSTACLE_WALKABLE_COLOR
        painter.setBrush(QBrush(walkable_color))
        painter.setPen(QPen(walkable_color.darker(110)))
        painter.drawRect(self.rect()) 
        
        inner_rect = self.model.solid_rect
        if inner_rect.isValid():
            painter.setBrush(QBrush(const.OBSTACLE_SOLID_COLOR))
            painter.setPen(QPen(const.OBSTACLE_SOLID_PEN_COLOR))
            local_inner_rect = self.mapFromScene(inner_rect).boundingRect()
            painter.drawRect(local_inner_rect)

class RobotItemView(QGraphicsItem):
    def __init__(self, model: RobotModel, robot_size_px: float):
        super().__init__()
        self.model = model
        self.robot_size = robot_size_px
        self.setPos(model.pos)
        self.setZValue(1)

    def boundingRect(self):
        half_size = self.robot_size / 2
        return QRectF(-half_size - 5, -half_size - 5, self.robot_size + 10, self.robot_size + 20)

    def shape(self):
        path = QPainterPath()
        half_size = self.robot_size / 2
        path.addEllipse(-half_size, -half_size, self.robot_size, self.robot_size)
        return path

    def paint(self, painter, option, widget):
        state_colors = {
            "IDLE": const.STATE_COLOR_IDLE, 
            "MOVING": const.STATE_COLOR_MOVING,
            "CHARGING": const.STATE_COLOR_CHARGING, 
            "Low Battery": const.STATE_COLOR_LOW_BATTERY
        }
        color = state_colors.get(self.model.state, QColor("gray"))
        if self.model.is_c_amr:
            color = const.C_AMR_COLOR
            
        half_size = self.robot_size / 2
        body_rect = QRectF(-half_size, -half_size, self.robot_size, self.robot_size)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(Qt.PenStyle.NoPen))
        painter.drawEllipse(body_rect)

        if self.model.dx != 0 or self.model.dy != 0:
            painter.setBrush(QBrush(QColor("white")))
            angle = self.model.heading
            painter.save()
            painter.rotate(angle)
            arrow_scale = self.robot_size / 2.0
            points = [QPointF(arrow_scale * 0.6, 0), QPointF(-arrow_scale * 0.3, -arrow_scale * 0.5),
                      QPointF(0, 0), QPointF(-arrow_scale * 0.3, arrow_scale * 0.5)]
            arrow = QPolygonF(points)
            painter.drawPolygon(arrow)
            painter.restore()

        painter.setFont(const.ROBOT_ID_FONT)
        painter.setPen(QPen(QColor("black")))
        id_text = str(self.model.robot_id)
        font_metrics = painter.fontMetrics()
        text_width = font_metrics.horizontalAdvance(id_text)
        text_x = -text_width / 2
        text_y = half_size + font_metrics.height()
        painter.drawText(QPointF(text_x, text_y), id_text)

class CentralAMRItemView(RobotItemView):
    def __init__(self, model: CentralAMRAgentModel, robot_size_px: float):
        super().__init__(model, robot_size_px)
        self.setZValue(10)

    def boundingRect(self):
        extra = self.model.markov_radius + 5
        base_rect = super().boundingRect()
        return base_rect.adjusted(-extra, -extra, extra, extra)

    def paint(self, painter, option, widget):
        if self.model.is_c_amr and self.model.markov_radius > 0:
            painter.setPen(QPen(Qt.PenStyle.NoPen))
            painter.setBrush(QBrush(const.MARKOV_BLANKET_COLOR))
            center = QPointF(0, 0)
            painter.drawEllipse(center, self.model.markov_radius, self.model.markov_radius)
        
        super().paint(painter, option, widget)

class NodeItemView(QGraphicsEllipseItem):
    def __init__(self, model: NodeModel):
        size = const.NODE_DIAMETER
        super().__init__(-size/2, -size/2, size, size)
        self.setPos(model.pos)
        self.model = model
        self.setBrush(QBrush(const.NODE_BRUSH_COLOR))
        self.setPen(QPen(const.NODE_PEN_COLOR, 1))

    def boundingRect(self):
        return super().boundingRect().adjusted(-10, -2, 10, 15)

    def paint(self, painter, option, widget):
        super().paint(painter, option, widget)
        painter.setFont(const.NODE_ID_FONT)
        painter.setPen(QPen(const.NODE_PEN_COLOR))
        id_text = str(self.model.node_id)
        font_metrics = painter.fontMetrics()
        text_width = font_metrics.horizontalAdvance(id_text)
        text_height = font_metrics.height()
        text_x = -text_width / 2
        text_y = (self.rect().height() / 2) + text_height
        painter.drawText(QPointF(text_x, text_y), id_text)

class StationItemView(QGraphicsRectItem):
    def __init__(self, model: StationModel):
        size = const.STATION_SIZE
        super().__init__(-size/2, -size/2, size, size)
        self.setPos(model.pos)
        self.model = model
        self.setBrush(QBrush(const.STATION_BRUSH_COLOR))
        self.setPen(QPen(const.STATION_PEN_COLOR, 1))
        self.setZValue(0.5)

    def boundingRect(self):
        return super().boundingRect().adjusted(-10, -2, 10, 15)

    def paint(self, painter, option, widget):
        super().paint(painter, option, widget)
        painter.setFont(const.NODE_ID_FONT)
        painter.setPen(QPen(QColor("black")))
        id_text = str(self.model.station_id)
        font_metrics = painter.fontMetrics()
        text_width = font_metrics.horizontalAdvance(id_text)
        text_height = font_metrics.height()
        text_x = -text_width / 2
        text_y = (self.rect().height() / 2) + text_height
        painter.drawText(QPointF(text_x, text_y), id_text)

class TrafficNetworkItem(QGraphicsItem):
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.setZValue(-1)
        self.setCacheMode(QGraphicsItem.CacheMode.ItemCoordinateCache)

    def boundingRect(self):
        return self.model.map_rect

    def _draw_spline_turn_qt(self, painter, control_points_data, pen):
        if len(control_points_data) < 2: return

        painter.setPen(pen)
        control_points_tuples = [(p['x'], p['y']) for p in control_points_data]
        curve_points = BsplineUtils.bspline_curve(control_points_tuples, p=3, num_points=50)

        if len(curve_points) < 2: return

        path = QPainterPath()
        path.moveTo(QPointF(curve_points[0][0], curve_points[0][1]))
        for point in curve_points[1:]:
            path.lineTo(QPointF(point[0], point[1]))
        
        painter.drawPath(path)

    def paint(self, painter, option, widget):
        h_paths = self.model.horizontal_paths
        v_corridors = self.model.vertical_corridors

        if not h_paths and not v_corridors:
            return

        red_pen = QPen(QColor("red"), const.TRAFFIC_PEN_WIDTH)
        green_pen = QPen(QColor("green"), const.TRAFFIC_PEN_WIDTH)
        purple_pen = QPen(QColor("purple"), const.TRAFFIC_PEN_WIDTH)
        shared_turn_pen = QPen(QColor("blue"), const.TRAFFIC_PEN_WIDTH)
        divider_pen = QPen(QColor("black"), const.TRAFFIC_PEN_WIDTH, Qt.PenStyle.DashLine)
        
        self._draw_straight_lanes(painter, red_pen, green_pen, purple_pen, divider_pen)
        self._draw_intersections(painter, red_pen, green_pen, shared_turn_pen)

    def _draw_straight_lanes(self, painter, red_pen, green_pen, purple_pen, divider_pen):
        for h_path in self.model.horizontal_paths:
            segments = h_path['segments']
            for seg in segments:
                p1 = QPointF(seg['x1'], seg['y'])
                p2 = QPointF(seg['x2'], seg['y'])
                if seg['is_wide']:
                    L = seg['gap'] / 4.0
                    painter.setPen(green_pen)
                    painter.drawLine(int(p1.x()), int(p1.y() - L), int(p2.x()), int(p2.y() - L))
                    painter.setPen(red_pen)
                    painter.drawLine(int(p1.x()), int(p1.y() + L), int(p2.x()), int(p2.y() + L))
                    painter.setPen(divider_pen)
                    painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p2.y()))
                else:
                    painter.setPen(purple_pen)
                    painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p2.y()))

            for i in range(len(segments) - 1):
                seg1 = segments[i]
                seg2 = segments[i+1]
                if seg1['is_wide'] != seg2['is_wide']:
                    T_W = 40
                    x_trans = seg1['x2']
                    x_destination = seg2['x1']
                    x_center = (x_trans + x_destination) / 2
                    
                    if seg1['is_wide']:
                        L1 = seg1['gap'] / 4.0
                        y_wide_top, y_wide_bottom = seg1['y'] - L1, seg1['y'] + L1
                        y_narrow = seg2['y']

                        y_top_center = (y_wide_top + y_narrow) / 2
                        y_bot_center = (y_wide_bottom + y_narrow) / 2

                        cps_top = [{'x': x_trans - T_W, 'y': y_wide_top}, {'x': x_trans, 'y': y_wide_top}, {'x': x_center, 'y': y_top_center}, {'x': x_destination, 'y': y_narrow}, {'x': x_destination + T_W, 'y': y_narrow}]
                        cps_bot = [{'x': x_trans - T_W, 'y': y_wide_bottom}, {'x': x_trans, 'y': y_wide_bottom}, {'x': x_center, 'y': y_bot_center}, {'x': x_destination, 'y': y_narrow}, {'x': x_destination + T_W, 'y': y_narrow}]
                        
                        self._draw_spline_turn_qt(painter, cps_top, purple_pen)
                        self._draw_spline_turn_qt(painter, cps_bot, purple_pen)
                    else:
                        L2 = seg2['gap'] / 4.0
                        y_narrow = seg1['y']
                        y_wide_top, y_wide_bottom = seg2['y'] - L2, seg2['y'] + L2
                        y_top_center = (y_wide_top + y_narrow) / 2
                        y_bot_center = (y_wide_bottom + y_narrow) / 2
                        cps_top = [{'x': x_trans - T_W, 'y': y_narrow}, {'x': x_trans, 'y': y_narrow}, {'x': x_center, 'y': y_top_center}, {'x': x_destination, 'y': y_wide_top}, {'x': x_destination + T_W, 'y': y_wide_top}]
                        cps_bot = [{'x': x_trans - T_W, 'y': y_narrow}, {'x': x_trans, 'y': y_narrow}, {'x': x_center, 'y': y_bot_center}, {'x': x_destination, 'y': y_wide_bottom}, {'x': x_destination + T_W, 'y': y_wide_bottom}]
                        self._draw_spline_turn_qt(painter, cps_top, purple_pen)
                        self._draw_spline_turn_qt(painter, cps_bot, purple_pen)

        for v_corridor in self.model.vertical_corridors:
            x, width = v_corridor['x'], v_corridor['width']
            is_wide = width >= self.model.traffic_config_params.get("NARROW_LANE_THRESHOLD_PX", 60)
            p1 = QPointF(x, 0)
            p2 = QPointF(x, self.model.map_rect.height())

            if is_wide:
                L = width / 4.0
                painter.setPen(red_pen); painter.drawLine(int(p1.x() - L), int(p1.y()), int(p2.x() - L), int(p2.y()))
                painter.setPen(green_pen); painter.drawLine(int(p1.x() + L), int(p1.y()), int(p2.x() + L), int(p2.y()))
                painter.setPen(divider_pen); painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p2.y()))
            else:
                painter.setPen(purple_pen); painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p2.y()))

    def _draw_intersections(self, painter, red_pen, green_pen, shared_pen):
        for inter in self.model.intersections:
            p = inter['points']
            is_wide_h = inter['is_wide_h']
            is_wide_v = inter['is_wide_v']
            is_real = inter['type'] == 'real'
            
            if not (is_wide_h and is_wide_v and is_real):
                continue
            
            if is_wide_h and is_wide_v:
                if is_real:
                    cps_RL_RD = [{'x': p['RL_adj']['x'] - 20,'y':p['RL_adj']['y']}, 
                                 p['RL_adj'], p['RL'], p['RD_adj'], 
                                 {'x': p['RD_adj']['x'],'y':p['RD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_RL_RD, red_pen)

                    cps_GD_RR = [{'x': p['GD_adj']['x'], 'y': p['GD_adj']['y'] + 20},
                                 p['GD_adj'], p['RR'], p['RR_adj'],
                                 {'x': p['RR_adj']['x'] + 20, 'y': p['RR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_GD_RR, green_pen)

                    cps_GR_GU = [{'x':p['GR_adj']['x'] + 20, 'y':p['GR_adj']['y']},
                                   p['GR_adj'], p['GR'], p['GU_adj'], 
                                   {'x':p['GU_adj']['x'],'y':p['GU_adj']['y'] - 20}]
                    self._draw_spline_turn_qt(painter, cps_GR_GU, green_pen)

                    cps_RD_GL = [{'x':p['RU_adj']['x'],'y':p['RU_adj']['y']-20},
                                 p['RU_adj'],p['GL'],p['GL_adj'],
                                 {'x':p['GL_adj']['x'] - 20, 'y':p['GL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_RD_GL, red_pen)

                    cps_GR_LD = [p['GR_adj'],p['GR'], p['center'], p['RL'], p['RD_adj']]
                    self._draw_spline_turn_qt(painter, cps_GR_LD, red_pen)

                    cps_RU_RR = [p['RU_adj'],p['GL'], p['center'], p['RR'], p['RR_adj']]
                    self._draw_spline_turn_qt(painter, cps_RU_RR, red_pen)

                    cps_RL_GU = [p['RL_adj'], p['RL'], p['center'], p['GU'], p['GU_adj']]
                    self._draw_spline_turn_qt(painter, cps_RL_GU, green_pen)

                    cps_GD_GL = [p['GD_adj'], p['GD'], p['center'], p['GL'], p['GL_adj']]
                    self._draw_spline_turn_qt(painter, cps_GD_GL, green_pen)
                else:
                    if inter['virtual_side'] == 'left':
                        cps_RU_GL = [{'x' : p['RU_adj']['x'], 'y' : p['RU_adj']['y']-20}, 
                                     p['RU_adj'], p['RU'], p['GL_adj'], 
                                     {'x':p['GL_adj']['x']-20,'y':p['GL_adj']['y']}]
                        self._draw_spline_turn_qt(painter, cps_RU_GL, red_pen)

                        cps_RL_RD = [{'x':p['RL_adj']['x']-20,'y':p['RL_adj']['y']},
                                     p['RL_adj'], p['RD'], p['RD_adj'],
                                     {'x':p['RD_adj']['x'], 'y':p['RD_adj']['y']+20}]
                        self._draw_spline_turn_qt(painter, cps_RL_RD, red_pen)

                        cps_RL_GU = [p['RL_adj'],p['RL'],p['center'],p['GU'], p['GU_adj']]
                        self._draw_spline_turn_qt(painter, cps_RL_GU, green_pen)

                        cps_GD_GL = [p['GD_adj'],p['GD'],p['center'],p['GL'], p['GL_adj']]
                        self._draw_spline_turn_qt(painter, cps_GD_GL, green_pen)

                    elif inter['virtual_side'] == 'right':
                        cps_GR_GU = [{'x':p['GR_adj']['x']+20,'y':p['GR_adj']['y']},
                                     p['GR_adj'], p['GR'], p['GU_adj'],
                                     {'x':p['GU_adj']['x'], 'y':p['GU_adj']['y']-20}]
                        self._draw_spline_turn_qt(painter, cps_GR_GU, green_pen)

                        cps_GD_RR = [{'x':p['GD_adj']['x'],'y':p['GD_adj']['y']+20},
                                     p['GD_adj'], p['GD'], p['RR_adj'],
                                     {'x':p['RR_adj']['x']+20, 'y':p['RR_adj']['y']}]
                        self._draw_spline_turn_qt(painter, cps_GD_RR, green_pen)

                        cps_RU_RR = [p['RU_adj'], p['RU'], p['center'], p['RR'], p['RR_adj']]
                        self._draw_spline_turn_qt(painter, cps_RU_RR, red_pen)

                        cps_GR_LD = [p['GR_adj'], p['GR'], p['center'], p['RD'], p['RD_adj']]
                        self._draw_spline_turn_qt(painter, cps_GR_LD, red_pen)

            elif not is_wide_h and is_wide_v:
                if is_real:
                    cps_PL_RD = [{'x':p['RL_adj']['x'] - 20, 'y':p['RL_adj']['y']},
                                 p['RL_adj'], p['RL'], p['RD_adj'],
                                 {'x':p['RD_adj']['x'], 'y':p['RD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_PL_RD, red_pen)

                    cps_RU_PL = [{'x':p['RU_adj']['x'], 'y':p['RU_adj']['y'] - 20},
                                 p['RU_adj'], p['RL'], p['RL_adj'],
                                 {'x':p['RL_adj']['x'] - 20, 'y':p['RL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_RU_PL, red_pen)

                    cps_PL_GU = [p['RL_adj'],
                                 p['RL'], p['center'], p['GU_adj'], 
                                 {'x':p['GU_adj']['x'], 'y':p['GU_adj']['y'] - 20}]
                    self._draw_spline_turn_qt(painter, cps_PL_GU, green_pen)

                    cps_PL_GD = [p['RL_adj'],
                                 p['RL'], p['center'], p['GD_adj'], 
                                 {'x':p['GD_adj']['x'], 'y':p['GD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_PL_GD, green_pen)

                    cps_PR_GU = [{'x':p['RR_adj']['x'] + 20, 'y':p['RR_adj']['y']},
                                 p['RR_adj'], p['RR'], p['GU_adj'],
                                 {'x': p['GU_adj']['x'], 'y': p['GU_adj']['y'] - 20}]
                    self._draw_spline_turn_qt(painter, cps_PR_GU, green_pen) 

                    cps_GD_PR = [{'x':p['GD_adj']['x'], 'y':p['GD_adj']['y'] + 20}, 
                                   p['GD_adj'], p['GD'], p['GR_adj'], 
                                   {'x': p['GR_adj']['x'] + 20, 'y': p['GR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_GD_PR, green_pen)

                    cps_PR_RD = [p['RR_adj'], p['RR'], p['center'], p['RD_adj'], 
                                   {'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_PR_RD, red_pen)

                    cps_RU_PR = [{'x':p['RU_adj']['x'],'y':p['RU_adj']['y'] - 20},
                                 p['RU_adj'], p['center'], p['RR'], p['RR_adj']]
                    self._draw_spline_turn_qt(painter, cps_RU_PR, red_pen)
                else:
                    if inter['virtual_side'] == 'left':
                        cps_PL_RD = [{'x':p['RL_adj']['x'] - 20, 'y':p['RL_adj']['y']},
                                     p['RL_adj'], p['RL'], p['RD_adj'],
                                     {'x':p['RD_adj']['x'], 'y':p['RD_adj']['y'] + 20}]
                        self._draw_spline_turn_qt(painter, cps_PL_RD, red_pen)
                        
                        cps_RU_PL = [{'x':p['RU_adj']['x'], 'y':p['RU_adj']['y'] - 20},
                                       p['RU_adj'], p['RL'], p['RL_adj'],
                                       {'x':p['RL_adj']['x'] - 20, 'y':p['RL_adj']['y']}]
                        self._draw_spline_turn_qt(painter, cps_RU_PL, red_pen)

                        cps_PL_GU = [p['RL_adj'], p['RL'], p['center'], p['GU_adj'], 
                                     {'x':p['GU_adj']['x'], 'y':p['GU_adj']['y'] - 20}]
                        self._draw_spline_turn_qt(painter, cps_PL_GU, green_pen)

                        cps_PL_GD = [p['RL_adj'], p['RL'], p['center'], p['GD_adj'], 
                                     {'x':p['GD_adj']['x'], 'y':p['GD_adj']['y'] + 20}]
                        self._draw_spline_turn_qt(painter, cps_PL_GD, green_pen)
                    else:
                        cps_PR_GU = [{'x':p['RR_adj']['x']+20, 'y':p['RR_adj']['y']},
                                     p['RR_adj'], p['RR'], p['GU_adj'], 
                                     {'x': p['GU_adj']['x'], 'y': p['GU_adj']['y'] - 20}]
                        self._draw_spline_turn_qt(painter, cps_PR_GU, green_pen)

                        cps_GD_PR = [{'x':p['GD_adj']['x'], 'y':p['GD_adj']['y']+20}, 
                                       p['GD_adj'], p['GD'], p['GR_adj'], 
                                       {'x': p['GR_adj']['x']+20, 'y': p['GR_adj']['y']}]
                        self._draw_spline_turn_qt(painter, cps_GD_PR, green_pen)

                        cps_PR_RD = [p['RR_adj'], p['RR'],p['center'], p['RD_adj'], 
                                     {'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}]
                        self._draw_spline_turn_qt(painter, cps_PR_RD, red_pen)

                        cps_RU_PR = [{'x':p['RU_adj']['x'],'y':p['RU_adj']['y'] - 20},
                                     p['RU_adj'], p['center'], p['RR'], p['RR_adj']]
                        self._draw_spline_turn_qt(painter, cps_RU_PR, red_pen)

            elif is_wide_h and not is_wide_v:
                if is_real:
                    cps_GR_PU = [{'x':p['GR_adj']['x'] + 20, 'y':p['GR_adj']['y']},
                                 p['GR_adj'], p['GU'], p['GU_adj'],
                                 {'x':p['GU_adj']['x'], 'y':p['GU_adj']['y'] - 20}]
                    self._draw_spline_turn_qt(painter, cps_GR_PU, green_pen)

                    cps_PD_RR = [{'x':p['GD_adj']['x'], 'y':p['GD_adj']['y'] + 20}, 
                                 p['GD_adj'], p['RR'], p['RR_adj'],
                                 {'x':p['RR_adj']['x'] + 20, 'y':p['RR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PD_RR, green_pen)

                    cps_PU_GL = [{'x':p['RU_adj']['x'], 'y':p['RU_adj']['y'] - 20},
                                 p['RU_adj'],p['GL'], p['GL_adj'],
                                 {'x':p['GL_adj']['x'] - 20, 'y':p['GL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PU_GL, red_pen)

                    cps_RL_PD = [{'x':p['RL_adj']['x'], 'y':p['RL_adj']['y']}, 
                                 p['RL_adj'], p['RD'], p['RD_adj'], 
                                 {'x':p['RD_adj']['x'], 'y':p['RD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_RL_PD, red_pen)

                    cps_PD_GL = [p['GD'], p['GL'], p['GL_adj'], 
                                 {'x':p['GL_adj']['x'] - 20, 'y':p['GL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PD_GL, green_pen)

                    cps_PU_RR = [p['RU'], p['RR'], p['RR_adj'], 
                                 {'x':p['RR_adj']['x'] + 20, 'y':p['RR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PU_RR, red_pen)

                    cps_PD_GR = [p['GD'], p['GR'], p['GR_adj'], 
                                 {'x':p['GR_adj']['x'] + 20, 'y':p['GR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PD_GR, red_pen)

                    cps_PU_RL = [p['RU'], p['RL'], p['RL_adj'], 
                                 {'x': p['RL_adj']['x'] - 20, 'y': p['RL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PU_RL, red_pen)

            elif not is_wide_h and not is_wide_v:
                if is_real: 
                    cps_PU_RL = [{'x':p['RU_adj']['x'], 'y':p['RU_adj']['y'] - 20},
                                 p['RU_adj'], p['center'], p['RL_adj'], 
                                 {'x': p['RL_adj']['x'] - 20, 'y': p['RL_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PU_RL, shared_pen)

                    cps_PD_PL = [{'x': p['RL_adj']['x'] - 20, 'y': p['RL_adj']['y']}, 
                                 p['RL_adj'], p['center'], p['RD_adj'], 
                                 {'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}]
                    self._draw_spline_turn_qt(painter, cps_PD_PL, shared_pen)

                    cps_PD_PR = [{'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}, 
                                 p['RD_adj'], p['center'], p['RR_adj'], 
                                 {'x': p['RR_adj']['x'] + 20, 'y': p['RR_adj']['y']}]
                    self._draw_spline_turn_qt(painter, cps_PD_PR, shared_pen)

                    cps_PR_PU = [{'x': p['RR_adj']['x'] + 20, 'y': p['RR_adj']['y']}, 
                                 p['RR_adj'], p['center'], p['RU_adj'], 
                                 {'x': p['RU_adj']['x'], 'y': p['RU_adj']['y'] - 20}]
                    self._draw_spline_turn_qt(painter, cps_PR_PU, shared_pen)
                else:
                    if inter['virtual_side'] == 'left':
                        cps_PL_PU = [{'x': p['RL_adj']['x'] - 20, 'y': p['RL_adj']['y']}, 
                                     p['RL_adj'], p['center'], p['RU_adj'], 
                                     {'x': p['RU_adj']['x'], 'y': p['RU_adj']['y'] - 20}]
                        self._draw_spline_turn_qt(painter, cps_PL_PU, shared_pen)

                        cps_PL_PD = [{'x': p['RL_adj']['x'] - 20, 'y': p['RL_adj']['y']}, 
                                     p['RL_adj'], p['center'], p['RD_adj'], 
                                     {'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}]
                        self._draw_spline_turn_qt(painter, cps_PL_PD, shared_pen)
                    else:
                        cps_PR_PU = [{'x': p['RR_adj']['x'] + 20, 'y': p['RR_adj']['y']}, 
                                     p['RR_adj'], p['center'], p['RU_adj'], 
                                     {'x': p['RU_adj']['x'], 'y': p['RU_adj']['y'] - 20}]
                        self._draw_spline_turn_qt(painter, cps_PR_PU, shared_pen)

                        cps_PR_PD = [{'x': p['RR_adj']['x'] + 20, 'y': p['RR_adj']['y']}, 
                                     p['RR_adj'], p['center'], p['RD_adj'], 
                                     {'x': p['RD_adj']['x'], 'y': p['RD_adj']['y'] + 20}]
                        self._draw_spline_turn_qt(painter, cps_PR_PD, shared_pen)

class CentralAMRAgentItemView(RobotItemView):
    def __init__(self, model: CentralAMRAgentModel, robot_size_px: float):
        super().__init__(model, robot_size_px)

    def boundingRect(self):
        extra = self.model.markov_radius + 5
        base_rect_half_size = (self.robot_size / 2) + extra
        return QRectF(-base_rect_half_size, -base_rect_half_size, 2 * base_rect_half_size, 2 * base_rect_half_size)

    def paint(self, painter, option, widget):
        if self.model.markov_radius > 0:
            painter.setPen(QPen(Qt.PenStyle.NoPen))
            painter.setBrush(QBrush(const.MARKOV_BLANKET_COLOR))
            painter.drawEllipse(QPointF(0, 0), self.model.markov_radius, self.model.markov_radius)
        
        super().paint(painter, option, widget)

class DebugIntersectionItem(QGraphicsItem):
    def __init__(self, intersections: list, map_rect: QRectF):
        super().__init__()
        self.intersections = intersections
        self._bounding_rect = map_rect
        self.setZValue(2) 

        self.pens = {
            'green': QPen(QColor("green"), 0.5),
            'red': QPen(QColor("red"), 0.5),
            'center': QPen(QColor("black"), 0.75),
            'adjacent': QPen(QColor("gray"), 0.5, Qt.PenStyle.DotLine)
        }
        self.brushes = {
            'green': QBrush(QColor("lightgreen")),
            'red': QBrush(QColor("#FF7F7F")), 
            'center': QBrush(QColor("black")),
            'adjacent': QBrush(QColor("lightgray"))
        }
        self.font = QFont("Arial", 5)

    def boundingRect(self):
        return self._bounding_rect

    def paint(self, painter, option, widget):
        painter.setFont(self.font)

        for inter in self.intersections:
            points_dict = inter.get('points', {})
            
            for p_key, p_data in points_dict.items():
                point_pos = QPointF(p_data['x'], p_data['y'])
                point_type = p_data.get('type', 'center') if 'adj' not in p_key else 'adjacent'
                
                pen = self.pens.get(point_type, self.pens['center'])
                brush = self.brushes.get(point_type, self.brushes['center'])
                painter.setPen(pen)
                painter.setBrush(brush)

                painter.drawEllipse(point_pos, 2, 2)
                
class LaneNodeItem(QGraphicsItem):
    def __init__(self, node_positions: dict, map_rect: QRectF):
        super().__init__()
        self.node_positions = node_positions
        self._bounding_rect = map_rect
        self.setZValue(2) 

        self.brush = QBrush(QColor("orange"))
        self.pen = QPen(QColor("darkred"), 0.5)
        self.font = QFont("Arial", 6)

    def boundingRect(self):
        return self._bounding_rect

    def paint(self, painter, option, widget):
        painter.setPen(self.pen)
        painter.setFont(self.font)

        for node_id, pos in self.node_positions.items():
            painter.setBrush(self.brush)
            painter.drawEllipse(pos, 2, 2)
            
            painter.setBrush(Qt.BrushStyle.NoBrush)
            text_pos = QPointF(pos.x() + 4, pos.y() + 4)
            painter.drawText(text_pos, node_id)

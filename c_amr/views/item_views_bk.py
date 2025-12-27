# c_amr/views/item_views.py
# Chứa tất cả các lớp QGraphicsItem để hiển thị các đối tượng từ model.
# Các lớp này chỉ chịu trách nhiệm vẽ, không chứa logic nghiệp vụ.

import math
import bisect
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsEllipseItem, QGraphicsItem
from PyQt5.QtGui import QBrush, QPen, QFont, QPolygonF, QPainterPath, QColor
from PyQt5.QtCore import QRectF, QPointF, Qt, QSizeF

from c_amr import constants as const
from c_amr.models.robot_model import RobotModel, CentralAMRAgentModel
from c_amr.models.obstacle_model import ObstacleModel
from c_amr.models.map_elements import NodeModel, StationModel

class ObstacleItemView(QGraphicsRectItem):
    """
    Lớp hiển thị cho một ObstacleModel.
    """
    def __init__(self, model: ObstacleModel):
        super().__init__(model.full_rect)
        self.model = model
        self.setPen(QPen(Qt.NoPen))

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
    """
    Lớp hiển thị cơ sở cho một RobotModel.
    """
    def __init__(self, model: RobotModel, robot_size_px: float):
        super().__init__()
        self.model = model
        self.robot_size = robot_size_px
        self.setPos(model.pos)
        self.setZValue(1)

    def boundingRect(self):
        font_height = 12 
        battery_bar_height = 5
        padding = 4
        total_height = self.robot_size + padding + font_height + padding + battery_bar_height
        return QRectF(-5, -5, self.robot_size + 10, total_height + 5)

    def shape(self):
        path = QPainterPath()
        path.addEllipse(0, 0, self.robot_size, self.robot_size)
        return path

    def paint(self, painter, option, widget):
        state_colors = {
            "IDLE": const.STATE_COLOR_IDLE, 
            "MOVING": const.STATE_COLOR_MOVING,
            "CHARGING": const.STATE_COLOR_CHARGING, 
            "Low Battery": const.STATE_COLOR_LOW_BATTERY
        }
        color = state_colors.get(self.model.state, QColor("gray"))

        body_rect = QRectF(0, 0, self.robot_size, self.robot_size)
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(Qt.NoPen))
        painter.drawEllipse(body_rect)

        if self.model.dx != 0 or self.model.dy != 0:
            painter.setBrush(QBrush(Qt.white))
            angle = math.degrees(math.atan2(self.model.dy, self.model.dx))
            center = body_rect.center()
            painter.save()
            painter.translate(center)
            painter.rotate(angle)
            arrow_scale = self.robot_size / 2.0
            points = [QPointF(arrow_scale * 0.6, 0), QPointF(-arrow_scale * 0.3, -arrow_scale * 0.5),
                      QPointF(0, 0), QPointF(-arrow_scale * 0.3, arrow_scale * 0.5)]
            arrow = QPolygonF(points)
            painter.drawPolygon(arrow)
            painter.restore()

        painter.setFont(const.ROBOT_ID_FONT)
        painter.setPen(QPen(Qt.black))
        id_text = str(self.model.robot_id)
        font_metrics = painter.fontMetrics()
        text_width = font_metrics.horizontalAdvance(id_text)
        text_height = font_metrics.height()
        text_x = (self.robot_size - text_width) / 2
        text_y = self.robot_size + text_height
        painter.drawText(QPointF(text_x, text_y), id_text)

class CentralAMRItemView(RobotItemView):
    def __init__(self, model: CentralAMRAgentModel, robot_size_px: float):
        super().__init__(model, robot_size_px)

    def boundingRect(self):
        extra = self.model.markov_radius + 5
        base_rect = super().boundingRect()
        return base_rect.adjusted(-extra, -extra, extra, extra)

    def paint(self, painter, option, widget):
        if self.model.markov_radius > 0:
            painter.setPen(QPen(Qt.NoPen))
            painter.setBrush(QBrush(const.MARKOV_BLANKET_COLOR))
            center = QPointF(self.robot_size / 2, self.robot_size / 2)
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
        painter.setPen(QPen(Qt.black))
        id_text = str(self.model.station_id)
        font_metrics = painter.fontMetrics()
        text_width = font_metrics.horizontalAdvance(id_text)
        text_height = font_metrics.height()
        text_x = -text_width / 2
        text_y = (self.rect().height() / 2) + text_height
        painter.drawText(QPointF(text_x, text_y), id_text)

class TrafficNetworkItem(QGraphicsItem):
    """
    Một QGraphicsItem duy nhất để vẽ toàn bộ mạng lưới giao thông, bao gồm cả các khúc cua.
    """
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.setZValue(-1)

    def boundingRect(self):
        return self.model.map_rect

    @staticmethod
    def get_y_at_x(path_data, x_coord):
        if not path_data or not path_data.get('x_points'): return None
        x_points, y_coords = path_data['x_points'], path_data['y_coords']
        if not (x_points[0] <= x_coord <= x_points[-1]): return None
        k = bisect.bisect_right(x_points, x_coord) - 1
        return y_coords[k] if 0 <= k < len(y_coords) else None

    @staticmethod
    def get_gap_at_x(path_data, x_coord):
        if not path_data or not path_data.get('x_points'): return 0
        x_points = path_data.get('x_points', [])
        gaps = path_data.get('gaps', [])
        if not x_points or not (x_points[0] <= x_coord <= x_points[-1]): return 0
        k = bisect.bisect_right(x_points, x_coord) - 1
        return gaps[k] if 0 <= k < len(gaps) and gaps[k] is not None else 0

    def paint(self, painter, option, widget):
        params = self.model.traffic_config_params
        if not params:
            return

        NARROW_LANE_THRESHOLD_PX = params.get("NARROW_LANE_THRESHOLD_PX", 60.0)
        STROKE_WIDTH_TRAFFIC = const.TRAFFIC_PEN_WIDTH

        red_pen = QPen(QColor("red"), STROKE_WIDTH_TRAFFIC)
        green_pen = QPen(QColor("green"), STROKE_WIDTH_TRAFFIC)
        purple_pen = QPen(QColor("purple"), STROKE_WIDTH_TRAFFIC)
        blue_pen = QPen(QColor("blue"), STROKE_WIDTH_TRAFFIC) 
        divider_pen = QPen(QColor("black"), const.DIVIDER_PEN_WIDTH, Qt.DashLine)

        self._draw_straight_lanes(painter, red_pen, green_pen, purple_pen, divider_pen, NARROW_LANE_THRESHOLD_PX)
        self._draw_intersections(painter, red_pen, green_pen, blue_pen, divider_pen, NARROW_LANE_THRESHOLD_PX)

    def _draw_straight_lanes(self, painter, red_pen, green_pen, purple_pen, divider_pen, threshold):
        # Vẽ các hành lang ngang
        for path_data in self.model.horizontal_paths.values():
            x_points, y_coords, gaps = path_data.get('x_points', []), path_data.get('y_coords', []), path_data.get('gaps', [])
            for i in range(len(gaps)):
                if gaps[i] is None or y_coords[i] is None: continue
                start_x, end_x, y_current, gap_current = x_points[i], x_points[i+1], y_coords[i], gaps[i]
                
                draw_start_x, draw_end_x = start_x, end_x
                if any(abs(start_x - ix) < 1 and self._is_wide_intersection(ix, y_current, threshold) for ix, iy in self.model.intersections):
                    width_v = self.model.vertical_corridors.get(start_x, 0)
                    draw_start_x = start_x + width_v / 4.0
                if any(abs(end_x - ix) < 1 and self._is_wide_intersection(ix, y_current, threshold) for ix, iy in self.model.intersections):
                    width_v = self.model.vertical_corridors.get(end_x, 0)
                    draw_end_x = end_x - width_v / 4.0

                if draw_end_x <= draw_start_x: continue

                if gap_current < threshold:
                    painter.setPen(purple_pen)
                    painter.drawLine(QPointF(draw_start_x, y_current), QPointF(draw_end_x, y_current))
                else:
                    L = gap_current / 4.0
                    painter.setPen(green_pen); painter.drawLine(QPointF(draw_end_x, y_current - L), QPointF(draw_start_x, y_current - L))
                    painter.setPen(red_pen); painter.drawLine(QPointF(draw_start_x, y_current + L), QPointF(draw_end_x, y_current + L))
                    painter.setPen(divider_pen); painter.drawLine(QPointF(draw_start_x, y_current), QPointF(draw_end_x, y_current))

        # Vẽ các hành lang dọc
        for x_corridor, width in self.model.vertical_corridors.items():
            y_events = {0, self.model.map_rect.height()}
            for cx, cy in self.model.intersections:
                if abs(cx - x_corridor) < 1: y_events.add(cy)
            sorted_y = sorted(list(y_events))
            for i in range(len(sorted_y) - 1):
                start_y, end_y = sorted_y[i], sorted_y[i+1]
                
                draw_start_y, draw_end_y = start_y, end_y
                h_path_start = next((p for p in self.model.horizontal_paths.values() if self.get_y_at_x(p, x_corridor) is not None and abs(self.get_y_at_x(p, x_corridor) - start_y) < 1), None)
                if h_path_start and self._is_wide_intersection(x_corridor, start_y, threshold):
                    gap_h = self.get_gap_at_x(h_path_start, x_corridor)
                    draw_start_y = start_y + gap_h / 4.0
                h_path_end = next((p for p in self.model.horizontal_paths.values() if self.get_y_at_x(p, x_corridor) is not None and abs(self.get_y_at_x(p, x_corridor) - end_y) < 1), None)
                if h_path_end and self._is_wide_intersection(x_corridor, end_y, threshold):
                    gap_h = self.get_gap_at_x(h_path_end, x_corridor)
                    draw_end_y = end_y - gap_h / 4.0

                if draw_end_y <= draw_start_y: continue

                if width < threshold:
                    painter.setPen(purple_pen); painter.drawLine(QPointF(x_corridor, draw_start_y), QPointF(x_corridor, draw_end_y))
                else:
                    L = width / 4.0
                    painter.setPen(red_pen); painter.drawLine(QPointF(x_corridor - L, draw_start_y), QPointF(x_corridor - L, draw_end_y))
                    painter.setPen(green_pen); painter.drawLine(QPointF(x_corridor + L, draw_end_y), QPointF(x_corridor + L, start_y))
                    painter.setPen(divider_pen); painter.drawLine(QPointF(x_corridor, draw_start_y), QPointF(x_corridor, draw_end_y))

    def _is_wide_intersection(self, cx, cy_approx, threshold):
        """Kiểm tra xem một giao lộ có đủ rộng không."""
        h_path = next((p for p in self.model.horizontal_paths.values() if self.get_y_at_x(p, cx) is not None and abs(self.get_y_at_x(p, cx) - cy_approx) < 1), None)
        if not h_path: return False
        gap_h = self.get_gap_at_x(h_path, cx)
        width_v = self.model.vertical_corridors.get(cx, 0)
        return gap_h >= threshold and width_v >= threshold

    def _draw_intersections(self, painter, red_pen, green_pen, blue_pen, divider_pen, threshold):
        """Vẽ các đường đi thẳng, rẽ trái và rẽ phải tại các ngã tư."""
        for cx, cy_approx in self.model.intersections:
            if not self._is_wide_intersection(cx, cy_approx, threshold): continue
            
            h_path = next(p for p in self.model.horizontal_paths.values() if self.get_y_at_x(p, cx) is not None and abs(self.get_y_at_x(p, cx) - cy_approx) < 1)
            cy = self.get_y_at_x(h_path, cx)
            gap_h, width_v = self.get_gap_at_x(h_path, cx), self.model.vertical_corridors.get(cx, 0)
            L_h, L_v = gap_h / 4.0, width_v / 4.0

            # 1. Vẽ các đoạn đi thẳng qua ngã tư
            painter.setPen(red_pen); painter.drawLine(QPointF(cx - L_v, cy + L_h), QPointF(cx + L_v, cy + L_h))
            painter.setPen(green_pen); painter.drawLine(QPointF(cx + L_v, cy - L_h), QPointF(cx - L_v, cy - L_h))
            painter.setPen(red_pen); painter.drawLine(QPointF(cx - L_v, cy - L_h), QPointF(cx - L_v, cy + L_h))
            painter.setPen(green_pen); painter.drawLine(QPointF(cx + L_v, cy + L_h), QPointF(cx + L_v, cy - L_h))

            # 2. Vẽ các cung rẽ (sử dụng blue_pen)
            painter.setPen(blue_pen)
            turn_path = QPainterPath()
            
            # --- CUNG RẼ PHẢI (Đường nhánh - Slip Lanes) ---
            # North -> West (Top-Left corner)
            path = QPainterPath(QPointF(cx - L_v, cy - L_h - L_h))
            path.quadTo(QPointF(cx - L_v, cy - L_h), QPointF(cx - L_v - L_v, cy - L_h))
            turn_path.addPath(path)
            
            # East -> North (Top-Right corner)
            path = QPainterPath(QPointF(cx + L_v + L_v, cy - L_h))
            path.quadTo(QPointF(cx + L_v, cy - L_h), QPointF(cx + L_v, cy - L_h - L_h))
            turn_path.addPath(path)

            # South -> East (Bottom-Right corner)
            path = QPainterPath(QPointF(cx + L_v, cy + L_h + L_h))
            path.quadTo(QPointF(cx + L_v, cy + L_h), QPointF(cx + L_v + L_v, cy + L_h))
            turn_path.addPath(path)

            # West -> South (Bottom-Left corner)
            path = QPainterPath(QPointF(cx - L_v - L_v, cy + L_h))
            path.quadTo(QPointF(cx - L_v, cy + L_h), QPointF(cx - L_v, cy + L_h + L_h))
            turn_path.addPath(path)

            # --- CUNG RẼ TRÁI (Vòng qua tâm - Dùng Cubic Bezier) ---
            # Eastbound -> Northbound
            path = QPainterPath(QPointF(cx - L_v, cy + L_h))
            c1 = QPointF(cx - L_v, cy)
            c2 = QPointF(cx, cy - L_h)
            end = QPointF(cx + L_v, cy - L_h)
            path.cubicTo(c1, c2, end)
            turn_path.addPath(path)

            # Northbound -> Westbound
            path = QPainterPath(QPointF(cx + L_v, cy + L_h))
            c1 = QPointF(cx, cy + L_h)
            c2 = QPointF(cx - L_v, cy)
            end = QPointF(cx - L_v, cy - L_h)
            path.cubicTo(c1, c2, end)
            turn_path.addPath(path)
            
            # Westbound -> Southbound
            path = QPainterPath(QPointF(cx + L_v, cy - L_h))
            c1 = QPointF(cx + L_v, cy)
            c2 = QPointF(cx, cy + L_h)
            end = QPointF(cx - L_v, cy + L_h)
            path.cubicTo(c1, c2, end)
            turn_path.addPath(path)

            # Southbound -> Eastbound
            path = QPainterPath(QPointF(cx - L_v, cy - L_h))
            c1 = QPointF(cx, cy - L_h)
            c2 = QPointF(cx + L_v, cy)
            end = QPointF(cx + L_v, cy + L_h)
            path.cubicTo(c1, c2, end)
            turn_path.addPath(path)

            painter.drawPath(turn_path)

            # 3. Vẽ đường phân cách tại ngã tư
            painter.setPen(divider_pen)
            painter.drawLine(QPointF(cx - L_v, cy), QPointF(cx + L_v, cy))
            painter.drawLine(QPointF(cx, cy - L_h), QPointF(cx, cy + L_h))

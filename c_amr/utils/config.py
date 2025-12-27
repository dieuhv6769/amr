class Config:
    """Stores all configuration parameters for map generation."""
    PX_PER_METER = 20
    MAP_SIZE_PX = 1000
    POSSIBLE_WIDTHS_M = [4.0]
    POSSIBLE_OBSTACLE_HEIGHTS_M = [3.0, 4.0]
    FIXED_ROW_HEIGHT_M = 3.0
    ROW_SPACINGS_M = [3.0]
    INTRA_ROW_SPACINGS_M = [3.0]
    WALL_MARGIN_M = 1.5
    ACTIVE_AREA_INSET_M = 1.0
    STATION_SIZE_M = 0.4
    STATION_INSET_M = 0.6
    STROKE_WIDTH_THIN = 0.25
    STROKE_WIDTH_BORDER = 0.5
    
    MIN_PATH_SPACE_M = 1.5
    NARROW_LANE_THRESHOLD_M = 3.0
    TRANSITION_WIDTH_M = 1.0
    STROKE_WIDTH_TRAFFIC = 0.25
    NODE_RADIUS = 2

    def __init__(self, map_width_m: float, map_height_m: float):
        """Initializes and calculates pixel values from meters."""
        self.MAP_WIDTH_PX = int(map_width_m * self.PX_PER_METER)
        self.MAP_HEIGHT_PX = int(map_height_m * self.PX_PER_METER)
        self.MARGIN_PX = self.WALL_MARGIN_M * self.PX_PER_METER
        self.POSSIBLE_WIDTHS_PX = [w * self.PX_PER_METER for w in self.POSSIBLE_WIDTHS_M]
        self.POSSIBLE_OBSTACLE_HEIGHTS_PX = [h * self.PX_PER_METER for h in self.POSSIBLE_OBSTACLE_HEIGHTS_M]
        self.FIXED_ROW_HEIGHT_PX = self.FIXED_ROW_HEIGHT_M * self.PX_PER_METER
        self.ROW_SPACINGS_PX = [s * self.PX_PER_METER for s in self.ROW_SPACINGS_M]
        self.INTRA_ROW_SPACINGS_PX = [s * self.PX_PER_METER for s in self.INTRA_ROW_SPACINGS_M]
        self.ACTIVE_AREA_INSET_PX = self.ACTIVE_AREA_INSET_M * self.PX_PER_METER
        self.STATION_SIZE_PX = self.STATION_SIZE_M * self.PX_PER_METER
        self.STATION_INSET_PX = self.STATION_INSET_M * self.PX_PER_METER
        
        self.MIN_PATH_SPACE_PX = self.MIN_PATH_SPACE_M * self.PX_PER_METER
        self.NARROW_LANE_THRESHOLD_PX = self.NARROW_LANE_THRESHOLD_M * self.PX_PER_METER
        self.TRANSITION_WIDTH_PX = self.TRANSITION_WIDTH_M * self.PX_PER_METER

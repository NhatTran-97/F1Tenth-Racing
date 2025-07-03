from config import config
from f1tenth_lane_tracking.Detection.Lanes.colour_segmentation import segment_lanes

def detect_lanes(frame):
    segment_lanes(frame, config.minArea_resized)
    
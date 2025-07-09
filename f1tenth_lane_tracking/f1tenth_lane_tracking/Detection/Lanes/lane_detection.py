from config import config
from Detection.Lanes.colour_segmentation import segment_lanes

def detect_lanes(frame):
    segment_lanes(frame, config.minArea_resized)
    
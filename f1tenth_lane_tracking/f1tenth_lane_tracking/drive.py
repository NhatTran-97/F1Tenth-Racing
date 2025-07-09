from Detection.Lanes.lane_detection import detect_lanes
import cv2

class Car():
    def drive_car(self, frame):
        if frame is None:
            print("⚠️ Warning: Received empty frame")
            return 
        
        img = cv2.resize(frame, (320, 240))
        detect_lanes(img)
        
        
        
        
        
        

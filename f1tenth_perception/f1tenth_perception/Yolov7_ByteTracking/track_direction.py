from algorithm.object_detector import YOLOv7
from utils.detections import draw

from tqdm import tqdm
import numpy as np
import cv2

yolov7 = YOLOv7()
yolov7.load('coco.weights', classes='coco.yaml', device='cpu') # use 'gpu' for CUDA GPU inference

webcam = cv2.VideoCapture(0,cv2.CAP_DSHOW)
if webcam.isOpened() == False:
	print('[!] error opening the webcam')

lines = {}

arrow_lines = []
arrow_line_length = 50
try:
    while webcam.isOpened():
        ret, frame = webcam.read()
        if ret == True:
            detections = yolov7.detect(frame, track=True); detected_frame_draw = draw(frame, detections)
            detected_frame = frame
            
            for detection in detections: 
                color = (np.random.randint(0,255), np.random.randint(0,255), np.random.randint(0,255))
                if 'id' in detection:
                    detection_id = detection['id']
                    if detection_id not in lines:
                         detection['color'] = color 
                         lines[detection_id] = {'points':[], 'arrows':[], 'color':color}
                    else:
                         detection['color'] = lines[detection_id]['color']
                         
                    lines[detection_id]['points'].append(np.array([detection['x'] + detection['width']/2, detection['y'] + detection['height']/2], np.int32))
                    points = lines[detection_id]['points']
                    if len(points) >= 2:
                        arrow_lines = lines[detection_id]['arrows']
                        if len(arrow_lines) > 0:
                            distance = np.linalg.norm(points[-1] - arrow_lines[-1]['end'])
                            if distance >= arrow_line_length:
                                start = np.rint(arrow_lines[-1]['end'] - ((arrow_lines[-1]['end'] - points[-1])/distance)*10).astype(int)
                                arrow_lines.append({'start':start, 'end':points[-1]})
                            else:
                                distance = 0
                                arrow_lines.append({'start':points[-2], 'end':points[-1]})
            for line in lines.values():
                arrow_lines = line['arrows']
                for arrow_line in arrow_lines:
                    detected_frame = cv2.arrowedLine(detected_frame, arrow_line['start'], arrow_line['end'], line['color'], 2, line_type=cv2.LINE_AA)
                                
            detected_frame = draw(frame, detections)
            
            #cv2.imshow('webcam', detected_frame)
            #cv2.waitKey(1)
        else:
            break
except KeyboardInterrupt:
    pass

webcam.release()
print('[+] webcam closed')
yolov7.unload()
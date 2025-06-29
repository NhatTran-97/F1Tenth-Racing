import warnings
warnings.filterwarnings('ignore')
from utils.general import check_img_size, non_max_suppression, scale_coords, crop
from models.experimental import attempt_load
from utils.torch_utils import select_device
from utils.detections import Detections
from utils.datasets import letterbox

from byte_tracker import BYTETracker
import numpy as np
import torch
import yaml

class YOLOv7:
    def __init__(self,conf_thres=0.25, iou_thres=0.45, img_size=640):
        self.settings = {
            'conf_thres':conf_thres,
            'iou_thres':iou_thres,
            'img_size':img_size,

        }
        self.tracker = BYTETracker()
        self.text_recognizer = None

    def load(self, weights_path, classes, device='cpu'):
        with torch.no_grad():
            self.device = select_device(device);
            self.model = attempt_load(weights_path, device=self.device)
            if device != 'cpu':
                self.model.half()
                self.model.to(self.device).eval()

            stride = int(self.model.stride.max());# print("stride: ", stride)
            self.imgsz = check_img_size(self.settings['img_size'], s=stride); #print("imgsz: ",self.imgsz)
            self.classes = yaml.load(open(classes), Loader=yaml.SafeLoader)['classes']; #print("self.classes: ",self.classes)
   
    def unload(self):
        if self.device.type != 'cpu':
            torch.cuda.empty_cache()
    def set(self, **config):
        for key in config.keys():
            if key in self.settings.keys():
                self.settings[key] = config[key]
            else:
                raise Exception(f'{key} is not a valid inference setting')

    def __parse_image(self, img):
        im0 = img.copy()
        img = letterbox(im0, self.imgsz, auto=self.imgsz != 1280)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.device.type != 'cpu' else img.float()
        img /= 255.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return im0, img
    def detect(self, img, track=False):
        with torch.no_grad():
            im0, img = self.__parse_image(img)
            #print("img: ",img.shape); print("img0: ", im0.shape)

            pred = self.model(img)[0]; 
            pred = non_max_suppression(pred, self.settings['conf_thres'], self.settings['iou_thres'])
            print("pred: ", pred)
            #nms = pred[0].detach().cpu().numpy();print("value: ", nms)    # print("nms: ", nms.shape); 
         
    
            raw_detection = np.empty((0,6), float)

            for det in pred:
                #print("det: ", det[:, :4]); 
                
                if len(det) > 0:
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round();
                    print("det: ",det)
                    #print("det2: ",det)
                    for *xyxy, conf, cls in reversed(det):
                        raw_detection = np.concatenate((raw_detection, [[int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), round(float(conf), 2), int(cls)]]))
                        print("raw_det: ", raw_detection)
                        #print("raw_detection_0: ",raw_detection.shape)

                    #print("raw_detection_1: ",raw_detection.shape); print()
            if track:
                raw_detection = self.tracker.update(raw_detection) # generate ID and bounding box

                #print("raw_detetcion: ",raw_detection)
            detections = Detections(raw_detection, self.classes, tracking=track).to_dict()
            print("raw_detetcion: ",raw_detection)
                        

        return detections





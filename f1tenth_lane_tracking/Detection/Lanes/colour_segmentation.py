#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np 

# White Regions Range

hls=0
src=0

hue_l = 0
lit_l = 255
sat_l = 0 

# Yellow Regions Range
hue_l_y = 30
hue_h_y = 33 
lit_l_y = 160
sat_l_y = 0

def maskextract():
    mask = color_segment(hls, (hue_l, lit_l, sat_l), 255)
    mask_y = color_segment(hls, (hue_l_y, lit_l_y, sat_l_y), (hue_h_y, 255))
    mask_ = mask != 0 
    dst = src * (mask_[:, :, None].astype(src.dtype))
    
    mask_y_ = mask_y != 0
    dst_Y = src * (mask_y_[:, :, None].astype(src.dtype))
    
    cv2.imshow('White_regions', dst)
    cv2.imshow('Yellow_regions', dst_Y)

def on_hue_low_change(val):
    global hue_l 
    hue_l = val 
    maskextract()

def on_lit_low_change(val):
    global hue_l
    lit_l = val 
    maskextract()

def on_sat_low_change(val):
    global sat_l
    sat_l = val
    maskextract()

def on_hue_low_y_change(val):
    global hue_l_y
    hue_l_y = val
    maskextract()

def on_hue_high_y_change(val):
    global hue_h_y
    hue_h_y = val
    maskextract()

def on_lit_low_y_change(val):
    global lit_l_y
    lit_l_y = val
    maskextract()

def on_sat_low_y_change(val):
    global sat_l_y
    sat_l_y = val
    maskextract()

cv2.namedWindow("White_regions")
cv2.namedWindow("Ywllow_regions")

cv2.createTrackbar("Hue_L", "white_regions", hue_l, 255, on_hue_low_change)
cv2.createTrackbar("Lit_L", "white_regions", lit_l, 255, on_lit_low_change)
cv2.createTrackbar("Sat_L", "white_regions", sat_l, 255, on_sat_low_change)

cv2.createTrackbar("Hue_L_Y", "yellow_regions", hue_l, 255, on_hue_low_y_change)
cv2.createTrackbar("Hue_H_Y", "yellow_regions", hue_l, 255, on_hue_high_y_change)
cv2.createTrackbar("Lit_L_Y", "yellow_regions", hue_l, 255, on_lit_low_y_change)
cv2.createTrackbar("Sat_L_Y", "yellow_regions", hue_l, 255, on_sat_low_y_change)

def color_segment( hls, lower_range, upper_range):
    mask_in_range = cv2.inRange(hls, lower_range, upper_range)
    kernerl = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    mask_dilated = cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernerl)
    return mask_dilated

def segment_lanes(frame, min_area):
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    
    # Segmenting White regions
    white_regions = color_segment(hls, np.array([hue_l, lit_l, sat_l]), np.array([255, 255, 255]))
    yellow_regions = color_segment(hls, np.array([hue_l_y, lit_l_y, sat_l_y]), np.array([hue_h_y, 255, 255]))
    
    cv2.imshow("white_regions: ", white_regions)
    cv2.imshow("yellow_regions: ", yellow_regions)
    cv2.waitKey(1)
    


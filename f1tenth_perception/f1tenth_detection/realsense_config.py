import pyrealsense2 as rs 
import numpy as np
import os 
import time

class Realsense_para:
    def __init__(self, ci, di, ex):
        self.color_inner_matrix = np.array([[ci.fx, 0, ci.ppx], [0, ci.fy, ci.ppx], [0,0,1]])
        self.depth_inner_matrix = np.array([[di.fx, 0, di.ppx], [0, di.fy, di.ppx], [0,0,1]])
        self.color_to_depth_rotation = np.array(ex.rotation).reshape(3, 3)
        self.color_to_depth_translation = np.array(ex.translation)
        # print("test: ",self.color_inner_matrix )
    def refresh_mat(self):
        self.frames = pipeline.wait_for_frames()
        self.depth = self.frames.get_depth_frame()
        self.color = self.frames.get_color_frame()
        hole_filling = rs.hole_filling_filter()
        self.depth = hole_filling.process(self.depth)
        
        self.depthmat = np.asanyarray(self.depth.get_data())
        self.colormat = np.asanyarray(self.color.get_data())
        distance = self.depth.get_distance(200,200)
        print("distance: ", distance)
        return self.depthmat, self.colormat



pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.enable_auto_exposure, True)
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()
depth_profile = depth.get_profile()
color_profile = color.get_profile()
print("depth_profile: ",depth_profile)
print("color_profile: ", color_profile)

cvsprofile = rs.video_stream_profile(color_profile)
dvsprofile = rs.video_stream_profile(depth_profile)

color_intrin = cvsprofile.get_intrinsics()
depth_intrin = dvsprofile.get_intrinsics()
extrin = depth_profile.get_extrinsics_to(color_profile)
print("color_intrin: ", color_intrin)
print("depth_intrin: ", depth_intrin)
print("extrin: ", extrin)

d435 = Realsense_para(color_intrin, depth_intrin, extrin)
pipeline.stop()

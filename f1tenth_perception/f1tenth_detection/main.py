import realsense_config as rs_config
import cv2
import numpy as np 

if __name__ == '__main__':
    
    rs_config.pipeline.start(rs_config.config)
    cv2.namedWindow('RealsenseRGB', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('RealsenseDEPTH', cv2.WINDOW_AUTOSIZE)
    
    while True:
        depth_mat, color_mat = rs_config.d435.refresh_mat()
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_mat, alpha=0.03), cv2.COLORMAP_JET)
        # print("data: ", color_mat)
            
        cv2.imshow('RealsenseRGB', rs_config.d435.colormat)
        cv2.imshow('RealsenseDEPTH', depth_mat)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
rs_config.pipeline.stop()
cv2.destroyAllWindows()